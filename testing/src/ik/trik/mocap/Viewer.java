package ik.trik.mocap;

import ik.basic.Util;
import nub.core.Graph;
import nub.core.Node;
import nub.core.constraint.BallAndSocket;
import nub.core.constraint.Constraint;
import nub.ik.loader.bvh.BVHLoader;
import nub.ik.solver.trik.Tree;
import nub.ik.solver.trik.implementations.IKSolver;
import nub.ik.animation.Joint;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.core.PGraphics;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Viewer extends PApplet {
  //String path = "/testing/data/bvh/0007_Cartwheel001.bvh";
  boolean absolute = true;
  //String path = "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/cmu-mocap-master/data/001/01_02.bvh";
  String path = "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__SlowFly.bvh";
  Scene scene;
  BVHLoader parser;
  List<Skeleton> skeletons;

  boolean readNext = false;
  boolean solve = false;

  public void settings() {
    size(700, 700, P3D);
  }

  public void setup() {
    Joint.depth  = true;
    Joint.axes = true;
    Joint.drawCylinder = true;

    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.eye().rotate(new Quaternion(0, 0, PI));
    scene.setRadius(600);
    scene.fit(0);
    parser = new BVHLoader(absolute ? path : (sketchPath() + path), scene, null);
    parser.nextPose(true);
    parser.nextPose(true);
    parser.generateConstraints();
    System.out.println("Height : " + calculateHeight(parser));
    skeletons = new ArrayList<Skeleton>();

    skeletons.add(new Skeleton(parser, IKSolver.HeuristicMode.COMBINED, scene, 0, 255, 0, scene.radius() * 0.01f, new Vector(0,0,-scene.radius())));
    //skeletons.add(new Skeleton(parser, IKSolver.HeuristicMode.COMBINED, scene, 255,0,0, scene.radius() * 0.01f));
    //parser.root().cull(true);
    //skeleton._root.cull(true);
    //for(Skeleton sk : skeletons) sk._reference.cull(true);

  }

  public void draw() {
    background(0);
    ambientLight(102, 102, 102);
    lightSpecular(204, 204, 204);
    directionalLight(102, 102, 102, 0, 0, -1);
    specular(255, 255, 255);
    shininess(10);
    //Draw Constraints
    scene.drawAxes();
    scene.render();
    //skeletons.get(0).renderNames();
    if (readNext) {
      readNextPose();
    }

  }

  public void readNextPose() {
    parser.nextPose();

    for (Skeleton skeleton : skeletons) {
      Constraint c = skeleton._root.constraint();
      skeleton._root.setConstraint(null);
      skeleton._root.setPosition(skeleton._reference.worldLocation(parser.root().position().get()));
      skeleton._root.setOrientation(skeleton._reference.worldDisplacement(parser.root().orientation().get()));
      skeleton._root.setConstraint(c);

      for (Node joint : scene.branch(skeleton._root)) {
          if (joint.children() == null || joint.children().isEmpty()) {
          Node node = skeleton._jointToNode.get(joint);
          Node target = skeleton._targets.get(parser.joint().get(node.id()).name());
          if(target == null) continue;
          target.setPosition(skeleton._reference.worldLocation(node.position().get()));
          target.setOrientation(skeleton._reference.worldDisplacement(node.orientation().get()));
          //modify end effector rotation
          joint.setRotation(node.rotation());
        }
      }
      skeleton._solver.change(true);
      skeleton._solver.solve();
    }
  }

  public void keyPressed() {
    if (key == 'W' || key == 'w') {
      readNext = !readNext;
    }
    if (key == 'S' || key == 's') {
      readNextPose();
    }
    if(key == ' '){
        skeletons.get(0)._solver.change(true);
        skeletons.get(0)._solver.solve();
    }
  }

public void mousePressed(){
    prev = new Vector(mouseX, mouseY);
}


  @Override
  public void mouseMoved() {
    scene.mouseTag();
  }

  public void mouseDragged() {
    if (mouseButton == LEFT) {
      if(keyPressed) rotate(scene.node());
      else scene.mouseSpin();
    } else if (mouseButton == RIGHT) {
      scene.mouseTranslate();
    } else {
      scene.scale(scene.mouseDX());
    }
  }

  public void mouseWheel(MouseEvent event) {
    scene.scale(event.getCount() * 20);
  }

  public void mouseClicked(MouseEvent event) {
    if (event.getCount() == 2)
      if (event.getButton() == LEFT)
        scene.focus();
      else
        scene.align();
  }

  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.trik.mocap.Viewer"});
  }

  class Skeleton {
    Scene _scene;
    Node _reference;
    Joint _root;
    Tree _solver;
    BVHLoader _loader;
    HashMap<Joint, Node> _jointToNode;
    HashMap<String, Joint> _structure;
    HashMap<String, Node> _targets;
    float _radius;

    public Skeleton(BVHLoader loader, IKSolver.HeuristicMode mode, Scene scene, int red, int green, int blue, float radius, Vector offset) {
      _scene = scene;
      _radius = radius;
      _reference = new Node();
      _reference.setTranslation(offset);
      _reference.tagging = false;
      _createSkeleton(loader, red, green, blue, radius);
      _createSolver(mode);
    }

    protected void _createSkeleton(BVHLoader loader, int red, int green, int blue, float radius) {
      HashMap<Node, Joint> pairs = new HashMap<>();
      _structure = new HashMap<>();
      _jointToNode = new HashMap<>();
      for (Node node : loader.branch()) {
        Joint joint = new Joint(red, green, blue, radius);
        Joint reference = pairs.get(node.reference());
        if (reference != null) {
          joint.setReference(reference);
        }
        joint.setTranslation(node.translation().get());
        joint.setRotation(node.rotation().get());
        joint.setConstraint(node.constraint());
        pairs.put(node, joint);
        _structure.put(loader.joint().get(node.id()).name(), joint);

        if (node.reference() != null && node.reference().children().size() == 1) {
          //duplicateBone((Joint) joint.reference(), joint);
        }

        _jointToNode.put(joint, node);
      }
      _root = pairs.get(loader.root());
      _root.setReference(_reference);
      _root.setRoot(true);
    }


    protected void duplicateBone(Joint j_i, Joint j_i1) {
      Joint j_mid = new Joint(j_i.red(), j_i.green(), j_i.blue(), j_i.radius());
      Vector v = j_i1.translation().get();
      j_mid.setReference(j_i);
      j_mid.setTranslation(Vector.multiply(v, 0.5f));
      BallAndSocket bs = new BallAndSocket(radians(20), radians(20));
      bs.setRestRotation(j_mid.rotation().get(), v.orthogonalVector(), v);
      j_mid.setConstraint(bs);
      //Set j_i1
      Constraint c_i1 = j_i1.constraint();
      Vector pos = j_i1.position();
      Quaternion or = j_i1.orientation();
      j_i1.setConstraint(null);
      j_i1.setReference(j_mid);
      j_i1.setPosition(pos);
      j_i1.setOrientation(or);
      j_i1.setConstraint(c_i1);

    }

    protected void _createSolver(IKSolver.HeuristicMode mode) {
      _solver = new Tree(_root, mode);
      _solver.setMaxError(0.01f);
      //_solver.setDirection(true);
      //_solver.setSearchingAreaRadius(0.3f, true);
      //_solver.setOrientationWeight(0.5f);

      _solver.setTimesPerFrame(10);
      _solver.setMaxIterations(10);
      _solver.setChainMaxIterations(3);

      //add task to scene

      /*TimingTask task = new TimingTask() {
        @Override
        public void execute() {
          _solver.solve();
        }
      };
      task.run(40);*/

      _targets = new HashMap<>();

      for (Map.Entry<String, Joint> entry : _structure.entrySet()) {
        Node node = entry.getValue();
        if (node.children() == null || node.children().isEmpty()) {
          node.tagging = false;
          Node target = Util.createTarget(scene, _radius);
          target.setReference(_reference);
          target.setPosition(node.position().get());
          target.setOrientation(node.orientation().get());
          _solver.addTarget(node, target);
          _targets.put(entry.getKey(), target);
        }
      }

    }


    protected void renderNames() {
      _scene.beginHUD();
      PGraphics pg = _scene.context();
      pg.pushStyle();
      pg.noLights();
      pg.fill(46, 76, 125);
      for (Map.Entry<String, Joint> entry : _structure.entrySet()) {
        Vector scrLocation = _scene.screenLocation(entry.getValue());
        pg.text(entry.getKey() + "\n" + String.format("x %.2f", entry.getValue().position().x())
            + "\n" + String.format("y %.2f", entry.getValue().position().y())
            + "\n" + String.format("z %.2f", entry.getValue().position().z()), scrLocation.x(), scrLocation.y());
      }

      for (Map.Entry<String, Node> entry : _targets.entrySet()) {
        Vector scrLocation = _scene.screenLocation(entry.getValue());
        pg.text(entry.getKey() + "\n" + String.format("x %.2f", entry.getValue().position().x())
            + "\n" + String.format("y %.2f", entry.getValue().position().y())
            + "\n" + String.format("z %.2f", entry.getValue().position().z()), scrLocation.x(), scrLocation.y());
      }

      pg.popStyle();
      _scene.endHUD();
    }

  }

  float calculateHeight(BVHLoader parser){ //calculates the height of the skeleton
    Vector min = new Vector(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
    Vector max = Vector.multiply(min, -1);
    for(Node n : parser.branch()){
      Vector pos = parser.root().location(n);
      if(max.x() < pos.x()) max.setX(pos.x());
      if(max.y() < pos.y()) max.setY(pos.y());
      if(max.z() < pos.z()) max.setZ(pos.z());
      if(min.x() > pos.x()) min.setX(pos.x());
      if(min.y() > pos.y()) min.setY(pos.y());
      if(min.z() > pos.z()) min.setZ(pos.z());
    }
    float mX = max.x() - min.x();
    float mY = max.y() - min.y();
    float mZ = max.z() - min.z();
    return Math.max(Math.max(mX, mY), mZ);
  }


  Vector prev;
  public void rotate(Node node){
    if(node == null || node.reference() == null) return;
    Node parent = node.reference();
    Vector screenDis = new Vector(mouseX - prev.x(), mouseY - prev.y());
    Vector dis = scene.displacement(screenDis, parent);
    Quaternion rot = new Quaternion(node.translation(), Vector.add(node.translation(), dis));
    parent.rotate(rot);
  }
}
