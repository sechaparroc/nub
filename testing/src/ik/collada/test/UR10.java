package ik.collada.test;

import ik.basic.Util;
import ik.interactive.Target;
import nub.core.Graph;
import nub.core.Node;
import nub.core.constraint.Hinge;
import nub.ik.loader.collada.URDFLoader;
import nub.ik.loader.collada.data.Model;
import nub.ik.solver.Solver;
import nub.ik.solver.GHIK;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.processing.TimingTask;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Created by sebchaparr on 08/05/19.
 */
public class UR10 extends PApplet {
  Scene scene;
  String path = "/testing/data/dae/";
  String dae = "ur10_joint_limited_robot.dae";
  Model model;
  Solver solver;
  Vector base;
  Target target;


  //DEBUG VARIABLES
  boolean debug = true;
  boolean ccd = false;
  boolean solve = !debug;
  boolean show[] = new boolean[4];
  Random random = new Random();

  public void settings() {
    size(700, 700, P3D);
  }

  public void setup() {
    if (debug) {
      for (int i = 0; i < show.length; i++) show[i] = true;
    }

    //Joint.markers = true;
    randomSeed(14);
    this.g.textureMode(NORMAL);
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);

    model = URDFLoader.loadColladaModel(sketchPath() + path, dae, scene);


    model.printNames();
    target = new Target(scene.radius() * 0.01f);

    /*Adding constraints*/
    Node node_1 = model.skeleton().get("node1");
    Hinge hinge_1 = new Hinge(radians(180), radians(180), node_1.rotation(), new Vector(1, 0, 0), new Vector(0, 0, 1));
    node_1.setConstraint(hinge_1);


    Node node_3 = model.skeleton().get("node3");
    Hinge hinge_3 = new Hinge(radians(180), radians(180), node_3.rotation(), new Vector(1, 0, 0), new Vector(0, 1, 0));
    node_3.setConstraint(hinge_3);

    Node node_4 = model.skeleton().get("node4");
    Hinge hinge_4 = new Hinge(radians(180), radians(180), node_4.rotation(), new Vector(1, 0, 0), new Vector(0, 0, 1));
    node_4.setConstraint(hinge_4);

    Node node_6 = model.skeleton().get("node6");
    Hinge hinge_6 = new Hinge(radians(180), radians(180), node_6.rotation(), new Vector(0, 0, 1), new Vector(0, 1, 0));
    node_6.setConstraint(hinge_6);

    Node node_8 = model.skeleton().get("node8");
    Hinge hinge_8 = new Hinge(radians(180), radians(180), node_8.rotation(), new Vector(0, 0, 1), new Vector(0, 1, 0));
    node_8.setConstraint(hinge_8);

    Node node_9 = model.skeleton().get("node9");
    Hinge hinge_9 = new Hinge(radians(180), radians(180), node_9.rotation(), new Vector(1, 0, 0), new Vector(0, 0, 1));
    node_9.setConstraint(hinge_9);

    Node node_10 = model.skeleton().get("node10");
    Hinge hinge_10 = new Hinge(radians(180), radians(180), node_10.rotation(), new Vector(0, 0, 1), new Vector(0, 1, 0));
    node_10.setConstraint(hinge_10);

    //Cull unnecesary nodes
    model.skeleton().get("node2").cull = true;

    //Adding solver
    List<Node> branch = Node.path(model.skeleton().get("node1"), model.skeleton().get("node2"));


    if (!ccd) {
      solver = new GHIK(branch, GHIK.HeuristicMode.BFIK);
    } else {
      solver = new GHIK(branch, GHIK.HeuristicMode.ECTIK);
    }

    solver.setTimesPerFrame(10);
    solver.setMaxIterations(10);
    solver.setMaxError(scene.radius() * 0.01f);
    solver.setMinDistance(scene.radius() * 0.01f);
    solver.setTarget(branch.get(branch.size() - 1), target);
    target.setPosition(branch.get(branch.size() - 1).position().get());
    TimingTask task = new TimingTask() {
      @Override
      public void execute() {
        if (solve) solver.solve();
      }
    };
    task.run(40);
    base = model.skeleton().get("node1").reference().translation();


    scene.eye().rotate(new Quaternion(new Vector(1, 0, 0), PI / 2));
    scene.eye().rotate(new Quaternion(new Vector(0, 0, 1), PI));
    scene.setBounds(scene.radius() * 2f);
    scene.fit();
  }

  public void draw() {
    background(0);
    lights();
    scene.drawAxes();
    scene.render();
    scene.beginHUD();
    for (String s : model.skeleton().keySet()) {
      Node n = model.skeleton().get(s);
      if (n.cull || !n.tagging || debug) continue;
      Vector sc = scene.screenLocation(new Vector(), n);
      text(s, sc.x(), sc.y());
    }
    Util.printInfo(scene, solver, base);
    scene.endHUD();
  }

  @Override
  public void mouseMoved() {
    scene.mouseTag();
  }

  public void mouseDragged() {
    if (mouseButton == LEFT) {
      scene.mouseSpin();
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

  public void keyPressed() {
    if (key == 'S' || key == 's') {
      solve = !solve;
    } else if (key == 'a' || key == 'A') {
      solver.solve();
    } else if (key == ' ') {
      Node f = generateRandomReachablePosition(Node.path(model.skeleton().get("vkmodel0_node1"), model.skeleton().get("vkmodel0_node2")), scene.is3D());
      Vector delta = Vector.subtract(f.position(), target.position());
      target.setPosition(Vector.add(target.position(), delta));
    } else {
      try {
        int i = Integer.parseInt("" + key) - 1;
        show[i] = !show[i];
      } catch (Exception e) {
      }
    }
  }

  public Node generateRandomReachablePosition(List<? extends Node> original, boolean is3D) {
    ArrayList<? extends Node> chain = Util.detachedCopy(original);
    for (int i = 0; i < chain.size(); i++) {
      if (is3D)
        chain.get(i).rotate(new Quaternion(Vector.random(), (float) (random.nextGaussian() * random.nextFloat() * PI / 2)));
      else
        chain.get(i).rotate(new Quaternion(new Vector(0, 0, 1), (float) (random.nextGaussian() * random.nextFloat() * PI / 2)));
    }
    return chain.get(chain.size() - 1);
  }


  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.collada.test.UR10"});
  }
}
