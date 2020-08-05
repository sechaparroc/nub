package ik.trik;

import nub.core.Graph;
import nub.core.Interpolator;
import nub.core.Node;
import nub.ik.solver.trik.implementations.IKSolver;
import nub.processing.Scene;
import nub.processing.TimingTask;
import processing.core.PApplet;
import processing.core.PShape;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.List;

public class LowerLimb extends PApplet {
  int numJoints = 20;
  float boneLength = 100;
  float radius = 10;
  float targetRadius = radius * 1.2f;

  Scene scene;
  Node target;
  Interpolator pathInterpolator;

  public void settings() {
    size(700, 700, P3D);
  }

  public void setup() {
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.setRadius(boneLength * numJoints);
    scene.fit();
    //Create a kinematic chain
    target = createTarget(radius * 1.5f);
    List<Node> skeleton = createSkeleton(scene, numJoints, boneLength, radius, color(255, 0, 255));
    //Add target
    //Place target at end effector position
    target.setPosition(skeleton.get(skeleton.size() - 1));
    //Define the target trajectory
    createPath(target, 9f, 7, 2, boneLength * numJoints * 0.2f, 10);

    //Create the IK solver
    IKSolver solver = new IKSolver(skeleton, IKSolver.HeuristicMode.COMBINED);
    solver.setTimesPerFrame(5);
    solver.setMaxIterations(5);
    solver.setMaxError(scene.radius() * 0.001f);
    solver.setTarget(skeleton.get(skeleton.size() - 1), target);

    List<Node> skeleton2 = createSkeleton(scene, numJoints, boneLength, radius, color(0, 255, 0));
    IKSolver solver2 = new IKSolver(skeleton2, IKSolver.HeuristicMode.COMBINED_TRIK);
    solver2.context().enableDelegation(true);

    //solver2.enableSmooth(true);
    solver2.setTimesPerFrame(10);
    solver2.setMaxIterations(10);
    solver2.setMaxError(scene.radius() * 0.001f);
    solver2.setTarget(skeleton2.get(skeleton2.size() - 1), target);

    TimingTask task = new TimingTask() {
      @Override
      public void execute() {
        solver.solve();
        solver2.solve();
      }
    };
    task.run(40);
  }

  public void draw() {
    background(0);
    lights();
    scene.render();
    stroke(255);
    pathInterpolator.enableHint(Interpolator.SPLINE);
  }

  public Node createTarget(float radius) {
    PShape sphere = createShape(SPHERE, radius);
    sphere.setFill(color(255, 0, 0));
    sphere.setStroke(false);
    return new Node(sphere);
  }

  public List<Node> createSkeleton(Scene scene, int numJoints, float boneLength, float radius, int col) {
    List<Node> skeleton = new ArrayList<Node>();
    Node root = new Node(pg ->{
        pg.pushStyle();
        pg.fill(col);
        pg.sphere(radius);
        pg.popStyle();
    });
    skeleton.add(root);
    Node prev = root;
    for (int i = 0; i < numJoints; i++) {
      Node joint = new Node();
      joint.enableHint(Node.BONE, col);
      joint.setReference(prev);
      joint.translate(0, -boneLength, 0);
      prev = joint;
      skeleton.add(joint);
    }
    root.translate(0, 0.7f * scene.radius(), 0);
    return skeleton;
  }

  //This part of the code defines a trajectory based on Lissajous curve
  public void createPath(Node target, float x_speed, float y_speed, float z_speed, float radius, float detail) {
    pathInterpolator = new Interpolator(target);
    float step = 360 / detail;
    for (float angle = 0; angle < 360 + step; angle += step) {
      float rad = radians(angle);
      float x = radius * cos(x_speed * rad);
      float y = radius * sin(y_speed * rad);
      float z = radius * sin(z_speed * rad);
      Node n = new Node();
      n.setPosition(target);
      n.translate(x, y + radius * 1.2f, z);
      pathInterpolator.addKeyFrame(n);
    }
    pathInterpolator.enableRecurrence(true);
    pathInterpolator.run();
  }


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

  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.trik.LowerLimb"});
  }

}
