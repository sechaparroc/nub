package ik.interactive;

import nub.core.Graph;
import nub.core.Interpolator;
import nub.core.Node;
import nub.core.constraint.BallAndSocket;
import nub.core.constraint.Hinge;
import nub.ik.solver.Solver;
import nub.ik.solver.geometric.TreeSolver;
import nub.ik.solver.trik.Tree;
import nub.ik.solver.trik.implementations.IKSolver;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by sebchaparr on 27/10/18.
 * */

 public class SkeletonBuilder extends PApplet {
  //TODO Check Fitting curve method for target path
  Scene scene;
  Scene[] views;
  boolean showPath = false, showLast = false, debug = true;
  //focus;
  //OptionPanel panel;
  //PGraphics canvas1;

  FitCurve fitCurve;

  float radius = 30;
  int w = 1000, h = 700;
  //Create different skeletons to interact with

  //Choose FX2D, JAVA2D, P2D or P3D
  String renderer = P3D;

  //Constraint Parameters

  float minAngle = radians(60);
  float maxAngle = radians(60);

  List<Target> targets = new ArrayList<Target>();


  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.interactive.SkeletonBuilder"});
  }

  public void settings() {
    size(w, h, renderer);
  }

  public void setup() {
    scene = new Scene(this);
    if (scene.is3D()) scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.setBounds(800);
    scene.fit();
    scene.enableHint(Graph.BACKGROUND | Graph.AXES);

    new InteractiveJoint(true, color(random(255), random(255), random(255)), radius, true);
  }

  public void draw() {
    ambientLight(102, 102, 102);
    lightSpecular(204, 204, 204);
    directionalLight(102, 102, 102, 0, 0, -1);
    specular(255, 255, 255);
    shininess(10);
    //canvas1.stroke(255,0,0);
    stroke(255);
    stroke(255, 0, 0);
    //scene.drawAxes();
    scene.render();
    for (Target target : targets) {
      if (showPath) target._interpolator.enableHint(Interpolator.SPLINE);
      if (showLast) {
        pushStyle();
        colorMode(HSB);
        strokeWeight(radius / 3.f);
        Vector p = !target.last().isEmpty() ? target.last().get(0) : null;
        for (int i = 0; i < target.last().size(); i++) {
          Vector v = target.last().get(i);
          fill((frameCount + i) % 255, 255, 255, 100);
          stroke((frameCount + i) % 255, 255, 255, 100);
          line(v.x(), v.y(), v.z(), p.x(), p.y(), p.z());
          p = v;
        }
        popStyle();
      }
    }

    if (fitCurve != null)
      if (fitCurve._interpolator != null)
        fitCurve._interpolator.enableHint(Interpolator.SPLINE);

    scene.beginHUD();
    if (fitCurve != null) fitCurve.drawCurves(scene.context());
    scene.endHUD();

    if(debug && solve){
      for(Solver s: solvers){
        s.solve();
      }
    }

  }

  //mouse events
  @Override
  public void mouseMoved() {
    if (!mousePressed) {
      scene.mouseTag();
    }
  }

  public void mouseDragged(MouseEvent event) {
    if (mouseButton == RIGHT && event.isControlDown()) {
      Vector vector = new Vector(scene.mouseX(), scene.mouseY());
      if (scene.node() != null)
        scene.node().interact("OnAdding", scene, vector);
    } else if (mouseButton == LEFT) {
      if (event.isControlDown() && fitCurve != null) {
        if (fitCurve.started()) {
          fitCurve.add(mouseX, mouseY);
          fitCurve.fitCurve();
        }
      } else {
        scene.spin(scene.pmouseX(), scene.pmouseY(), scene.mouseX(), scene.mouseY());
      }
    } else if (mouseButton == RIGHT) {
      scene.translate(scene.mouseX() - scene.pmouseX(), scene.mouseY() - scene.pmouseY(), 0, 0);
      Target.multipleTranslate(scene);
    } else if (mouseButton == CENTER) {
      scene.scale(scene.mouseDX());
    } else if (scene.node() != null)
      scene.node().interact("Reset");
    if (!Target.selectedTargets().contains(scene.node())) {
      Target.clearSelectedTargets();
    }
  }

  public void mousePressed(MouseEvent event) {
    if (event.isControlDown() && event.getButton() == LEFT) {
      //Reset Curve
      fitCurve = new FitCurve();
      fitCurve.setStarted(true);
    }
  }

  public void mouseReleased(MouseEvent event) {
    if(event.isControlDown() && event.getButton() == LEFT){
      //Reset Curve
      if(fitCurve != null){
        fitCurve.setStarted(false);
        fitCurve.printCurves();
        fitCurve.getCatmullRomCurve(scene, scene.node(), 0);
        fitCurve._interpolator.run();
        fitCurve._interpolator.enableHint(Interpolator.SPLINE);
      }
  }


    //mouse = scene.location(mouse);
    //mouse = Vector.projectVectorOnPlane(mouse, scene.viewDirection());
    //mouse.add(scene.defaultFrame().position());
    Vector vector = new Vector(scene.mouseX(), scene.mouseY());
    if (scene.node() != null)
      if (scene.node() instanceof InteractiveJoint)
        scene.node().interact("Add", scene, vector);
        //else focus.trackedFrame().interact("Add", vector, false);
      else {
        if (fitCurve != null) {
          fitCurve.setStarted(false);
          fitCurve.getCatmullRomCurve(scene, scene.node(), 0);
          //fitCurve._interpolator.run();
          scene.node().interact("AddCurve", scene, fitCurve);
        }
      }
    fitCurve = null;

  }

  public void mouseWheel(MouseEvent event) {
    scene.scale(event.getCount() * 20);
  }

  public void mouseClicked(MouseEvent event) {
    if (event.getButton() == LEFT) {
      if (event.getCount() == 1) {
        //panel.setFrame(scene.trackedFrame());
        if (event.isControlDown()) {
          if (scene.node() != null)
            scene.node().interact("KeepSelected");
        }
      } else if (event.getCount() == 2) {
        if (event.isShiftDown())
          if (scene.node() != null)
            scene.node().interact("Remove");
          else
            scene.focus();
      } else {
        scene.align();
      }
    }
  }

  boolean solve = false;

  public void keyPressed() {
    if (key == '+') {
      new InteractiveJoint(true, color(random(255), random(255), random(255)), radius, true);
    }
    if (key == 'A' || key == 'a') {
      addTreeSolver();
    }
    if (key == 'C' || key == 'c') {
      addConstraint(scene.node(), false);
    }
    if (key == 'H' || key == 'h') {
      addConstraint(scene.node(), true);
    }

    if (key == 'S' || key == 's') {
      minAngle += radians(5);
      if (minAngle >= radians(170)) minAngle = radians(170);
      System.out.println("minAngle : " + degrees(minAngle));
    }
    if (key == 'D' || key == 'd') {
      minAngle -= radians(5);
      if (minAngle <= radians(0)) minAngle = radians(0);
      System.out.println("minAngle : " + degrees(minAngle));
    }
    if (key == 'F' || key == 'f') {
      maxAngle += radians(5);
      if (maxAngle >= radians(170)) maxAngle = radians(170);
      System.out.println("maxAngle : " + degrees(maxAngle));
    }
    if (key == 'G' || key == 'g') {
      maxAngle -= radians(5);
      if (maxAngle <= radians(0)) maxAngle = radians(0);
      System.out.println("maxAngle : " + degrees(maxAngle));
    }
    if (key == 'i' || key == 'I') {
      printTree(scene.node(), "");
    }

    if (key == ' ') {
      for (Target target : targets) {
        target._interpolator.run();
      }
    }
    if (key == '1') {
      if (debug) {
        for (Solver s : solvers) {
          System.out.println("Entra!!");
          s.solve();
        }
      }
    }
    if (key == '2') {
      if (debug) solve = !solve;
    }
    if (key == '3') {
      showLast = !showLast;
    }
    if (key == '4') {
      showPath = !showPath;
    }
    if (key == '5') {
      scene.toggleHint(Graph.GRID);
    }

    if (key == 'r' || key == 'R') {
      for (Target target : targets) {
        target._interpolator.enableRecurrence(!target._interpolator.isRecurrent());
      }
    }

    if (key == CODED) {
      if (keyCode == SHIFT) {
        for (Solver solver : scene.treeSolvers()) {
          if (solver instanceof Tree) ((Tree) solver).setDirection(true);
        }
      }
    }
  }

  public void keyReleased() {
    if (key == CODED) {
      if (keyCode == SHIFT) {
        for (Solver solver : scene.treeSolvers()) {
          if (solver instanceof Tree) ((Tree) solver).setDirection(false);
        }
      }
    }
  }

  public void findEndEffectors(Node frame, List<Node> endEffectors) {
    if (frame.children().isEmpty()) {
      endEffectors.add(frame);
      return;
    }
    for (Node child : frame.children()) {
      findEndEffectors(child, endEffectors);
    }
  }

  public void addConstraint(Node frame, boolean hinge) {
    //If has a child
    hinge = hinge || scene.is2D();
    if (frame == null) return;
    if (frame.children().size() != 1) return;
    if (!hinge) {
      BallAndSocket constraint = new BallAndSocket(minAngle, minAngle, maxAngle, maxAngle);
      Vector twist = frame.children().get(0).translation().get();
      constraint.setRestRotation(frame.rotation().get(), Vector.orthogonalVector(twist), twist);
      frame.setConstraint(constraint);
    } else {
      Vector twist = new Vector(0, 0, 1);
      Vector up = new Vector(0, 1, 0);
      if (frame.children() != null) {
        up = frame.children().get(0).translation().get();
        twist = Vector.orthogonalVector(up);
      }
      Hinge constraint = new Hinge(minAngle, maxAngle, frame.rotation().get(), up, twist);
      frame.setConstraint(constraint);
    }

  }

  List<Solver> solvers = new ArrayList<Solver>();

  public void addTreeSolver() {
    if (scene.node() == null) return;
    Solver solver = null;
    if (debug) {
      solver = new Tree(scene.node(), IKSolver.HeuristicMode.BACK_AND_FORTH_TRIK);
      //solver.setTimesPerFrame(1f);
      solvers.add(solver);
    } else {
      if (scene.node() != null) {
        solver = scene.registerTreeSolver(scene.node());
        solvers.add(solver);
      }
    }

    //add target
    //get leaf nodes
    ArrayList<Node> endEffectors = new ArrayList<Node>();
    findEndEffectors(scene.node(), endEffectors);
    for (Node endEffector : endEffectors) {
      endEffector.tagging = false;
      Target target = new Target(radius * 1.2f, endEffector.position().get(), endEffector.orientation().get());
      target.setReference(scene.node().reference());
      if (solver instanceof Tree) ((Tree) solver).addTarget(endEffector, target);
      targets.add(target);
    }
  }

  public void printTree(Node root, String sep) {
    if (root == null) return;
    System.out.print(sep + "|-> Node ");
    System.out.println("translation: " + root.translation() + "rotation axis: " + root.rotation().axis() + "rotation angle : " + root.rotation().angle());
    for (Node child : root.children()) {
      printTree(child, sep + "\t");
    }
  }
}

