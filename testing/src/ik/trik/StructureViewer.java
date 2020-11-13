package ik.trik;

import ik.basic.Util;
import nub.core.Graph;
import nub.core.Node;
import nub.ik.solver.NodeInformation;
import nub.ik.solver.GHIK;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.processing.TimingTask;
import processing.core.PApplet;
import processing.event.MouseEvent;


import java.util.ArrayList;
import java.util.List;

public class StructureViewer extends PApplet {
  Scene scene;
  String renderer = P3D;
  int numJoints = 8;
  float targetRadius;
  float boneLength = 50;
  boolean solve = false;
  String path ="C:/Users/olgaa/Desktop/Sebas/Thesis/Results/Viz/Structure/";
  String name = "structure";
  int idx = 0;
  boolean axes = true;
  List<Node> idle;
  List<Vector> positions;
  List<List<Node>> structures = new ArrayList<>();
  List<GHIK> solvers = new ArrayList<>();
  boolean continuous = true;
  int n_structures = 3;

  int n = 100;

  public void settings(){
    size(1900,1000, renderer);

    smooth(8);
    randomSeed(0);
  }

  public void setup(){
    NodeInformation.disableCache = true;
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    targetRadius = numJoints * boneLength * 0.8f * 0.05f;
    scene.setBounds(numJoints * boneLength * 0.6f);
    scene.leftHanded = false;
    scene.fit(0);
    scene.context().smooth(8);

    if(axes)scene.enableHint(Graph.AXES);
    scene.enableHint(Graph.BACKGROUND, color(255));

    //Create target
    for(int i = 0; i < n_structures; i++) {
      Node target = Util.createTarget(scene, targetRadius);
      float inc = i == 0 ? 0.01f : 0;
      List<Node> structure = Util.generateAttachedChain(numJoints, (0.7f + inc) * targetRadius, boneLength, !axes ? new Vector(0, -0.25f * numJoints * boneLength, 0) : new Vector(), 0, 100, 150);
      idle = Util.detachedCopy(structure);

      //Util.generateConstraints(structure, Util.ConstraintType.MIX_CONSTRAINED, 13, scene.is3D());

      GHIK solver = new GHIK(structure, GHIK.HeuristicMode.TRIK, false);
      //solver.setHeuristic(new CCDViz(  solver.context(), viz));
      solver.setMaxError(-10f);
      solver.setMinDistance(-10f);
      solver.setTimesPerFrame(5);
      solver.setMaxIterations(5);
      solver.setTarget(structure.get(numJoints - 1), target);
      solver.context().setTopToBottom(false);
      target.setPosition(structure.get(numJoints - 1).position());
      //Set the original color structure
      for (Node node : structure) {
        println(10 + 240.f * i / n_structures);
        if(i == 1) node._boneColor = color(204, 255, 179);
        if(i == 0) node._boneColor = color(0, 0, 255, 255);
        if(i == 2) node._boneColor = color(222, 135, 135);
      }
      TimingTask task = new TimingTask() {
        @Override
        public void execute() {
          if (solve) solver.solve();
        }
      };
      task.run(40);
      solvers.add(solver);
    }

    scene.setShape(pg ->{
      pg.pushStyle();
      int s = 0;
      Vector prev = null;
      for(Vector p : positions){
        pg.pushMatrix();
        pg.strokeWeight(4);
        pg.fill(10 + 235.f * s++ / n,0,0, 255);
        pg.stroke(10 + 235.f * s++ / n,0,0, 255);
        if(prev != null && continuous) pg.line(prev.x(), prev.y(), prev.z(), p.x(), p.y(), p.z());
        else sphere(targetRadius);
        prev = p;
        pg.popMatrix();
      }
      pg.popStyle();
    });
    positions = generatePath(idle, n);
  }

  public void draw() {
    render();
    scene.beginHUD();
    pushStyle();
    text(mouseX + ", " + mouseY, mouseX, mouseY);
    noFill();
    popStyle();
    scene.endHUD();
  }

  public void render(){
    ambientLight(102, 102, 102);
    lightSpecular(204, 204, 204);
    directionalLight(102, 102, 102, 0, -0.1f, 0.4f);
    specular(255, 255, 255);
    shininess(10);
    scene.render();
  }

  //Random discontinuous path



  //Random continuous path
  public List<Vector> generatePath(List<? extends Node> structure, int n) {
    noiseSeed((int) random(1000));
    ArrayList<Vector> targetPositions = new ArrayList<Vector>();
    //Generate a random near pose
    Node node = structure.get(structure.size() - 1);
    float maxDist = 0, minDist = Float.MAX_VALUE, meanDist = 0;
    Vector prev = node.position();
    float step = 0.01f;
    float last = step * n;


    for (float t = 0; t < last; t += step) {
      for (int i = 0; i < structure.size(); i++) {
        float angle = TWO_PI * noise(1000 * i + t) - PI;
        Vector dir = new Vector(noise(10000 * i + t), noise(20000 * i + t), noise(30000 * i + t));
        structure.get(i).setRotation(new Quaternion(dir, angle));
      }
      targetPositions.add(node.position());
      Vector curr = node.position();
      if (t != 0) {
        if (Vector.distance(prev, curr) > maxDist) {
          maxDist = Vector.distance(prev, curr);
        }
        if (Vector.distance(prev, curr) < minDist) {
          minDist = Vector.distance(prev, curr);
        }
        meanDist += Vector.distance(prev, curr);
        n++;
      }
      prev = curr;
    }
    solvers.get(0).target().setPosition(targetPositions.get(targetPositions.size() / 2).get());
    solvers.get(1).target().setPosition(targetPositions.get(0).get());
    solvers.get(2).target().setPosition(targetPositions.get(targetPositions.size() - 1).get());

    return targetPositions;
  }

  public ArrayList<Vector> generateLissajousCurve(int n, float x_speed, float y_speed, float z_speed, float radius) {
    ArrayList<Vector> targetPositions = new ArrayList<Vector>();
    Vector init = new Vector(0, radius,0);
    float step = 360f / n;
    for (float angle = 0; angle < 360 + step; angle += step) {
      float rad = radians(angle);
      float x = radius * cos(x_speed * rad);
      float y = radius * sin(y_speed * rad);
      float z = radius * sin(z_speed * rad);
      targetPositions.add(new Vector(init.x() + x, init.y() - (y + radius * 1.2f), init.z() + z));
    }
    return targetPositions;
  }

  public void keyPressed() {
    if (key == 'w' || key == 'W') {
      solve = !solve;
    }

    if (key == 's' || key == 'S') {
      for(GHIK solver : solvers) solver.solve();
    }

    if(key == 'q' || key == 'Q'){
      positions = generatePath(idle, n);
    }

    if(key == 'p' || key == 'P'){
      scene.context().save( path + "/" + name + idx++ + ".jpg");
    }
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

  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.trik.StructureViewer"});
  }
}
