package ik.trik;

import ik.basic.Util;
import nub.core.Graph;
import nub.core.Interpolator;
import nub.core.Node;
import nub.ik.solver.Solver;
import nub.ik.solver.GHIK;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.timing.Task;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class HeuristicBenchmark extends PApplet {
  //Scene Parameters
  Scene scene;
  String renderer = P3D; //Define a 2D/3D renderer
  int numJoints = 10; //Define the number of joints that each chain will contain
  float targetRadius = 10; //Define size of target
  float boneLength = 50; //Define length of segments (bones)

  //Benchmark Parameters
  Util.ConstraintType constraintType = Util.ConstraintType.NONE; //Choose what kind of constraints apply to chain
  Random random = new Random();
  ArrayList<Solver> solvers; //Will store Solvers
  int randRotation = -1; //Set seed to generate initial random rotations, otherwise set to -1
  int randLength = 0; //Set seed to generate random segment lengths, otherwise set to -1

  Util.SolverType solversType[] = {Util.SolverType.TRIK, Util.SolverType.FABRIK};
  ArrayList<ArrayList<Node>> structures = new ArrayList<>(); //Keep Structures
  ArrayList<Node> idleSkeleton;
  ArrayList<Node> targets = new ArrayList<Node>(); //Keep targets

  ArrayList<Interpolator> interpolators = new ArrayList<Interpolator>();
  Task task;

  float sk_height = 0;
  boolean solve = false;

  public void settings() {
    size(1500, 800, renderer);
  }

  public void setup() {
    scene = new Scene(this);
    if (scene.is3D()) scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.setBounds(numJoints * 1f * boneLength);
    scene.fit(1);
    scene.leftHanded = false;

    int numSolvers = solversType.length;
    //1. Create Targets
    targets = Util.createTargets(numSolvers, scene, targetRadius);

    float alpha = 1.f * width / height > 1.5f ? 0.5f * width / height : 0.5f;
    alpha *= numSolvers / 4f; //avoid undesirable overlapping

    //2. Generate Structures
    for (int i = 0; i < numSolvers; i++) {
      float offset = numSolvers == 1 ? 0 : i * 2 * alpha * scene.radius() / (numSolvers - 1) - alpha * scene.radius();
      int r = (int) random(255), g = (int) random(255), b = (int) random(255);
      structures.add(Util.generateAttachedChain(numJoints, 0.7f * targetRadius, boneLength, new Vector(offset, 0, 0), r, g, b, randRotation, randLength + 10));
    }

    for(int i = 1; i < structures.get(0).size(); i++){
      sk_height += structures.get(0).get(i).translation().magnitude();
    }

    //3. Apply constraints
    for (ArrayList<Node> structure : structures) {
      Util.generateConstraints(structure, constraintType, 0, scene.is3D());
    }

    idleSkeleton = Util.detachedCopy(structures.get(0));

    //4. Set eye scene
    scene.eye().rotate(new Quaternion(new Vector(1, 0, 0), PI / 2.f));
    scene.eye().rotate(new Quaternion(new Vector(0, 1, 0), PI));

    //5. generate solvers
    solvers = new ArrayList<>();
    for (int i = 0; i < numSolvers; i++) {
      Solver solver = Util.createSolver(solversType[i], structures.get(i));
      solvers.add(solver);
      //6. Define solver parameters
      if(solvers.get(i) instanceof GHIK) ((GHIK)solvers.get(i)).enableDeadLockResolution(true);
      solvers.get(i).setMaxError(0.001f * sk_height);
      solvers.get(i).setMinDistance(-10f);
      solvers.get(i).setTimesPerFrame(1);
      solvers.get(i).setMaxIterations(50);
      //7. Set targets
      solvers.get(i).setTarget(structures.get(i).get(numJoints - 1), targets.get(i));
      targets.get(i).setPosition(structures.get(i).get(numJoints - 1).position());
      Interpolator interpolator = new Interpolator(targets.get(i));
      interpolator.configHint(Interpolator.SPLINE);
      interpolators.add(interpolator);
    }

    //define the interpolator task
    task = new Task() { //TODO : Make this task work
      @Override
      public void execute() {
        Vector pos = targets.get(0).position().get();
        //interpolator.execute();
        //update other targets
        for (Node target : targets) {
          if (target == targets.get(0)) continue;
          Vector diff = Vector.subtract(targets.get(0).position(), pos);
          target.translate(diff);
          target.setOrientation(targets.get(0).orientation());
        }
      }
    };


  }

  public void draw() {
    background(0);
    if (scene.is3D()) lights();
    //Draw Constraints
    scene.drawAxes();
    scene.render();
    scene.beginHUD();
    for (int i = 0; i < solvers.size(); i++) {
      if(solve) solvers.get(i).solve();
      Util.printInfo(scene, solvers.get(i), structures.get(i).get(0).position(), sk_height);
    }
    scene.endHUD();
    fill(255);
    stroke(255);
  }

  public Node generateRandomReachablePosition(List<? extends Node> chain, boolean is3D) {
    for (int i = 0; i < chain.size() - 1; i++) {
      if (is3D) {
        if (random.nextFloat() > 0.5f) {
          chain.get(i).rotate(new Quaternion(new Vector(0, 0, 1), random.nextFloat() * 2 * PI - PI));
          chain.get(i).rotate(new Quaternion(new Vector(0, 1, 0), random.nextFloat() * 2 * PI - PI));
          chain.get(i).rotate(new Quaternion(new Vector(1, 0, 0), random.nextFloat() * 2 * PI - PI));
        }
      } else
        chain.get(i).rotate(new Quaternion(new Vector(0, 0, 1), (float) (random.nextFloat() * PI)));
    }
    return chain.get(chain.size() - 1);
  }

  public void generatePath() {
    int idx = 0;
    int seed = (int) (random(0,10000));
    for(Interpolator interpolator : interpolators){
      List<? extends Node> structure = structures.get(idx);
      randomSeed(seed);
      noiseSeed(seed);
      interpolator.clear(); // reset the interpolator
      interpolator.setNode(targets.get(idx++));
      //Generate a random near pose
      Node node = structure.get(structure.size() - 1);
      float maxDist = 0, minDist = Float.MAX_VALUE, meanDist = 0;
      Vector prev = node.position();
      int n = 100;
      float step = 0.1f;
      float last = step * n;


      for (float t = 0; t < last; t += step) {
        for (int i = 0; i < structure.size(); i++) {
          float angle = TWO_PI * noise(1000 * i + t) - PI;
          Vector dir = new Vector(noise(10000 * i + t), noise(20000 * i + t), noise(30000 * i + t));
          structure.get(i).setRotation(new Quaternion(dir, angle));
        }
        Node key = new Node(node.position(), node.orientation(), 1.f);
        interpolator.addKeyFrame(key, 512, 1);
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
    }
  }


  public void keyPressed() {
    if (key == 'w' || key == 'W') {
      solve = !solve;
    }
    if (key == 'R' || key == 'r') {
      Node f = generateRandomReachablePosition(idleSkeleton, scene.is3D());
      Vector delta = Vector.subtract(f.position(), targets.get(0).position());
      for (Node target : targets) {
        target.setPosition(Vector.add(target.position(), delta));
        target.setOrientation(f.orientation());
      }
    }
    if (key == 'i' || key == 'I') {
      for (List<Node> structure : structures) {
        for (Node f : structure) {
          f.setRotation(new Quaternion());
        }
      }
    }

    if (key == 's' || key == 'S') {
      for (Solver s : solvers) s.solve();
    }

    if (key == 'm' || key == 'M') {
      for (Solver s : solvers) {
        if (s instanceof GHIK)
          ((GHIK) s).context().setSingleStep(!((GHIK) s).context().singleStep());
        if (s instanceof GHIK)
          ((GHIK) s).context().setSingleStep(!((GHIK) s).context().singleStep());
      }
    }

    if (key == '0') {
      for (Solver s : solvers) {
        if (s instanceof GHIK)
          ((GHIK) s).context().setDirection(!((GHIK) s).context().direction());
      }
    }

    if (key == 'p' || key == 'P') {
      for (List<Node> structure : structures) {
        for (Node f : structure) {
          f.setRotation(new Quaternion());
        }
      }
      generatePath();
      for(Interpolator interpolator : interpolators){
        interpolator.enableRecurrence();
        interpolator.run();
      }
    }

    if(key == 's' || key == 'S'){
      for(Interpolator interpolator : interpolators){
        interpolator.task().stop();
      }
    }


    if (key == '1') {
      for(Interpolator interpolator : interpolators){
        interpolator.toggleHint(Interpolator.SPLINE);
      }
    }
    if (key == '2') {
      for(Interpolator interpolator : interpolators){
        interpolator.setSpeed(interpolator.speed() * 1.2f);
      }
    }
    if (key == '3') {
      for(Interpolator interpolator : interpolators){
        interpolator.setSpeed(interpolator.speed() * 0.8f);
      }
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
      if (targets.contains(scene.node())) {
        for (Node target : targets) scene.translateNode(target, scene.mouseDX(), scene.mouseDY(), 0, 0);
      } else {
        scene.mouseTranslate();
      }
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
    PApplet.main(new String[]{"ik.trik.HeuristicBenchmark"});
  }

}
