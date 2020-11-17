/*
Benchmark between IK Heuristics
By Sebastian Chaparro Cuevas

This example compares the performance in terms of accuracy between different IK heuristics steps using GHIK-Chain.
You can customize several parameters of this benchmark as:
*) The number of joints of the IK Chain
*) The Type of chain. we distinguish among:
    - NONE : Unconstrained chain
    - HINGE : Each joint has a random Hinge constraint
    - CONE_ELLIPSE : Each joint has a random Ball and Socket constraint
    - MIX_CONSTRAINED : Each joint has a random constraint.
*) The Type of solvers to use: Add to the solversType array the IK Heuristic step we want to consider. Choose among:
    - CCD : Cyclic Coordinate Descent
    - BFIK_CCD: Back and Forth - CCD
    - TIK: Triangulation
    - BFIK_TIK: Back and Forth - Triangulation
    - TRIK: Translate and Reach IK
    - BFIK_TRIK: Back and Forth - Translate and Reach IK 
    - ECTIK: Extended CCD and Triangulation IK
    - TRIK_ECTIK: Translate and Reach IK and Extended CCD Triangulation IK
    - ECTIK_DAMP: Extended CCD and Triangulation IK with dampening
    Preferable solver for unconstrained chain is TRIK, for constrained chains choose either TRIK_ECTIK or BFIK_TRIK.
    
* Press 'p' to follow a continuous path and 's' to stop the interpolators.
* Press 'w' to start/stop the solvers
* Press 'r' to generate a random Target position
* Drag with the right button the red balls to solve IK.
*/

import nub.core.*;
import nub.ik.solver.*;
import nub.primitives.*;
import nub.core.constraint.*;
import nub.processing.*;
import nub.timing.Task;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

//Benchmark Parameters
int numJoints = 10; //Define the number of joints that each chain will contain
Util.ConstraintType constraintType = Util.ConstraintType.NONE; //Choose among Util.ConstraintType.NONE, Util.ConstraintType.HINGE, Util.ConstraintType.CONE_ELLIPSE, Util.ConstraintType.MIX_CONSTRAINED 
Util.SolverType solversType[] = {Util.SolverType.CCD, Util.SolverType.TIK, Util.SolverType.TRIK, Util.SolverType.BFIK_TRIK, Util.SolverType.TRIK_ECTIK};
ArrayList<Solver> solvers; //Will store Solvers
int randRotation = -1; //Set seed to generate initial random rotations, otherwise set to -1
int randLength = 0; //Set seed to generate random segment lengths, otherwise set to -1

//Scene Parameters
Random random = new Random();
Scene scene;
String renderer = P3D; //Define a 2D/3D renderer
float targetRadius = 10; //Define size of target
float boneLength = 50; //Define length of segments (bones)
ArrayList<ArrayList<Node>> structures = new ArrayList<ArrayList<Node>>(); //Keep Structures
ArrayList<Node> idleSkeleton;
ArrayList<Node> targets = new ArrayList<Node>(); //Keep targets
ArrayList<Interpolator> interpolators = new ArrayList<Interpolator>(); //Interpolators
boolean solve = true;
Task task;


void settings() {
  size(800, 600, renderer);
}

void setup() {
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
  //2. Generate IK Chains
  for (int i = 0; i < numSolvers; i++) {
    float offset = numSolvers == 1 ? 0 : i * 2 * alpha * scene.radius() / (numSolvers - 1) - alpha * scene.radius();
    int r = (int) random(255), g = (int) random(255), b = (int) random(255);
    structures.add(Util.generateAttachedChain(scene, numJoints, 0.7f * targetRadius, boneLength, new Vector(offset, 0, 0), color(r, g, b), randRotation, randLength));
  }
  //3. Apply constraints
  for (ArrayList<Node> structure : structures) {
    Util.generateConstraints(structure, constraintType, 0, scene.is3D());
  }
  idleSkeleton = Util.detachedCopy(structures.get(0)); //Dummy chain 
  //4. Set eye scene
  scene.eye().rotate(new Quaternion(new Vector(1, 0, 0), PI / 2.f));
  scene.eye().rotate(new Quaternion(new Vector(0, 1, 0), PI));
  //5. generate solvers
  solvers = new ArrayList<Solver>();
  for (int i = 0; i < numSolvers; i++) {
    final Solver solver = Util.createSolver(solversType[i], structures.get(i));
    solvers.add(solver);
    //6. Define solver parameters
    solvers.get(i).setMaxError(0.0001);
    solvers.get(i).setMinDistance(0);
    solvers.get(i).setTimesPerFrame(15);
    solvers.get(i).setMaxIterations(15);
    //7. Set targets
    solvers.get(i).setTarget(structures.get(i).get(numJoints - 1), targets.get(i));
    targets.get(i).setPosition(structures.get(i).get(numJoints - 1).position());
    //8. Register task
    TimingTask task = new TimingTask() {
      @Override
      public void execute() {
        if (solve) solver.solve();
      }
    };
    task.run(40);
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
  
  //Scene hints
  scene.enableHint(Scene.BACKGROUND, 0);
  scene.enableHint(Scene.AXES);

}

void draw() {
  lights();
  ambientLight(102, 102, 102);
  lightSpecular(204, 204, 204);
  directionalLight(102, 102, 102, 0, 0, -1);
  specular(255, 255, 255);
  shininess(10);

  //Draw Constraints
  scene.render();
  scene.beginHUD();
  for (int i = 0; i < solvers.size(); i++) {
    Util.printInfo(scene, solvers.get(i), structures.get(i).get(0).position());
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


void keyPressed() {
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
      if (s instanceof GHIK){
        ((GHIK) s).context().setSingleStep(!((GHIK) s).context().singleStep());
        if(((GHIK) s).context().singleStep())
          s.setTimesPerFrame(1);
      }
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

void mouseMoved() {
  scene.mouseTag();
}

void mouseDragged() {
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

void mouseWheel(MouseEvent event) {
  scene.scale(event.getCount() * 20);
}

void mouseClicked(MouseEvent event) {
  if (event.getCount() == 2)
    if (event.getButton() == LEFT)
      scene.focus();
    else
      scene.align();
}
