package ik.trik;

import ik.basic.Util;
import nub.core.Graph;
import nub.core.Interpolator;
import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.solver.GHIK;
import nub.ik.solver.Solver;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.timing.Task;
import processing.core.PApplet;
import processing.core.PFont;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class FABRIKVsTRIK extends PApplet {
  int numJoints = 8; //Define the number of joints that each chain will contain
  Util.ConstraintType constraintType = Util.ConstraintType.NONE; //Choose among Util.ConstraintType.NONE, Util.ConstraintType.HINGE, Util.ConstraintType.CONE_ELLIPSE, Util.ConstraintType.MIX_CONSTRAINED
  //Util.SolverType solversType[] = {Util.SolverType.CCD, Util.SolverType.TIK, Util.SolverType.TRIK, Util.SolverType.BFIK_TRIK, Util.SolverType.TRIK_ECTIK}; //If you wish you could add other Solvers, as the ones listed above
  Util.SolverType solversType[] = {Util.SolverType.FABRIK, Util.SolverType.TRIK}; //If you wish you could add other Solvers, as the ones listed above
  //-------------------------------------------------------------------
  //Scene Parameters
  int randRotation = -1; //Set seed to generate initial random rotations, otherwise set to -1
  int randLength = 0; //Set seed to generate random segment lengths, otherwise set to -1
  Random random = new Random();
  Scene scene;
  String renderer = P3D; //Define a 2D/3D renderer
  float targetRadius = 10; //Define size of target
  float boneLength = 50; //Define length of segments (bones)
  ArrayList<ArrayList<ArrayList<Node>>> listStructures = new ArrayList<ArrayList<ArrayList<Node>>>(); //Keep Structures
  ArrayList<ArrayList<Node>> listIdleSkeleton = new ArrayList<ArrayList<Node>>();
  ArrayList<ArrayList<Node>> listTargets = new ArrayList<ArrayList<Node>>(); //Keep targets
  ArrayList<ArrayList<Interpolator>> listInterpolators = new ArrayList<ArrayList<Interpolator>>(); //Interpolators
  ArrayList<ArrayList<Solver>> listSolvers = new ArrayList<ArrayList<Solver>>(); //Will store Solvers

  boolean solve = true;
  Task task;
  int[] cols;

  public void settings() {
    fullScreen(renderer);
  }

  public void setup() {
    randomSeed(0);
    PFont myFont = createFont("Times New Roman Bold", 50, true);
    textFont(myFont);
    cols = new int[]{color(0,255,0),color(0,0,255)};
    scene = new Scene(this);
    if (scene.is3D()) scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.setBounds(numJoints * 1f * boneLength);
    scene.fit(1);
    scene.leftHanded = false;

    numJoints = 5;
    generateChains(0, 1, 0.5f * boneLength * 3f);
    numJoints = 10;
    generateChains(0, 1, 0.5f * boneLength * 1.5f);
    numJoints = 15;
    generateChains(0, 1, 0.5f * boneLength * 1.25f);
    numJoints = 20;
    generateChains(0, 1, 0.5f * boneLength);

    changeVisibleChain();


    scene.eye().setConstraint(new Constraint() {
      @Override
      public Quaternion constrainRotation(Quaternion rotation, Node node) {
        return new Quaternion(0,rotation.eulerAngles().y(),0);
      }
    });

    //Scene hints
    scene.enableHint(Scene.BACKGROUND, 0);
    scene.enableHint(Scene.AXES, 0.5f * boneLength * numJoints);
    scene.fit(0);
  }

  public void generateChains(int k, int nc, float boneLength){
    int numSolvers = solversType.length;
    //1. Create Targets
    ArrayList<Node> targets = Util.createTargets(numSolvers, scene, targetRadius);
    ArrayList<ArrayList<Node>> structures = new ArrayList<ArrayList<Node>>();
    listTargets.add(targets);
    float alpha = 1.f * width / height > 1.5f ? 0.5f * width / height : 0.5f;
    alpha *= numSolvers / 4f; //avoid undesirable overlapping
    //2. Generate IK Chains
    for (int i = 0; i < numSolvers; i++) {
      float offset = numSolvers == 1 ? 0 : i * 2 * alpha * scene.radius() / (numSolvers - 1) - alpha * scene.radius();
      int r = (int) red(cols[i]), g = (int) green(cols[i]), b = (int) blue(cols[i]);
      structures.add(Util.generateAttachedChain(numJoints, 0.7f * targetRadius + 0.0000001f * i, boneLength, new Vector(0, 0, 0), r, g, b, randRotation, randLength));
    }
    listStructures.add(structures);
    //3. Apply constraints
    for (ArrayList<Node> structure : structures) {
      Util.generateConstraints(structure, constraintType, 29, scene.is3D());
    }
    ArrayList<Node> idleSkeleton = Util.detachedCopy(structures.get(0)); //Dummy chain
    listIdleSkeleton.add(idleSkeleton);
    //4. Set eye scene
    scene.eye().rotate(new Quaternion(new Vector(1, 0, 0), PI / 2.f));
    scene.eye().rotate(new Quaternion(new Vector(0, 1, 0), PI));
    //5. generate solvers
    ArrayList<Solver> solvers = new ArrayList<Solver>();
    ArrayList<Interpolator> interpolators = new ArrayList<Interpolator>();
    for (int i = 0; i < numSolvers; i++) {
      final Solver solver = Util.createSolver(solversType[i], structures.get(i));
      solvers.add(solver);
      //6. Define solver parameters
      solver.setMaxError(0.01f * boneLength * numJoints); //Set error threshold
      solver.setMinDistance(0); //Set minimum distance
      solver.setTimesPerFrame(50); //Set number of times per frame the solver will be executed
      solver.setMaxIterations(50); //Ste the maximum iterations the solver will be executed
      //7. Set targets
      solvers.get(i).setTarget(structures.get(i).get(numJoints - 1), targets.get(i));
      targets.get(i).setPosition(structures.get(i).get(numJoints - 1).position());
      //8. Register task
      Interpolator interpolator = new Interpolator(targets.get(i));
      interpolator.configHint(Interpolator.SPLINE);
      interpolators.add(interpolator);
    }
    listSolvers.add(solvers);
    listInterpolators.add(interpolators);
  }


  public void draw() {
    lights();
    ambientLight(102, 102, 102);
    lightSpecular(204, 204, 204);
    directionalLight(102, 102, 102, 0, 5, 5);
    specular(255, 255, 255);
    shininess(10);
    for(ArrayList<Solver> solvers : listSolvers){
      for (int i = 0; i < solvers.size(); i++) {
        if (solve) solvers.get(i).solve();
      }
    }
    scene.render();
    scene.beginHUD();
    pushStyle();
    textSize(50);
    fill(255);
    stroke(255);
    textAlign(CENTER, CENTER);
    text("FABRIK and TRIK - Visual smooth comparison - # Joints: " + 5 * (visChain + 1) , width * 0.5f, 100);

    fill(cols[0]);
    stroke(cols[0]);
    textAlign(LEFT, TOP);
    ellipse(width * 0.3f - 50, 175, 45, 45);
    text("FABRIK", width * 0.3f, 150);
    fill(cols[1]);
    stroke(cols[1]);
    textAlign(LEFT, TOP);
    ellipse(width * 0.6f - 50, 175, 45, 45);
    text("TRIK", width * 0.6f, 150);
    popStyle();

    /*for (int i = 0; i < solvers.size(); i++) {
      Util.printInfo(scene, solvers.get(i), structures.get(i).get(0).position());
    }*/
    scene.endHUD();

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
    int k = 0;
    for(List<Interpolator> interpolators : listInterpolators){
      int idx = 0;
      int seed = (int) (random(0,10000));
      ArrayList<ArrayList<Node>> structures = listStructures.get(k);
      ArrayList<Node> targets = listTargets.get(k);
      for(Interpolator interpolator : interpolators){
        List<? extends Node> structure = structures.get(idx);
        randomSeed(seed);
        noiseSeed(seed);
        interpolator.clear(); // reset the interpolator
        interpolator.setNode(targets.get(idx++));
        //Generate a random near pose
        Node node = structure.get(structure.size() - 1);
        float maxDist = 0, minDist = Float.MAX_VALUE;
        Vector prev = node.position();
        int n = 100;
        float step = 0.8f;
        float last = step * n;


        for (float t = 0; t < last; t += step) {
          for (int i = 0; i < structure.size(); i++) {
            float angle = TWO_PI * noise(1000 * i + t) - PI;
            Vector dir = new Vector(noise(10000 * i + t), noise(20000 * i + t), noise(30000 * i + t));
            structure.get(i).setRotation(new Quaternion(dir, angle));
          }
          Node key = new Node(node.position(), node.orientation(), 1.f);
          interpolator.addKeyFrame(key, 512, 0.5f);
          Vector curr = node.position();
          if (t != 0) {
            if (Vector.distance(prev, curr) > maxDist) {
              maxDist = Vector.distance(prev, curr);
            }
            if (Vector.distance(prev, curr) < minDist) {
              minDist = Vector.distance(prev, curr);
            }
            n++;
          }
          prev = curr;
        }
      }
      k++;
    }
  }

  int visChain = 0;
  int chains = 4;
  public void changeVisibleChain(){
    visChain = (visChain + 1) % chains;
    for(int k = 0; k < listStructures.size(); k++){
      int i = 0;
      for(ArrayList<Node> structure : listStructures.get(k)){
        structure.get(0).cull = k != visChain;
        listTargets.get(k).get(i).cull = k != visChain;
        i++;
      }
    }

  }


  public void keyPressed() {
    if (key == 'u' || key == 'U') {
      changeVisibleChain();
    }
    if (key == 'w' || key == 'W') {
      solve = !solve;
    }
    if (key == 'R' || key == 'r') {
      int k = 0;
      for(ArrayList<Node> idleSkeleton : listIdleSkeleton){
        ArrayList<Node> targets = listTargets.get(k);
        Node f = generateRandomReachablePosition(idleSkeleton, scene.is3D());
        Vector delta = Vector.subtract(f.position(), targets.get(0).position());
        for (Node target : targets) {
          target.setPosition(Vector.add(target.position(), delta));
          target.setOrientation(f.orientation());
        }
        k++;
      }
    }
    if (key == 'i' || key == 'I') {
      int k = 0;
      for(ArrayList<ArrayList<Node>> structures : listStructures){
        List<Node> targets = listTargets.get(k);
        int i = 0;
        for (List<Node> structure : structures) {
          for (Node f : structure) {
            f.setRotation(new Quaternion());
          }
          targets.get(i++).setPosition(structure.get(structure.size()-1).position().get());
        }
        k++;
      }
    }

    if (key == 's' || key == 'S') {
      for(ArrayList<Solver> solvers : listSolvers) {
        for (Solver s : solvers) s.solve();
      }
    }

    if (key == 'm' || key == 'M') {
      for(ArrayList<Solver> solvers : listSolvers) {
        for (Solver s : solvers) {
          if (s instanceof GHIK) {
            ((GHIK) s).context().setSingleStep(!((GHIK) s).context().singleStep());
            if (((GHIK) s).context().singleStep())
              s.setTimesPerFrame(1);
          }
        }
      }
    }

    if (key == '0') {
      for(ArrayList<Solver> solvers : listSolvers) {
        for (Solver s : solvers) {
          if (s instanceof GHIK) {
            ((GHIK) s).context().setOrientationWeight(1);
            ((GHIK) s).context().setDirection(!((GHIK) s).context().direction());
          }
        }
      }
    }

    if (key == 'p' || key == 'P') {
      for(ArrayList<ArrayList<Node>> structures : listStructures) {
        for (List<Node> structure : structures) {
          for (Node f : structure) {
            f.setRotation(new Quaternion());
          }
        }
      }
      generatePath();
      for(List<Interpolator> interpolators : listInterpolators) {
        for (Interpolator interpolator : interpolators) {
          interpolator.enableRecurrence();
          interpolator.run(1);
        }
      }
    }

    if(key == 's' || key == 'S'){
      for(List<Interpolator> interpolators : listInterpolators) {
        for (Interpolator interpolator : interpolators) {
          interpolator.task().stop();
        }
      }
    }

    if (key == '1') {
      for(List<Interpolator> interpolators : listInterpolators) {
        for (Interpolator interpolator : interpolators) {
          interpolator.toggleHint(Interpolator.SPLINE);
        }
      }
    }
    if (key == '2') {
      for(List<Interpolator> interpolators : listInterpolators) {
        for (Interpolator interpolator : interpolators) {
          interpolator.setSpeed(interpolator.speed() * 1.2f);
        }
      }
    }
    if (key == '3') {
      for(List<Interpolator> interpolators : listInterpolators) {
        for (Interpolator interpolator : interpolators) {
          interpolator.setSpeed(interpolator.speed() * 0.8f);
        }
      }
    }
  }

  public void mouseMoved() {
    scene.mouseTag();
  }

  public void mouseDragged() {
    if (mouseButton == LEFT) {
      scene.mouseSpin();
    } else if (mouseButton == RIGHT) {
      boolean isTarget = false;
      for(ArrayList<Node> targets : listTargets){
        if (targets.contains(scene.node())) {
          for (Node target : targets) scene.translateNode(target, scene.mouseDX(), scene.mouseDY(), 0, 0);
          isTarget = true;
          break;
        }
      }
      if(isTarget == false){
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
    PApplet.main(new String[]{"ik.trik.FABRIKVsTRIK"});
  }

}
