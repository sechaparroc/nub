package ik.paper;

import ik.basic.Util;
import nub.core.Node;
import nub.ik.solver.GHIK;
import nub.ik.solver.NodeInformation;
import nub.ik.solver.Solver;
import nub.ik.solver.fabrik.FABRIKChain;
import nub.ik.solver.heuristic.BFIK;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import processing.core.PApplet;
import processing.data.JSONArray;
import processing.data.JSONObject;

import java.io.File;
import java.util.*;

public class AccuracyVSTimeRefactored {
  static float PI = (float) Math.PI;
  static Random random = new Random();
  static int seed = 29;

  static String dir = "C:/Users/olgaa/Desktop/Sebas/Thesis/Results/Clean/Single/data/";

  static int numStructures = 500;
  static int numPostures = 100;
  static int randRotation = -1;
  static int randLength = 0;
  static int boneLength = 50;
  static int numJoints;
  static float accuracyThreshold = 0.001f;

  static Util.ConstraintType constraintTypes[] = {
      Util.ConstraintType.NONE,
      Util.ConstraintType.HINGE,
      Util.ConstraintType.CONE_ELLIPSE,
      Util.ConstraintType.MIX_CONSTRAINED
  }; //Choose what kind of constraints apply to chain

  static Util.SolverType solversType[] = {
      Util.SolverType.CCD,
      Util.SolverType.TIK,
      Util.SolverType.TRIK,
      Util.SolverType.BFCCD,
      Util.SolverType.BFTIK,
      Util.SolverType.BFTRIK,
      Util.SolverType.BFIK,
      Util.SolverType.FABRIK_O,
      Util.SolverType.FABRIK_P,
  }; //Place Here Solvers that you want to compare



  static List<Vector> targetPositions;
  static List<Quaternion> initialConfig;


  public static void generateExperiment(int num_structure, Util.SolverType type, SolverData solverData, Util.ConstraintType constraintType, int iterations, int seed, boolean continuous){
    //Generate structure
    List<Node> structure = Util.generateDetachedChain(numJoints, boneLength, randRotation, seed);
    //Apply constraints
    Util.generateConstraints(structure, constraintType, seed, true);
    //For continuous trajectory set the starting position
    if(continuous){
      for(int i = 0; i < initialConfig.size(); i++){
        structure.get(i).setRotation(initialConfig.get(i).get());
      }
      if(type == Util.SolverType.FABRIK_P){
        //just keep positions
        ArrayList<Vector> pos = new ArrayList<>();
        for(int i = 0; i < initialConfig.size(); i++){
          pos.add(structure.get(i).position().get());
        }
        //reset rotations and set positions
        for(int i = 0; i < initialConfig.size(); i++){
          structure.get(i).setRotation(new Quaternion());
          structure.get(i).setPosition(pos.get(i).get());
        }
      }
    }
    //Find initial configuration and skeleton height
    float sk_height = 0;
    int k = 0;
    //Save the current values
    List<Quaternion> rotations = new ArrayList<Quaternion>();
    for(Node n : structure){
      rotations.add(n.rotation().get());
      if(k > 0) sk_height += n.translation().magnitude();
      k++;
    }
    Node endEffector = structure.get(structure.size() - 1);
    //Generate solver
    Solver solver = Util.createSolver(type, structure);
    //Define solver parameters
    solver.setMaxError(sk_height * accuracyThreshold);
    solver.setMaxIterations(iterations);
    solver.setTimesPerFrame(1);
    solver.setMinDistance(-1);
    if(solver instanceof GHIK){
      GHIK GHIK = (GHIK) solver;
      GHIK.enableDeadLockResolution(true);
      if(GHIK.heuristic() instanceof BFIK){
        BFIK heuristic = (BFIK) GHIK.heuristic();
        heuristic.setTRIKFraction(continuous ? 0.1f : 0.1f); //First 5 iterations will use TRIK the others use combined heuristic
      }
    } else if(solver instanceof FABRIKChain){
      FABRIKChain fabrik = (FABRIKChain) solver;
      if(constraintType != Util.ConstraintType.NONE){
        fabrik.enableDeadLockResolution(true);
      }
    }

    //Set targets
    Node target = new Node();
    target.setPosition(endEffector.position().get());
    target.setOrientation(endEffector.orientation().get());
    solver.setTarget(endEffector, target);
    int sample = 0;

    List<Vector> prevPos = obtainPositions(structure), nextPos = null;
    List<Quaternion> prevOrs = obtainOrientations(structure), nextOrs = null;
    List<Quaternion> prevRots = obtainRotations(structure), nextRots = null;

    //Solve IK
    int idx = 0;
    for (Vector t : targetPositions) {
      if (sample % 100 == 0) System.out.println(type.name() + "On sample : " + sample);
      target.setPosition(t.get());
      if(solver instanceof GHIK)((GHIK) solver).reset();

      solver.change(true);
      float minError = Float.MAX_VALUE;
      int lastIteration = -1;
      long elapsedTime = 0;

      for (int i = 0; i < iterations; i++) {
        long start = System.nanoTime();
        solver.solve();
        elapsedTime += System.nanoTime() - start;
        minError = solver instanceof GHIK ? ((GHIK)solver).positionError() : solver.error();
        lastIteration = i + 1;
        if(minError <= accuracyThreshold * sk_height){
          break;
        }
      }

      //Do not consider an error below threshold
      minError = Math.max(0, minError - accuracyThreshold * sk_height);
      //Save values in Tidy format
      solverData.solverNames.add(type.name());
      solverData.constraintTypes.add(constraintType.name());
      solverData.continuous.add(continuous);
      solverData.metrics.get("structure").add(num_structure * 1.0);
      solverData.metrics.get("num").add(idx * 1.0);
      solverData.metrics.get("joints").add(numJoints * 1.0);
      solverData.metrics.get("distanceError").add(minError / sk_height * 100.0);
      solverData.metrics.get("iterations").add(1.0 * lastIteration);
      solverData.metrics.get("time").add(elapsedTime / 1000000.0);

      nextPos = obtainPositions(structure);
      solverData.metrics.get("meanJointPositionDistance").add(distanceBetweenPostures(prevPos, nextPos) / sk_height * 100f);

      if(type != Util.SolverType.FABRIK_P) {
        nextOrs = obtainOrientations(structure);
        solverData.metrics.get("meanJointOrientationDistance").add(distanceBetweenRotations(prevOrs, nextOrs));

        nextRots = obtainRotations(structure);
        double distRot = distanceBetweenRotations(prevRots, nextRots);
        solverData.metrics.get("meanJointRotationDistance").add(distRot);
        if (distRot > 0.01f) {
          solverData.metrics.get("gini").add(motionDistribution(prevRots, nextRots, distRot));
        } else{
          solverData.metrics.get("gini").add(Double.NaN);
        }
      } else{
        solverData.metrics.get("meanJointOrientationDistance").add(Double.NaN);
        solverData.metrics.get("meanJointRotationDistance").add(Double.NaN);
        solverData.metrics.get("gini").add(Double.NaN);
      }

      if(solver instanceof GHIK){
        solverData.metrics.get("deadlocks").add(((GHIK) solver).totalDeadlock() * 1.0);
      } else if(solver instanceof FABRIKChain){
        solverData.metrics.get("deadlocks").add(((FABRIKChain) solver).totalDeadlock() * 1.0);
      }
      prevPos = nextPos;
      prevOrs = nextOrs;
      prevRots = nextRots;
      sample++;
      idx++;
    }
  }




  public static void generateRandomReachablePositions(int n, int seed, Util.ConstraintType constraintType) {
    List<Node> chain = Util.generateDetachedChain(numJoints, boneLength, randRotation, seed);
    Util.generateConstraints(chain, constraintType, seed, true);

    targetPositions = new ArrayList<Vector>();
    for (int t = 0; t < n; t++) {
      Vector prev = chain.get(chain.size() - 1).position().get();
      for (int i = 0; i < chain.size() - 1; i++) {
        if (random.nextFloat() > 0.4f) {
          chain.get(i).rotate(new Quaternion(new Vector(0, 0, 1), random.nextFloat() * 2 * PI - PI));
          chain.get(i).rotate(new Quaternion(new Vector(0, 1, 0), random.nextFloat() * 2 * PI - PI));
          chain.get(i).rotate(new Quaternion(new Vector(1, 0, 0), random.nextFloat() * 2 * PI - PI));
        }
      }
      //save the position of the target
      Vector des = chain.get(chain.size() - 1).position().get();
      if(Vector.distance(des, prev) < 5) t--;
      else{
        targetPositions.add(des);
      }

    }
  }

  public static void main(String args[]) {
    NodeInformation.disableCache = false; //disable cache for "highly" precision benchmarking
    int numSolvers = solversType.length;

    for(int k = 0; k < constraintTypes.length; k++) {
      SolverData solverData = new SolverData();
      random.setSeed(seed);
      int c_seed = 13;
      Util.ConstraintType constraintType = constraintTypes[k];
      for (int s = 0; s < numStructures; s++) {
        numJoints = random.nextInt(16) + 4;
        System.out.println("On structure " + s + " joints : " + numJoints + " constraint : " + constraintType);
        generateRandomPath(numPostures, c_seed, constraintType);
        for (int i = 0; i < numSolvers; i++) {
          if(constraintType != Util.ConstraintType.NONE && solversType[i] == Util.SolverType.FABRIK_P) continue;
          generateExperiment(s, solversType[i], solverData, constraintType, 50, c_seed, true);
        }
        generateRandomReachablePositions(numPostures, c_seed, constraintType);

        for (int i = 0; i < numSolvers; i++) {
          if(constraintType != Util.ConstraintType.NONE && solversType[i] == Util.SolverType.FABRIK_P) continue;
          generateExperiment(s, solversType[i], solverData, constraintType, 50, c_seed, false);
        }
        c_seed++;
      }
      //save json file
      String name = constraintType.name();
      //add the parameters to the files name
      name += "_postures_" + numPostures + "_structures_" + numStructures + "_seed_" + seed;
      solverData.save().save(new File(dir + "metrics_" + name + ".json"), "compact");
    }
  }

  public static void generateRandomPath(int n, int seed, Util.ConstraintType constraintType) {
    PApplet pa = new PApplet();
    pa.randomSeed(seed);
    pa.noiseSeed(seed);
    List<Node> chain = Util.generateDetachedChain(numJoints, boneLength, randRotation, seed);
    Util.generateConstraints(chain, constraintType, seed, true);
    targetPositions = new ArrayList<Vector>();
    initialConfig = new ArrayList<Quaternion>();
    float step = 0.1f;
    float last = step * n;
    double mean_dist = 0;

    for (float t = 0; t < last; t += step) {
      //Generate a random near pose
      for (int i = 0; i < chain.size(); i++) {
        float angle = 2 * PI * pa.noise(1000 * i + t) - PI;
        Vector dir = new Vector(pa.noise(10000 * i + t), pa.noise(20000 * i + t), pa.noise(30000 * i + t));
        chain.get(i).setRotation(new Quaternion(dir, angle));
        if(t == 0) initialConfig.add(chain.get(i).rotation().get());
      }
      targetPositions.add(chain.get(chain.size() - 1).position().get());
      if (t > 0) {
        mean_dist += Vector.distance(targetPositions.get(targetPositions.size() - 1), targetPositions.get(targetPositions.size() - 2));
      }
    }
    mean_dist /= targetPositions.size() - 1;
    System.out.println("mean dist : " + mean_dist);
  }


  public static List<Vector> obtainPositions(List<? extends Node> structure) {
    List<Vector> positions = new ArrayList<Vector>();
    for (Node node : structure) {
      positions.add(node.position().get());
    }
    return positions;
  }

  public static List<Quaternion> obtainRotations(List<? extends Node> structure) {
    List<Quaternion> rotations = new ArrayList<Quaternion>();
    for (Node node : structure) {
      rotations.add(node.rotation().get());
    }
    return rotations;
  }

  public static List<Quaternion> obtainOrientations(List<? extends Node> structure) {
    List<Quaternion> orientations = new ArrayList<Quaternion>();
    for (Node node : structure) {
      orientations.add(node.orientation().get());
    }
    return orientations;
  }

  public static double distanceBetweenPostures(List<Vector> prev, List<Vector> cur) {
    double dist = 0;
    for (int i = 0; i < prev.size(); i++) {
      dist += Vector.distance(prev.get(i), cur.get(i));
    }
    return dist / prev.size();
  }

  public static double    distanceBetweenRotations(List<Quaternion> prev, List<Quaternion> cur) {
    double dist = 0;
    for (int i = 0; i < prev.size(); i++) {
      dist += quaternionDistance(prev.get(i), cur.get(i));
    }
    return dist / prev.size();
  }

  public static double motionDistribution(List<Quaternion> prev, List<Quaternion> cur, double dist) {
    List<Double> vals = new ArrayList<Double>();
    for (int i = 0; i < prev.size(); i++) {
      vals.add(quaternionDistance(prev.get(i), cur.get(i)));
    }
    return gini(vals);
  }


  public static double gini(List<Double> values) {
    double sumOfDifference = values.stream()
        .flatMapToDouble(v1 -> values.stream().mapToDouble(v2 -> Math.abs(v1 - v2))).sum();
    double mean = values.stream().mapToDouble(v -> v).average().getAsDouble();
    return sumOfDifference / (2 * values.size() * values.size() * mean);
  }


  public static double quaternionDistance(Quaternion a, Quaternion b) {
    double s1 = 1, s2 = 1;
    if (a.w() < 0) s1 = -1;
    if (b.w() < 0) s2 = -1;
    double dot = s1 * a._quaternion[0] * s2 * b._quaternion[0] + s1 * a._quaternion[1] * s2 * b._quaternion[1] + s1 * a._quaternion[2] * s2 * b._quaternion[2] + s1 * a._quaternion[3] * s2 * b._quaternion[3];
    dot = Math.max(Math.min(dot, 1), -1);
    return Math.acos(2 * Math.pow(dot, 2) - 1);
  }


  //Auxiliar class to keep stats info
  public static class SolverData{
    List<String> solverNames = new ArrayList<>();
    List<String> constraintTypes = new ArrayList<>();
    List<Boolean> continuous = new ArrayList<>();
    HashMap<String, List<Double>> metrics = new HashMap<>();


    public SolverData(){
      initMetrics();
    }

    void initMetrics(){
      metrics.put("structure", new ArrayList<>());
      metrics.put("num", new ArrayList<>());
      metrics.put("joints", new ArrayList<>());
      metrics.put("distanceError", new ArrayList<>());
      metrics.put("iterations", new ArrayList<>());
      metrics.put("deadlocks", new ArrayList<>());
      metrics.put("time", new ArrayList<>());
      metrics.put("meanJointPositionDistance", new ArrayList<>());
      metrics.put("meanJointOrientationDistance", new ArrayList<>());
      metrics.put("meanJointRotationDistance", new ArrayList<>());
      metrics.put("gini", new ArrayList<>());
    }

    JSONObject save(){
      //check that info is complete
      JSONObject json = new JSONObject();
      JSONArray jsonNames = new JSONArray();
      for(int i = 0; i < solverNames.size(); i++){
        jsonNames.append(solverNames.get(i));
      }
      json.setJSONArray("solver", jsonNames);
      if(solverNames.size() != constraintTypes.size()) throw new RuntimeException("Data incomplete: names " + solverNames.size() + " , constraints " + constraintTypes.size());
      JSONArray jsonConstraintTypes = new JSONArray();
      for(int i = 0; i < constraintTypes.size(); i++){
        jsonConstraintTypes.append(constraintTypes.get(i));
      }
      json.setJSONArray("constraint", jsonConstraintTypes);
      if(solverNames.size() != continuous.size()) throw new RuntimeException("Data incomplete: names " + solverNames.size() + " , continuous " + continuous.size());
      JSONArray jsonContinuous = new JSONArray();
      for(int i = 0; i < continuous.size(); i++){
        jsonContinuous.append(continuous.get(i));
      }
      json.setJSONArray("continuous", jsonContinuous);

      for(String key : metrics.keySet()){
        if(solverNames.size() != metrics.get(key).size()) throw new RuntimeException("Data incomplete: names " + solverNames.size() + " , " + key + metrics.get(key).size());
        JSONArray jsonArray = new JSONArray();
        List<Double> vals = metrics.get(key);
        for(Double v : vals){
          if(v.equals(Double.NaN)) jsonArray.append("NaN");
          else jsonArray.append(v);
        }
        json.setJSONArray(key, jsonArray);
      }
      return json;
    }
  }




}
