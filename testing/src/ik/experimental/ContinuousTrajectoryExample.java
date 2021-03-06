package ik.experimental;

import ik.basic.Util;
import nub.core.Node;
import nub.ik.solver.Solver;
import nub.ik.solver.NodeInformation;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import processing.core.PApplet;
import processing.data.JSONArray;
import processing.data.JSONObject;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class ContinuousTrajectoryExample {
  static float PI = (float) Math.PI;
  static Random random = new Random();
  static String dir = "C:/Users/olgaa/Desktop/Sebas/Thesis/Results/ContinuousTrajectoryExample/";
  //Benchmark Parameters
  static int numPostures = 1000; //Set the number of different postures to solve
  static int randRotation = -1; //Set seed to generate initial random rotations, otherwise set to -1
  static int randLength = 0; //Set seed to generate random segment lengths, otherwise set to -1
  static int numJoints = 10; //Define the number of joints that each chain will contain
  static float boneLength = 50; //Define length of segments (bones)
  static int seed = 0; //Seed to use
  static float maxError = 0.1f;

  static Util.ConstraintType constraintTypes[] = {Util.ConstraintType.NONE}; //Choose what kind of constraints apply to chain
  static Util.SolverType solversType[] = {
      Util.SolverType.CCD,
      //Util.SolverType.TRIANGULATION_HEURISTIC,
      //Util.SolverType.COMBINED_HEURISTIC,
      //Util.SolverType.COMBINED_EXPRESSIVE,
      //Util.SolverType.FABRIK
      };
  static List<Vector> targetPositions;
  static float[] lissajousPath = {1, 3, 3, boneLength * numJoints * 0.2f};
  static boolean enableLissajous = false;

  public static void generateExperiment(Util.SolverType type, Util.ConstraintType constraintType, int iterations, JSONObject jsonSolver) {
    //1. Generate structure
    List<Node> structure = Util.generateDetachedChain(numJoints, boneLength, randRotation, randLength);
    Node endEffector = structure.get(structure.size() - 1);
    //2. Apply constraints
    Util.generateConstraints(structure, constraintType, 0, true);
    //3. generate solver
    Solver solver = Util.createSolver(type, structure);
    //4. Define solver parameters
    solver.setMaxError(maxError);
    solver.setMaxIterations(iterations);
    solver.setMinDistance(-1);
    //5. Set target
    Node target = Node.detach(new Vector(), new Quaternion(), 1f);
    target.setPosition(endEffector.position());
    target.setOrientation(endEffector.orientation());
    solver.setTarget(endEffector, target);
    //6. Solve and keep statistics per iteration
    StatisticsPerExperiment stats = new StatisticsPerExperiment();
    int sample = 0;

    List<Vector> prevPos = obtainPositions(structure), nextPos;
    List<Quaternion> prevOrs = obtainOrientations(structure), nextOrs;

    for (Vector t : targetPositions) {
      if (sample % 100 == 0) System.out.println(type.name() + "On sample : " + sample);
      target.setPosition(t.get());

      for (int i = 0; i < iterations; i++) {
        solver.solve();
      }

      nextPos = obtainPositions(structure);
      stats.addDistance(distanceBetweenPostures(prevPos, nextPos));
      prevPos = nextPos;

      nextOrs = obtainOrientations(structure);
      stats.addOrientationDistance(orientationDistanceBetweenPostures(prevOrs, nextOrs));
      prevOrs = nextOrs;

      stats.addValue(solver.error());
      stats.addPoint(structure.get(structure.size() - 1).position());
      sample++;
    }

    stats.updateStatistics();
    //Save statistics
    JSONArray jsonErrorValues = new JSONArray();
    for (Double value : stats.values()) {
      jsonErrorValues.append(value);
    }

    JSONArray jsonDistanceValues = new JSONArray();
    for (Double value : stats.distances()) {
      jsonDistanceValues.append(value);
    }

    JSONArray jsonOrientationDistanceValues = new JSONArray();
    for (Double value : stats.orientationDistances()) {
      jsonOrientationDistanceValues.append(value);
    }

    jsonSolver.setJSONArray("errorValues", jsonErrorValues);
    jsonSolver.setJSONArray("distanceValues", jsonDistanceValues);
    jsonSolver.setJSONArray("orientationDistanceValues", jsonOrientationDistanceValues);

    JSONArray jsonPoints = new JSONArray();
    for (Vector point : stats.points()) {
      jsonPoints.append(point.x());
      jsonPoints.append(point.y());
      jsonPoints.append(point.z());
    }
    jsonSolver.setJSONArray("trajectoryGenerated", jsonPoints);


    JSONObject jsonStats = new JSONObject();
    jsonStats.setDouble("mean", stats.mean());
    jsonStats.setDouble("std", stats.std());
    jsonStats.setDouble("maxError", stats.maxError());
    jsonStats.setDouble("minError", stats.minError());
    jsonSolver.setJSONObject("stats", jsonStats);
  }

  public static void generateLissajousCurve(int n, Util.ConstraintType constraintType, JSONObject jsonObject, float x_speed, float y_speed, float z_speed, float radius) {
    JSONArray jsonPath = new JSONArray();
    PApplet pa = new PApplet();
    pa.randomSeed(0);
    pa.noiseSeed(0);
    List<Node> chain = Util.generateDetachedChain(numJoints, boneLength, randRotation, randLength);
    Util.generateConstraints(chain, constraintType, 0, true);
    targetPositions = new ArrayList<Vector>();
    Vector init = chain.get(chain.size() - 1).position();
    float step = 360f / n;
    float mean_dist = 0;
    for (float angle = 0; angle < 360 + step; angle += step) {
      float rad = pa.radians(angle);
      float x = radius * pa.cos(x_speed * rad);
      float y = radius * pa.sin(y_speed * rad);
      float z = radius * pa.sin(z_speed * rad);
      targetPositions.add(new Vector(init.x() + x, init.y() - (y + radius * 1.2f), init.z() + z));
      if (angle > 0)
        mean_dist += Vector.distance(targetPositions.get(targetPositions.size() - 1), targetPositions.get(targetPositions.size() - 2));
    }

    mean_dist /= targetPositions.size();
    System.out.println("mean dist : " + mean_dist);
    //add the parameters to the list
    //save the values in the json file
    for (int i = 0; i < targetPositions.size(); i++) {
      Vector v = targetPositions.get(i);
      jsonPath.append(v.x());
      jsonPath.append(v.y());
      jsonPath.append(v.z());
    }
    jsonObject.setJSONArray("trajectory", jsonPath);
  }

  public static void generateRandomPath(int n, Util.ConstraintType constraintType, JSONObject jsonObject) {
    JSONArray jsonPath = new JSONArray();
    PApplet pa = new PApplet();
    pa.randomSeed(0);
    List<Node> chain = Util.generateDetachedChain(numJoints, boneLength, randRotation, randLength);
    Util.generateConstraints(chain, constraintType, 0, true);
    targetPositions = new ArrayList<Vector>();
    float step = 0.01f;
    float last = step * n;
    float mean_dist = 0;
    for (float t = 0; t < last; t += step) {
      //Generate a random near pose
      for (int i = 0; i < chain.size(); i++) {
        //if(random.nextFloat() > 0.4f) {
        float angle = 2 * PI * pa.noise(1000 * i + t) - PI;
        Vector dir = new Vector(pa.noise(10000 * i + t), pa.noise(20000 * i + t), pa.noise(30000 * i + t));
        chain.get(i).setRotation(new Quaternion(dir, angle));
        //}
      }
      targetPositions.add(chain.get(chain.size() - 1).position().get());
      if (t > 0) {
        mean_dist += Vector.distance(targetPositions.get(targetPositions.size() - 1), targetPositions.get(targetPositions.size() - 2));
      }
    }
    mean_dist /= targetPositions.size();

    System.out.println("mean dist : " + mean_dist);

    //add the parameters to the list
    //save the values in the json file
    for (int i = 0; i < targetPositions.size(); i++) {
      Vector v = targetPositions.get(i);
      jsonPath.append(v.x());
      jsonPath.append(v.y());
      jsonPath.append(v.z());
    }
    jsonObject.setJSONArray("trajectory", jsonPath);
  }

  public static void main(String args[]) {
    NodeInformation.disableCache = true; //disable cache for "highly" precision benchmarking
    random.setSeed(seed);
    int numSolvers = solversType.length;
    JSONObject jsonExperiment = new JSONObject();
    JSONArray jsonSolverArray = new JSONArray();

    for (int c = 0; c < constraintTypes.length; c++) {
      if (enableLissajous)
        generateLissajousCurve(numPostures, constraintTypes[c], jsonExperiment, lissajousPath[0], lissajousPath[1], lissajousPath[2], lissajousPath[3]);
      else generateRandomPath(numPostures, constraintTypes[c], jsonExperiment);
      for (int i = 0; i < numSolvers; i++) {
        JSONObject jsonSolver = new JSONObject();
        generateExperiment(solversType[i], constraintTypes[c], 100, jsonSolver);
        jsonSolver.setString("name", solversType[i].name());
        jsonSolverArray.append(jsonSolver);
      }
      jsonExperiment.setJSONArray("solvers", jsonSolverArray);
      //save json file
      String name = "continuous_path_joints_" + numJoints + "_postures_" + numPostures + "_" + constraintTypes[c];
      if (enableLissajous)
        name += "_lissajous_" + "x_" + lissajousPath[0] + "_y" + lissajousPath[1] + "_z" + lissajousPath[2];
      else name += "_seed_" + seed;
      jsonExperiment.save(new File(dir + name + ".json"), null);
    }
  }

  public static class StatisticsPerExperiment {
    protected List<Vector> _points = new ArrayList<Vector>();
    protected List<Double> _errorValues = new ArrayList<Double>();
    protected List<Double> _distances = new ArrayList<Double>();
    protected List<Double> _orientationDistances = new ArrayList<Double>();

    protected double _minError = Float.MAX_VALUE, _maxError = Float.MIN_VALUE, _mean, _std, _meanDistances, _meanOrientationDistances;

    public double minError() {
      return _minError;
    }

    public double meanDistances() {
      return _meanDistances;
    }

    public double maxError() {
      return _maxError;
    }

    public double mean() {
      return _mean;
    }

    public double std() {
      return _std;
    }

    public List<Double> values() {
      return _errorValues;
    }

    public List<Double> distances() {
      return _distances;
    }

    public List<Double> orientationDistances() {
      return _orientationDistances;
    }

    public List<Vector> points() {
      return _points;
    }

    public float std(double mean) {
      double sum = 0;
      for (Double value : _errorValues) {
        sum += (value - mean) * (value - mean);
      }
      return (float) Math.sqrt(sum / _errorValues.size());
    }

    public void addPoint(Vector vector) {
      _points.add(vector);
    }

    public void addValue(double value) {
      _errorValues.add(value);
    }

    public void addDistance(double dist) {
      _distances.add(dist);
    }

    public void addOrientationDistance(double dist) {
      _orientationDistances.add(dist);
    }

    public void updateStatistics() {
      for (Double value : _errorValues) {
        _minError = Math.min(value, _minError);
        _maxError = Math.max(value, _maxError);
        _mean += value;
      }
      _mean = _mean / _errorValues.size();
      _std = std(_mean);

      _meanDistances = 0;
      for (Double value : _distances) {
        _meanDistances += value;
      }
      _meanDistances = _meanDistances / _distances.size();

      _meanOrientationDistances = 0;
      for (Double value : _orientationDistances) {
        _meanOrientationDistances += value;
      }
      _meanOrientationDistances = _meanOrientationDistances / _orientationDistances.size();
    }
  }

  public static List<Vector> obtainPositions(List<? extends Node> structure) {
    List<Vector> positions = new ArrayList<Vector>();
    for (Node node : structure) {
      positions.add(node.position().get());
    }
    return positions;
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

  public static double orientationDistanceBetweenPostures(List<Quaternion> prev, List<Quaternion> cur) {
    double dist = 0;
    for (int i = 0; i < prev.size(); i++) {
      dist += quaternionDistance(prev.get(i), cur.get(i));
    }
    return dist / prev.size();
  }

  public static double quaternionDistance(Quaternion a, Quaternion b) {
    double s1 = 1, s2 = 1;
    if (a.w() < 0) s1 = -1;
    if (b.w() < 0) s2 = -1;
    double dot = s1 * a._quaternion[0] * s2 * b._quaternion[0] + s1 * a._quaternion[1] * s2 * b._quaternion[1] + s1 * a._quaternion[2] * s2 * b._quaternion[2] + s1 * a._quaternion[3] * s2 * b._quaternion[3];
    dot = Math.max(Math.min(dot, 1), -1);
    return Math.acos(2 * Math.pow(dot, 2) - 1);
  }

}
