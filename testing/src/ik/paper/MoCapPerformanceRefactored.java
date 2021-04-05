package ik.paper;

import ik.basic.Util;
import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.animation.Skeleton;
import nub.ik.loader.bvh.BVHLoader;
import nub.ik.solver.GHIK;
import nub.ik.solver.GHIKTree;
import nub.ik.solver.Solver;
import nub.ik.solver.fabrik.FABRIKTree;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.data.JSONArray;
import processing.data.JSONObject;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MoCapPerformanceRefactored {
  static String dir = "C:/Users/olgaa/Desktop/Sebas/Thesis/Results/Clean/Multiple/data/";
  static Util.SolverType heuristicsMode[] = {
      Util.SolverType.CCD,
      Util.SolverType.TIK,
      Util.SolverType.TRIK,
      Util.SolverType.FABRIK_O,
      Util.SolverType.BFCCD,
      Util.SolverType.BFTIK,
      Util.SolverType.BFTRIK,
      Util.SolverType.BFIK,
  }; //Place Here Solvers that you want to compare
  static int iterationsChain = 5;
  static int iterations = 5;
  static float maxError = 0.001f;
  static int effs;
  static float height = -1;
  static String startAt = "Tyranno";
  static int n_packages = 1;

  public static List<BVHPackage> generateZooPackages(){
    List<BVHPackage> packages = new ArrayList<BVHPackage>();
    String zooInput = "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/";
    //Find all files
    File[] directories = new File(zooInput).listFiles(File::isDirectory);

    boolean start = false;
    int n = 0;
    for(File f : directories){
      if (f.getName().toUpperCase().equals(startAt.toUpperCase())) {
        start = true;
      }
      if(!start) continue;
      if(n == n_packages) break;
      System.out.println("Name " + f.getName());
      BVHPackage bvhP = new BVHPackage(zooInput, f.getName());
      if(bvhP.files.size() > 0) {
        packages.add(bvhP);
        n++;
      } else{
        System.err.println("Name " + f.getName() + " was excluded");
      }
    }
    return packages;
  }

  public static BVHLoader loadBVH(String path, boolean calcHeight){
    BVHLoader loader = new BVHLoader(path, null);
    loader.setLoop(false);
    //skip first two frames
    loader.nextPosture(true);
    loader.nextPosture(true);
    //Use proper EFFS
    humanoidEFFs(loader);
    //generate constraints
    loader.generateConstraints();
    //Calculate height
    float h = calculateHeight(loader);
    if(calcHeight) height = h;
    return loader;
  }

  public static void generateExperiment(String skname, String file, BVHLoader loader, Util.SolverType mode, SolverData solverData, float height){
    //1. reset the loader
    loader.postureAt(0);
    //2. create the appropriate skeleton
    BVHSkeleton bvhSkeleton = new BVHSkeleton(loader, mode);
    bvhSkeleton.createSolver(maxError, iterationsChain, iterations, height);
    effs = bvhSkeleton.skeleton.endEffectors().size();
    //3. Try to reach the desired postures and collect the statistics
    System.out.println("Solver" + mode.name() + " Current Pose " + loader.currentPosture() + " total : " + loader.postures());
    HashMap<Node, Vector> prevPos = bvhSkeleton.obtainPositions(), nextPos;
    HashMap<Node, Quaternion> prevOrs = bvhSkeleton.obtainOrientations(), nextOrs;
    HashMap<Node, Quaternion> prevRots = bvhSkeleton.obtainRotations(), nextRots;

    int idx = 0;
    while(loader.currentPosture() < loader.postures()){
      bvhSkeleton.updatePosture();
      //solve and collect statistics
      bvhSkeleton.solver.change(true);
      long start = System.nanoTime();
      bvhSkeleton.solver.solve();
      double end = (System.nanoTime() - start) / 1000000.0;

      float error = bvhSkeleton.solver.error() / bvhSkeleton.skeleton.endEffectors().size();
      error  = Math.max(0, error - 0.001f * height );
      error = error / height * 100f;

      solverData.solverNames.add(mode.name());
      solverData.fileNames.add(file);
      solverData.skeletonNames.add(skname);
      solverData.metrics.get("num").add(idx * 1.0);
      solverData.metrics.get("joints").add(bvhSkeleton.skeleton.joints().size() * 1.0);
      solverData.metrics.get("endEffectors").add(bvhSkeleton.skeleton.endEffectors().size() * 1.0);
      solverData.metrics.get("height").add(height * 1.0);
      solverData.metrics.get("distanceError").add(error * 1.0);
      solverData.metrics.get("iterations").add(1.0 * bvhSkeleton.solver.lastIteration());
      solverData.metrics.get("time").add(end);

      if(bvhSkeleton.solver instanceof GHIKTree) {
        solverData.metrics.get("orientationError").add(1.0 * ((GHIKTree)bvhSkeleton.solver).orientationError() / bvhSkeleton.skeleton.endEffectors().size());
      } else {
        solverData.metrics.get("orientationError").add(1.0 * ((FABRIKTree)bvhSkeleton.solver).orientationError() / bvhSkeleton.skeleton.endEffectors().size());
      }

      nextPos = bvhSkeleton.obtainPositions();
      solverData.metrics.get("meanJointPositionDistance").add(distanceBetweenPostures(prevPos, nextPos) / height * 100.0);
      prevPos = nextPos;

      nextOrs = bvhSkeleton.obtainOrientations();
      solverData.metrics.get("meanJointOrientationDistance").add(distanceBetweenRotations(prevOrs, nextOrs));
      prevOrs = nextOrs;

      nextRots = bvhSkeleton.obtainRotations();
      double distRot = distanceBetweenRotations(prevRots, nextRots);
      solverData.metrics.get("meanJointRotationDistance").add(distRot);
      if (distRot > 0.001f) {
        solverData.metrics.get("gini").add(motionDistribution(prevRots, nextRots, distRot));
      } else{
        solverData.metrics.get("gini").add(Double.NaN);
      }
      prevRots = nextRots;

      solverData.metrics.get("meanJointPositionError").add(bvhSkeleton.meanJointPositionError() / height * 100.0);
      solverData.metrics.get("meanJointRotationError").add(bvhSkeleton.meanJointRotationError() / height * 100.0);
      solverData.metrics.get("meanJointOrientationError").add(bvhSkeleton.meanJointOrientationError());
      loader.nextPosture();
      idx++;
    }
  }

  public static void main(String args[]) {
    //! generate BVH packages
    List<BVHPackage>  packages = generateZooPackages();
    for(BVHPackage bvhP : packages) {
      height = bvhP.height;
      SolverData solverData = new SolverData();
      System.out.println("On Package : " + bvhP.name);
      for(String bvhF : bvhP.files) {
        //load the appropriate bvh
        System.out.println(bvhF);
        BVHLoader loader = loadBVH(bvhP.path + bvhP.name + "/" + bvhF, bvhP.tpose_height == -1);
        for (int s = 0; s < heuristicsMode.length; s++) {
          //Stats per BVH and solver
          generateExperiment(bvhP.name, bvhF, loader, heuristicsMode[s], solverData, height);
        }
      }
      //save json file
      //Save json
      solverData.save().save(new File(dir + "metrics_" + bvhP.name + ".json"), null);
    }
  }


  //Convenient methods
  public static float calculateHeight(BVHLoader parser){ //calculates the height of the skeleton
    Vector min = new Vector(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
    Vector max = new Vector(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);
    float mX = 0;
    float mY = 0;
    float mZ = 0;

    int initial = parser.currentPosture();
    for(int i = initial; i < parser.postures(); i++) {
      parser.postureAt(i);
      for (Node n : parser.skeleton().joints().values()) {
        //Vector pos = parser.skeleton().reference().children().get(0).location(n);
        Vector pos = n.position().get();
        if (max.x() < pos.x()) max.setX(pos.x());
        if (max.y() < pos.y()) max.setY(pos.y());
        if (max.z() < pos.z()) max.setZ(pos.z());
        if (min.x() > pos.x()) min.setX(pos.x());
        if (min.y() > pos.y()) min.setY(pos.y());
        if (min.z() > pos.z()) min.setZ(pos.z());
      }
      mX = Math.max(max.x() - min.x(), mX);
      mY = Math.max(max.y() - min.y(),mY);
      mZ = Math.max(max.z() - min.z(),mZ);
    }
    parser.postureAt(initial);
    System.out.println("max" + max + " min " + min);
    System.out.println("mX" + mX + " mY " + mY + "mZ" + mZ);
    System.out.println("Height " + mY);
    return mY;
  }




  public static void humanoidEFFs(BVHLoader loader){

    Skeleton sk = loader.skeleton();
    String names[] = new String[]{"RTHUMB", "RIGHTHANDINDEX1", "LTHUMB", "LEFTTOEBASE", "RIGHTTOEBASE", "LEFTHANDINDEX1", "LEFTFINGERBASE", "RIGHTFINGERBASE"};
    for(String name : names){
      if(sk.joints().containsKey(name)) {
        Node n = sk.joint(name);
        sk.joints().remove(name);
        sk.names().remove(n);
        n.setReference(null);
        Scene.prune(n);
      }
    }
  }


  public static class BVHSkeleton{
    Solver solver;
    Util.SolverType mode;
    BVHLoader loader;
    Skeleton skeleton;

    public BVHSkeleton(BVHLoader loader, Util.SolverType mode){
      this.loader = loader;
      this.mode = mode;
      this.skeleton = loader.skeleton().get();
    }

    void createSolver(float maxError, int iterationsChain, int iterations, float height){
      //Create a target per end effector
      if(mode != Util.SolverType.FABRIK_O) {
        GHIK.HeuristicMode gmode = null;
        if(mode.equals(Util.SolverType.CCD)) gmode = GHIK.HeuristicMode.CCD;
        else if(mode.equals(Util.SolverType.TIK)) gmode = GHIK.HeuristicMode.TIK;
        else if(mode.equals(Util.SolverType.TRIK)) gmode = GHIK.HeuristicMode.TRIK;
        else if(mode.equals(Util.SolverType.BFCCD)) gmode = GHIK.HeuristicMode.BFIK_CCD;
        else if(mode.equals(Util.SolverType.BFTIK)) gmode = GHIK.HeuristicMode.BFIK_TIK;
        else if(mode.equals(Util.SolverType.BFTRIK)) gmode = GHIK.HeuristicMode.BFIK_TRIK;
        else if(mode.equals(Util.SolverType.BFIK)) gmode = GHIK.HeuristicMode.BFIK;
        if(gmode == null) throw new RuntimeException("Mode" + mode + "non supported...");

        GHIKTree gsolver = new GHIKTree(skeleton.reference().children().get(0), gmode);
        //define attributes
        gsolver.setMaxError(maxError * height); //1% of the skeleton height
        gsolver.setDirection(true, true);
        gsolver.setTimesPerFrame(iterations);
        gsolver.setMaxIterations(iterations);
        gsolver.setChainTimesPerFrame(1);
        gsolver.setChainMaxIterations(iterationsChain);
        solver = gsolver;
      } else{
        FABRIKTree fsolver = new FABRIKTree(skeleton.reference().children().get(0));
        //define attributes
        fsolver.setMaxError(maxError * height); //1% of the skeleton height
        fsolver.setTimesPerFrame(iterations);
        fsolver.setMaxIterations(iterations);
        fsolver.setChainTimesPerFrame(1);
        fsolver.setChainMaxIterations(iterationsChain);
        solver = fsolver;
      }
      for(Node node : skeleton.endEffectors()){
        Node target = Node.detach(new Vector(), new Quaternion(), 1);
        target.setPosition(node.position().get());
        target.setOrientation(node.orientation().get());
        skeleton.targets().put(skeleton.jointName(node), target);
        if(solver instanceof GHIKTree) ((GHIKTree) solver).addTarget(node, target);
        else if(solver instanceof FABRIKTree) ((FABRIKTree) solver).addTarget(node, target);
      }
    }

    void updatePosture(){
      //move the root of the skeleton
      for(int i = 0; i < loader.skeleton().reference().children().size(); i++){
        Node skeletonRoot = loader.skeleton().reference().children().get(i);
        Node root = skeleton.joint(loader.skeleton().jointName(skeletonRoot));
        Constraint c = root.constraint();
        root.setConstraint(null);
        root.setTranslation(skeletonRoot.translation().get());
        root.setRotation(skeletonRoot.rotation().get());
        root.setConstraint(c);
      }
      //Apply changes in translation
      for(Node skNode : loader.skeleton().BFS()){
        Node node = skeleton.joint(loader.skeleton().jointName(skNode));
        Constraint c = node.constraint();
        node.setConstraint(null);
        node.setTranslation(skNode.translation().get());
        node.setConstraint(c);
      }

      //Set the targets
      for(Map.Entry<String, Node> entry : skeleton.targets().entrySet()){
        Node desired =  loader.skeleton().joint(entry.getKey());
        //EFF own rotation is known
        entry.getValue().setPosition(desired.position().get());
        entry.getValue().setOrientation(desired.orientation().get());
      }
    }

    double meanJointPositionError(){
      double rms = 0;
      if(loader.skeleton().joints().size() != loader.skeleton().names().size())
        throw  new RuntimeException("Name map size: " + loader.skeleton().names().size() + " Joint map size: " + loader.skeleton().joints().size());
      for(String name : loader.skeleton().joints().keySet()){
        rms += Vector.distance(skeleton.joint(name).position(), loader.skeleton().joint(name).position());
      }
      rms /= loader.skeleton().joints().size();
      return rms;
    }

    double meanJointRotationError(){
      double dist = 0;
      if(loader.skeleton().joints().size() != loader.skeleton().names().size())
        throw  new RuntimeException("Name map size: " + loader.skeleton().names().size() + " Joint map size: " + loader.skeleton().joints().size());
      for(String name : loader.skeleton().joints().keySet()){
        dist += quaternionDistance(skeleton.joint(name).rotation(), loader.skeleton().joint(name).rotation());
      }
      return dist / loader.skeleton().joints().size();
    }

    double meanJointOrientationError(){
      double dist = 0;
      if(loader.skeleton().joints().size() != loader.skeleton().names().size())
        throw  new RuntimeException("Name map size: " + loader.skeleton().names().size() + " Joint map size: " + loader.skeleton().joints().size());
      for(String name : loader.skeleton().joints().keySet()){
        dist += quaternionDistance(skeleton.joint(name).orientation(), loader.skeleton().joint(name).orientation());
      }
      return dist / loader.skeleton().joints().size();
    }

    public HashMap<Node, Vector> obtainPositions() {
      HashMap<Node, Vector> positions = new HashMap<Node, Vector>();
      for (Node node : skeleton.BFS()) {
        positions.put(node, node.position().get());
      }
      return positions;
    }

    public HashMap<Node, Quaternion> obtainRotations() {
      HashMap<Node, Quaternion> rotations = new HashMap<Node, Quaternion>();
      for (Node node : skeleton.BFS()) {
        rotations.put(node, node.rotation().get());
      }
      return rotations;
    }

    public HashMap<Node, Quaternion> obtainOrientations() {
      HashMap<Node, Quaternion> rotations = new HashMap<Node, Quaternion>();
      for (Node node : skeleton.BFS()) {
        rotations.put(node, node.orientation().get());
      }
      return rotations;
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

  public static double distanceBetweenPostures(HashMap<Node, Vector> prev, HashMap<Node, Vector> cur) {
    double dist = 0;
    for (Node node  : prev.keySet()) {
      dist += Vector.distance(prev.get(node), cur.get(node));
    }
    return dist / prev.size();
  }

  public static double distanceBetweenRotations(HashMap<Node, Quaternion> prev, HashMap<Node, Quaternion> cur) {
    double dist = 0;
    for (Node node  : prev.keySet()) {
      dist += quaternionDistance(prev.get(node), cur.get(node));
    }
    return dist / prev.size();
  }

  public static double motionDistribution(HashMap<Node, Quaternion> prev, HashMap<Node, Quaternion> cur, double dist) {
    List<Double> vals = new ArrayList<Double>();
    for (Node node  : prev.keySet()) {
      vals.add(quaternionDistance(prev.get(node), cur.get(node)));
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


  public static class BVHPackage{
    String path;
    String name;
    List<String> files = new ArrayList<String>();
    float height;
    float tpose_height = -1;

    public BVHPackage(String p, String n){
      path = p;
      name = n;
      getFilesFromPath();
      if(files.size() == 0) return;
      calculateTPoseHeight();
      System.out.println("TPose Height " + height);
    }

    void getFilesFromPath(){
      files = new ArrayList<String>();
      File folder = new File(path + name);
      File[] listOfFiles = folder.listFiles();
      for (File file : listOfFiles) {
        if (file.isFile() && file.getName().contains("bvh")) {
          System.out.println(file.getName());
          files.add(file.getName());
        }
      }
    }

    void calculateTPoseHeight(){
      String t_pose = null;
      int idx = 0;
      for(String s : files){
        if(s.toUpperCase().contains("TPOSE")){
          t_pose = s;
          break;
        }
        idx++;
      }
      if(t_pose != null){
        files.remove(idx);
      } else return;
      BVHLoader auxLoader = new BVHLoader(path + name + "/" + t_pose, null);
      auxLoader.nextPosture(true);
      auxLoader.nextPosture(true);
      System.out.println("TPOSE --------------");
      tpose_height = calculateHeight(auxLoader);
      System.out.println("--------------------");
      height = tpose_height;
    }
  }

  //Auxiliar class to keep stats info
  public static class SolverData{
    List<String> solverNames = new ArrayList<>();
    List<String> fileNames = new ArrayList<>();
    List<String> skeletonNames = new ArrayList<>();
    HashMap<String, List<Double>> metrics = new HashMap<>();

    public SolverData(){
      initMetrics();
    }

    void initMetrics(){
      metrics.put("num", new ArrayList<>());
      metrics.put("joints", new ArrayList<>());
      metrics.put("endEffectors", new ArrayList<>());
      metrics.put("height", new ArrayList<>());
      metrics.put("distanceError", new ArrayList<>());
      metrics.put("orientationError", new ArrayList<>());
      metrics.put("iterations", new ArrayList<>());
      metrics.put("time", new ArrayList<>());
      metrics.put("meanJointPositionDistance", new ArrayList<>());
      metrics.put("meanJointOrientationDistance", new ArrayList<>());
      metrics.put("meanJointRotationDistance", new ArrayList<>());
      metrics.put("meanJointPositionError", new ArrayList<>());
      metrics.put("meanJointRotationError", new ArrayList<>());
      metrics.put("meanJointOrientationError", new ArrayList<>());
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
      if(solverNames.size() != skeletonNames.size()) throw new RuntimeException("Data incomplete: names " + solverNames.size() + " , constraints " + skeletonNames.size());
      JSONArray jsonSkeletonNames = new JSONArray();
      for(int i = 0; i < skeletonNames.size(); i++){
        jsonSkeletonNames.append(skeletonNames.get(i));
      }
      json.setJSONArray("skeleton", jsonSkeletonNames);
      if(solverNames.size() != fileNames.size()) throw new RuntimeException("Data incomplete: names " + solverNames.size() + " , continuous " + fileNames.size());
      JSONArray jsonFileNames = new JSONArray();
      for(int i = 0; i < fileNames.size(); i++){
        jsonFileNames.append(fileNames.get(i));
      }
      json.setJSONArray("file", jsonFileNames);

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
