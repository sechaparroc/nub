package ik.paper;

import ik.basic.Util;
import nub.core.Node;
import nub.ik.solver.Solver;
import nub.ik.solver.trik.NodeInformation;
import nub.ik.solver.trik.heuristic.TRIKECTIK;
import nub.ik.solver.trik.implementations.IKSolver;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import processing.core.PApplet;
import processing.data.JSONArray;
import processing.data.JSONObject;
import processing.data.Table;
import processing.data.TableRow;

import java.io.File;
import java.io.IOException;
import java.util.*;

public class AccuracyVSTime {
    static float PI = (float) Math.PI;
    static Random random = new Random();
    static int seed = 0;
    //Benchmark Parameters
    static boolean continuousPath = true;
    static boolean lissajous = false;

    static String dir = "C:/Users/olgaa/Desktop/Sebas/Thesis/Results/AccuracyVSTime/";

    static int numStructures = 100;
    static int numPostures = 500; //Set the number of different postures to solve
    static int randRotation = -1; //Set seed to generate initial random rotations, otherwise set to -1
    static int randLength = 0; //Set seed to generate random segment lengths, otherwise set to -1
    static float boneLength = 50; //Define length of segments (bones)
    static int numJoints;

    static float accuracyThreshold = continuousPath ? 0.001f : 0.001f;


    static Util.ConstraintType constraintTypes[] = {
            Util.ConstraintType.NONE,
            Util.ConstraintType.HINGE,
            //Util.ConstraintType.HINGE_ALIGNED,
            Util.ConstraintType.CONE_ELLIPSE,
            //Util.ConstraintType.MIX,
            Util.ConstraintType.MIX_CONSTRAINED
    }; //Choose what kind of constraints apply to chain

    static Util.SolverType solversType[] = {
        Util.SolverType.CCD_HEURISTIC,
        Util.SolverType.BACK_AND_FORTH_CCD_HEURISTIC,
        Util.SolverType.TRIANGULATION_HEURISTIC,
        Util.SolverType.BACK_AND_FORTH_TRIANGULATION_HEURISTIC,
        Util.SolverType.TRIK_HEURISTIC,
        Util.SolverType.BACK_AND_FORTH_TRIK_HEURISTIC,
        Util.SolverType.COMBINED_HEURISTIC,
        Util.SolverType.COMBINED_TRIK,
        //Util.SolverType.COMBINED_EXPRESSIVE,
    }; //Place Here Solvers that you want to compare

    static List<Vector> targetPositions;
    static List<Quaternion> initialConfig;

    static HashMap<Util.SolverType, SolverStats> _statisticsPerSolver = new HashMap<>();

    public static void generateExperiment(Util.SolverType type, SolverStats solverStats, Util.ConstraintType constraintType, int iterations, int seed) {
        //1. Generate structure
        List<Node> structure = Util.generateDetachedChain(numJoints, boneLength, randRotation, randLength);
        if(continuousPath){
          for(int i = 0; i < initialConfig.size(); i++){
            structure.get(i).setRotation(initialConfig.get(i).get());
          }
        }

        //Save the current values
        List<Quaternion> rotations = new ArrayList<Quaternion>();
        for(Node n : structure) rotations.add(n.rotation().get());

        Node endEffector = structure.get(structure.size() - 1);
        //2. Apply constraints
        Util.generateConstraints(structure, constraintType, seed, true);
        //3. generate solver
        Solver solver = Util.createSolver(type, structure);
        //4. Define solver parameters
        solver.setMaxError(-1);
        solver.setMaxIterations(iterations);
        solver.setTimesPerFrame(1);
        solver.setMinDistance(-1);
        if(solver instanceof IKSolver){
            IKSolver ikSolver = (IKSolver) solver;
            ikSolver.enableDeadLockResolution(false);
            if(ikSolver.heuristic() instanceof TRIKECTIK){
                TRIKECTIK heuristic = (TRIKECTIK) ikSolver.heuristic();
                heuristic.setTRIKFraction(continuousPath ? 0.3f : 0.05f); //First 5 iterations will use TRIK the others use combined heuristic
            }
            if(ikSolver.mode() == IKSolver.HeuristicMode.COMBINED_EXPRESSIVE){
                ikSolver.context().setDelegationIterationsRatio(continuousPath ? 0.4f : 0.1f);  //Apply smoothing on first ten iterations
            }
        }

        //5. Set target
        Node target = new Node();
        target.setPosition(endEffector.position());
        target.setOrientation(endEffector.orientation());
        solver.setTarget(endEffector, target);
        int sample = 0;

        List<Vector> prevPos = obtainPositions(structure), nextPos;
        List<Quaternion> prevOrs = obtainOrientations(structure), nextOrs;
        List<Quaternion> prevRots = obtainRotations(structure), nextRots;

        for (Vector t : targetPositions) {
            if (sample % 100 == 0) System.out.println(type.name() + "On sample : " + sample);
            target.setPosition(t.get());
            solver.change(true);
            float minError = Float.MAX_VALUE;
            int lastIteration = -1;
            long elapsedTime = 0;

            for (int i = 0; i < iterations; i++) {
                long start = System.nanoTime();
                solver.solve();
                elapsedTime += System.nanoTime() - start;
                minError = solver.error();
                lastIteration = i + 1;
                if(IKSolver.log){
                    System.out.println("Error " + minError);
                }
                if(minError <= accuracyThreshold){
                    break;
                }
            }

            if(minError > Float.MAX_VALUE){
                solver.error();
                System.out.println("Auxiliar");
                for(NodeInformation ni : ((IKSolver)solver).context().usableChainInformation()){
                    Vector cache = ni.positionCache();
                    Vector real = ni.node().position();
                    System.out.println("Cache : " + cache + " Real : " + real + " Diff" + Vector.distance(cache, real));
                }

                System.out.println("Current");
                for(NodeInformation ni : ((IKSolver)solver).context().chainInformation()){
                    Vector cache = ni.positionCache();
                    Vector real = ni.node().position();
                    System.out.println("Cache : " + cache + " Real : " + real + " Diff" + Vector.distance(cache, real));
                }
                //Last rotation
                System.out.println("Rot " + ((IKSolver)solver).context().chainInformation().get(structure.size() - 3).node().rotation());
                System.out.println("Rot " + ((IKSolver)solver).context().usableChainInformation().get(structure.size() - 3).node().rotation());

                System.out.println("Rot " + ((IKSolver)solver).context().chainInformation().get(structure.size() - 2).node().rotation());
                System.out.println("Rot " + ((IKSolver)solver).context().usableChainInformation().get(structure.size() - 2).node().rotation());
                System.out.println("target " + target.position());
                //solver.change(true);
                //solver.solve();
                throw new RuntimeException("ex" + minError + " it " + lastIteration);
            }


            solverStats.errorSt.addValue(minError);
            solverStats.iterationSt.addValue(lastIteration);
            solverStats.timeSt.addValue(elapsedTime / 1000000.0);

            nextPos = obtainPositions(structure);
            solverStats.distancePosSt.addValue(distanceBetweenPostures(prevPos, nextPos));
            prevPos = nextPos;

            nextOrs = obtainOrientations(structure);
            solverStats.distanceOrientationSt.addValue(distanceBetweenRotations(prevOrs, nextOrs));
            prevOrs = nextOrs;

            nextRots = obtainRotations(structure);
            double distRot = distanceBetweenRotations(prevRots, nextRots);
            solverStats.distanceRotationSt.addValue(distRot);
            if(distRot > 0.01f)solverStats.motionSt.addValue(motionDistribution(prevRots, nextRots, distRot));
            prevRots = nextRots;

            sample++;
        }
    }

    public static void generateRandomReachablePositions(int n, int seed, Util.ConstraintType constraintType) {
        List<Node> chain = Util.generateDetachedChain(numJoints, boneLength, randRotation, randLength);
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
                if(random.nextFloat() < 0.2f){
                    //Totally random and possibly unreacheable
                    targetPositions.add(new Vector(boneLength * numJoints * random.nextFloat(), boneLength * numJoints * random.nextFloat(), boneLength * numJoints * random.nextFloat()));
                }
                targetPositions.add(des);
            }

        }
    }

    public static void generateRandomPath(int n, int seed, Util.ConstraintType constraintType) {
        PApplet pa = new PApplet();
        pa.randomSeed(0);
        pa.noiseSeed(0);
        List<Node> chain = Util.generateDetachedChain(numJoints, boneLength, randRotation, randLength);
        Util.generateConstraints(chain, constraintType, seed, true);
        targetPositions = new ArrayList<Vector>();
        initialConfig = new ArrayList<Quaternion>();
        float step = 0.01f;
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

    public static void generateLissajousCurve(int n, int seed, Util.ConstraintType constraintType, float x_speed, float y_speed, float z_speed, float radius) {
        PApplet pa = new PApplet();
        pa.randomSeed(seed);
        pa.noiseSeed(seed);
        List<Node> chain = Util.generateDetachedChain(numJoints, boneLength, randRotation, randLength);
        Util.generateConstraints(chain, constraintType, seed, true);

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

        mean_dist /= targetPositions.size() - 1;
        System.out.println("mean dist : " + mean_dist);
    }


    public static void main(String args[]) {
        NodeInformation.disableCache = false; //disable cache for "highly" precision benchmarking
        int numSolvers = solversType.length;
        for(int k = 0; k < constraintTypes.length; k++) {
            _statisticsPerSolver.clear();
            random.setSeed(seed);
            //prepare random path/targets
            JSONObject jsonExperiment = new JSONObject();
            JSONArray jsonSolverArray = new JSONArray();

            //Initialize statistics per solver
            for (int s = 0; s < numSolvers; s++) {
                _statisticsPerSolver.put(solversType[s], new SolverStats());
            }

            int c_seed = 0;
            Util.ConstraintType constraintType = constraintTypes[k];

            for (int s = 0; s < numStructures; s++) {
                numJoints = random.nextInt(16) + 4;
                System.out.println("On structure " + s + " joints : " + numJoints + " constraint : " + constraintType);
                if (continuousPath){
                    if(lissajous){
                        generateLissajousCurve(numPostures,
                            c_seed,
                            constraintType,
                            0.5f *  random.nextFloat() * 4.5f,
                            0.5f *  random.nextFloat() * 4.5f,
                            0.5f *  random.nextFloat() * 4.5f,
                            boneLength * numJoints * 0.2f
                            );
                    } else {
                        generateRandomPath(numPostures, c_seed, constraintType);
                    }
                }
                else generateRandomReachablePositions(numPostures, c_seed, constraintType);


                for (int i = 0; i < numSolvers; i++) {
                    generateExperiment(solversType[i], _statisticsPerSolver.get(solversType[i]), constraintType, continuousPath ? 50 : 50, c_seed);
                }
                c_seed = random.nextInt(100000);
            }

            for (int s = 0; s < numSolvers; s++) {
                //Save all info in a json
                JSONObject jsonSolver = new JSONObject();
                jsonSolver.setString("name", solversType[s].name());
                jsonSolver.setJSONObject("error stats", _statisticsPerSolver.get(solversType[s]).errorSt.save());
                jsonSolver.setJSONObject("iteration stats", _statisticsPerSolver.get(solversType[s]).iterationSt.save());
                jsonSolver.setJSONObject("time stats", _statisticsPerSolver.get(solversType[s]).timeSt.save());
                jsonSolver.setJSONObject("dist pos stats", _statisticsPerSolver.get(solversType[s]).distancePosSt.save());
                jsonSolver.setJSONObject("dist rot stats", _statisticsPerSolver.get(solversType[s]).distanceRotationSt.save());
                jsonSolver.setJSONObject("dist ors stats", _statisticsPerSolver.get(solversType[s]).distanceOrientationSt.save());
                jsonSolver.setJSONObject("motion stats", _statisticsPerSolver.get(solversType[s]).motionSt.save());
                jsonSolverArray.append(jsonSolver);
            }
            jsonExperiment.setJSONArray("solvers", jsonSolverArray);
            //save json file
            String name = constraintType.name();
            name += continuousPath ? "continuous_path" : "discontinuous_path";
            name += continuousPath && lissajous ? "_lissajous" : "";

            //add the parameters to the files name
            name += "_joints_" + numJoints + "_postures_" + numPostures + "_structures_" + numStructures + "_seed_" + seed;
            jsonExperiment.save(new File(dir + name + ".json"), null);

            //Save the info in a cvs table
            Table table = new Table();
            table.addColumn("Solver");
            table.addColumn("Max error");
            table.addColumn("Min error");
            table.addColumn("Avg error");
            table.addColumn("Std error");
            table.addColumn("Med error");
            table.addColumn("Med Std error");

            table.addColumn("Max iterations");
            table.addColumn("Min iterations");
            table.addColumn("Avg iterations");
            table.addColumn("Std iterations");
            table.addColumn("Med iterations");
            table.addColumn("Med Std iterations");

            table.addColumn("Max time");
            table.addColumn("Min time");
            table.addColumn("Avg time");
            table.addColumn("Std time");
            table.addColumn("Med time");
            table.addColumn("Med Std time");


            table.addColumn("Max dist pos");
            table.addColumn("Min dist pos");
            table.addColumn("Avg dist pos");
            table.addColumn("Std dist pos");
            table.addColumn("Med dist pos");
            table.addColumn("Med Std dist pos");

            table.addColumn("Max dist rot");
            table.addColumn("Min dist rot");
            table.addColumn("Avg dist rot");
            table.addColumn("Std dist rot");
            table.addColumn("Med dist rot");
            table.addColumn("Med Std dist rot");

            table.addColumn("Max dist ors");
            table.addColumn("Min dist ors");
            table.addColumn("Avg dist ors");
            table.addColumn("Std dist ors");
            table.addColumn("Med dist ors");
            table.addColumn("Med Std dist ors");

            table.addColumn("Max motion");
            table.addColumn("Min motion");
            table.addColumn("Avg motion");
            table.addColumn("Std motion");
            table.addColumn("Med motion");
            table.addColumn("Med Std motion");


            for (int s = 0; s < numSolvers; s++) {
                TableRow row = table.addRow();
                row.setString("Solver", solversType[s].name());
                row.setDouble("Max error", _statisticsPerSolver.get(solversType[s]).errorSt._max);
                row.setDouble("Min error", _statisticsPerSolver.get(solversType[s]).errorSt._min);
                row.setDouble("Avg error", _statisticsPerSolver.get(solversType[s]).errorSt._mean);
                row.setDouble("Std error", _statisticsPerSolver.get(solversType[s]).errorSt._std);
                row.setDouble("Med error", _statisticsPerSolver.get(solversType[s]).errorSt._median);
                row.setDouble("Med Std error", _statisticsPerSolver.get(solversType[s]).errorSt._stdMedian);

                row.setDouble("Max iterations", _statisticsPerSolver.get(solversType[s]).iterationSt._max);
                row.setDouble("Min iterations", _statisticsPerSolver.get(solversType[s]).iterationSt._min);
                row.setDouble("Avg iterations", _statisticsPerSolver.get(solversType[s]).iterationSt._mean);
                row.setDouble("Std iterations", _statisticsPerSolver.get(solversType[s]).iterationSt._std);
                row.setDouble("Med iterations", _statisticsPerSolver.get(solversType[s]).iterationSt._median);
                row.setDouble("Med Std iterations", _statisticsPerSolver.get(solversType[s]).iterationSt._stdMedian);


                row.setDouble("Max time", _statisticsPerSolver.get(solversType[s]).timeSt._max);
                row.setDouble("Min time", _statisticsPerSolver.get(solversType[s]).timeSt._min);
                row.setDouble("Avg time", _statisticsPerSolver.get(solversType[s]).timeSt._mean);
                row.setDouble("Std time", _statisticsPerSolver.get(solversType[s]).timeSt._std);
                row.setDouble("Med time", _statisticsPerSolver.get(solversType[s]).timeSt._median);
                row.setDouble("Med Std time", _statisticsPerSolver.get(solversType[s]).timeSt._stdMedian);


                row.setDouble("Max dist pos", _statisticsPerSolver.get(solversType[s]).distancePosSt._max);
                row.setDouble("Min dist pos", _statisticsPerSolver.get(solversType[s]).distancePosSt._min);
                row.setDouble("Avg dist pos", _statisticsPerSolver.get(solversType[s]).distancePosSt._mean);
                row.setDouble("Std dist pos", _statisticsPerSolver.get(solversType[s]).distancePosSt._std);
                row.setDouble("Med dist pos", _statisticsPerSolver.get(solversType[s]).distancePosSt._median);
                row.setDouble("Med Std dist pos", _statisticsPerSolver.get(solversType[s]).distancePosSt._stdMedian);

                row.setDouble("Max dist rot", _statisticsPerSolver.get(solversType[s]).distanceRotationSt._max);
                row.setDouble("Min dist rot", _statisticsPerSolver.get(solversType[s]).distanceRotationSt._min);
                row.setDouble("Avg dist rot", _statisticsPerSolver.get(solversType[s]).distanceRotationSt._mean);
                row.setDouble("Std dist rot", _statisticsPerSolver.get(solversType[s]).distanceRotationSt._std);
                row.setDouble("Med dist rot", _statisticsPerSolver.get(solversType[s]).distanceRotationSt._median);
                row.setDouble("Med Std dist rot", _statisticsPerSolver.get(solversType[s]).distanceRotationSt._stdMedian);

                row.setDouble("Max dist ors", _statisticsPerSolver.get(solversType[s]).distanceOrientationSt._max);
                row.setDouble("Min dist ors", _statisticsPerSolver.get(solversType[s]).distanceOrientationSt._min);
                row.setDouble("Avg dist ors", _statisticsPerSolver.get(solversType[s]).distanceOrientationSt._mean);
                row.setDouble("Std dist ors", _statisticsPerSolver.get(solversType[s]).distanceOrientationSt._std);
                row.setDouble("Med dist ors", _statisticsPerSolver.get(solversType[s]).distanceOrientationSt._median);
                row.setDouble("Med Std dist ors", _statisticsPerSolver.get(solversType[s]).distanceOrientationSt._stdMedian);

                row.setDouble("Max motion", _statisticsPerSolver.get(solversType[s]).motionSt._max);
                row.setDouble("Min motion", _statisticsPerSolver.get(solversType[s]).motionSt._min);
                row.setDouble("Avg motion", _statisticsPerSolver.get(solversType[s]).motionSt._mean);
                row.setDouble("Std motion", _statisticsPerSolver.get(solversType[s]).motionSt._std);
                row.setDouble("Med motion", _statisticsPerSolver.get(solversType[s]).motionSt._median);
                row.setDouble("Med Std motion", _statisticsPerSolver.get(solversType[s]).motionSt._stdMedian);
            }

            try {
                table.save(new File(dir + name + ".csv"), null);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
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

    public static double distanceBetweenRotations(List<Quaternion> prev, List<Quaternion> cur) {
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

    public static class SolverStats{
        Statistics errorSt = new Statistics(), iterationSt = new Statistics(), timeSt = new Statistics();
        Statistics distancePosSt = new Statistics(), distanceOrientationSt = new Statistics(), distanceRotationSt = new Statistics(), motionSt = new Statistics();
    }

    public static class Statistics {
        protected List<Double> _values = new ArrayList<Double>();

        protected double _min = Float.MAX_VALUE, _max = Float.MIN_VALUE, _mean, _std, _median, _stdMedian;

        public double min() {
            return _min;
        }

        public double max() {
            return _max;
        }

        public double mean() {
            return _mean;
        }

        public double std() {
            return _std;
        }

        public double median(){
        return _median;
      }

        public double stdMedian(){
        return _stdMedian;
      }

        public float std(double mean) {
            double sum = 0;
            for (Double value : _values) {
                sum += (value - mean) * (value - mean);
            }
            return (float) Math.sqrt(sum / _values.size());
        }

        public void addValue(double value) {
            _values.add(value);
        }

        public void updateStatistics() {
            for (Double value : _values) {
                _min = Math.min(value, _min);
                _max = Math.max(value, _max);
                _mean += value;
            }
            _mean = _mean / _values.size();
            _std = std(_mean);
            ArrayList<Double> copy = new ArrayList<Double>(_values);
            Collections.sort(copy);
            int n = copy.size();
            if (n % 2 == 0)
              _median = (copy.get(n/2) + copy.get(n/2 - 1))/2.;
            else
              _median = copy.get(n/2);
            _stdMedian = std(_median);
        }

        public JSONObject save(){
            updateStatistics();
            JSONObject stats = new JSONObject();
            stats.setDouble("mean", _mean);
            stats.setDouble("std", _std);
            stats.setDouble("min", _min);
            stats.setDouble("max", _max);
            stats.setDouble("median", _median);
            stats.setDouble("median_std", _stdMedian);

            JSONArray values = new JSONArray();
            for(double val : _values){
                values.append(val);
            }
            stats.setJSONArray("values", values);
            return stats;
        }
    }
}
