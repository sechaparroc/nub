package ik.paper;

import ik.basic.Util;
import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.solver.Solver;
import nub.ik.solver.trik.NodeInformation;
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
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

public class AccuracyVSTime {
    static float PI = (float) Math.PI;
    static Random random = new Random();
    static int seed = 0;
    static String dir = "C:/Users/olgaa/Desktop/Sebas/Thesis/Results/AccuracyVSTime/";

    //Benchmark Parameters
    static boolean continuousPath = false;
    static int numStructures = 100;
    static int numPostures = 1000; //Set the number of different postures to solve
    static int randRotation = -1; //Set seed to generate initial random rotations, otherwise set to -1
    static int randLength = 0; //Set seed to generate random segment lengths, otherwise set to -1
    static float boneLength = 50; //Define length of segments (bones)
    static int numJoints;

    static float accuracyThreshold = 0.01f;


    static Util.ConstraintType constraintTypes[] = {
            Util.ConstraintType.NONE,
            Util.ConstraintType.HINGE,
            Util.ConstraintType.HINGE_ALIGNED,
            Util.ConstraintType.CONE_CIRCLE,
            Util.ConstraintType.CONE_ELLIPSE,
            Util.ConstraintType.MIX,
            Util.ConstraintType.MIX_CONSTRAINED
    }; //Choose what kind of constraints apply to chain

    static Util.SolverType solversType[] = {
            Util.SolverType.CCD_HEURISTIC,
            Util.SolverType.TRIANGULATION_HEURISTIC,
            Util.SolverType.BACK_AND_FORTH_TRIANGULATION_HEURISTIC,
            Util.SolverType.CCD_HEURISTIC,
            Util.SolverType.BACK_AND_FORTH_CCD_HEURISTIC,
            Util.SolverType.TRIK_HEURISTIC,
            Util.SolverType.BACK_AND_FORTH_TRIK_HEURISTIC,
            Util.SolverType.COMBINED_HEURISTIC,
            Util.SolverType.COMBINED_EXPRESSIVE,
            Util.SolverType.FABRIK
    }; //Place Here Solvers that you want to compare

    static List<Vector> targetPositions;

    static HashMap<Util.SolverType, SolverStats> _statisticsPerSolver = new HashMap<>();

    public static void generateExperiment(Util.SolverType type, SolverStats solverStats, Util.ConstraintType constraintType, int iterations, int seed) {
        //1. Generate structure
        List<Node> structure = Util.generateDetachedChain(numJoints, boneLength, randRotation, randLength);
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
        //5. Set target
        Node target = new Node();
        target.setPosition(endEffector.position());
        target.setOrientation(endEffector.orientation());
        solver.setTarget(endEffector, target);
        int sample = 0;
        for (Vector t : targetPositions) {
            if (sample % 100 == 0) System.out.println(type.name() + "On sample : " + sample);
            //if(s == 3 && sample == 30)IKSolver.debugERROR = true;
            //reset to initial position
            /*for(int k = 0; k < rotations.size(); k++){
                Constraint c = structure.get(k).constraint();
                structure.get(k).setConstraint(null);
                structure.get(k).setRotation(rotations.get(k).get());
                structure.get(k).setConstraint(c);
            }*/

            target.setPosition(t.get());
            solver.change(true);
            float minError = Float.MAX_VALUE;
            int lastIteration = -1;
            long elapsedTime = 0;

            // -- TODO: SAVE WORST CONFIGURATIONS AND SHOW THEM IN THE PAPER
            String info = "";
            for(Node node : structure) {
                info += "j = new Joint();" + "\n";
                info +="j.setReference(prev);" + "\n";
                info += "j.setTranslation(" + node.translation().x() + "f, "
                        + node.translation().y() + "f, "
                        + node.translation().z() + "f);" + "\n";
                Quaternion q = node.rotation();
                info +="q = new Quaternion(" + q.x() + "f, " + + q.y() + "f, " + q.z() + "f, " + q.w() + "f);" + "\n";
                info +="j.setRotation(q); " + "\n";
                info += "prev = j;" + "\n";
                info += "chain.add(j);" + "\n";
            }


            for (int i = 0; i < iterations; i++) {
                long start = System.nanoTime();
                solver.solve();
                elapsedTime += System.nanoTime() - start;
                minError = solver.error();
                lastIteration = i + 1;
                if(IKSolver.debugERROR){
                    System.out.println("Error " + minError);
                }
                if(minError <= accuracyThreshold){
                    break;
                }
            }

            if(minError > Float.MAX_VALUE){
                solver.error();
                System.out.println(info);

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
            //System.out.println(elapsedTime + " its " + lastIteration + " joints " + structure.size());
            sample++;
        }
    }

    public static void generateRandomReachablePositions(int n, int seed, Util.ConstraintType constraintType) {
        List<Node> chain = Util.generateDetachedChain(numJoints, boneLength, randRotation, randLength);
        Util.generateConstraints(chain, constraintType, seed, true);

        targetPositions = new ArrayList<Vector>();
        for (int t = 0; t < n; t++) {
            for (int i = 0; i < chain.size() - 1; i++) {
                if (random.nextFloat() > 0.4f) {
                    chain.get(i).rotate(new Quaternion(new Vector(0, 0, 1), random.nextFloat() * 2 * PI - PI));
                    chain.get(i).rotate(new Quaternion(new Vector(0, 1, 0), random.nextFloat() * 2 * PI - PI));
                    chain.get(i).rotate(new Quaternion(new Vector(1, 0, 0), random.nextFloat() * 2 * PI - PI));
                }
            }
            //save the position of the target
            targetPositions.add(chain.get(chain.size() - 1).position().get());
        }
    }

    public static void generateRandomPath(int n, int seed, Util.ConstraintType constraintType) {
        PApplet pa = new PApplet();
        pa.randomSeed(0);
        List<Node> chain = Util.generateDetachedChain(numJoints, boneLength, randRotation, randLength);
        Util.generateConstraints(chain, constraintType, seed, true);

        targetPositions = new ArrayList<Vector>();
        float last = 0.005f * n;
        for (float t = 0; t < last; t += 0.005f) {
            //Generate a random near pose
            for (int i = 0; i < chain.size(); i++) {
                float angle = 2 * PI * pa.noise(1000 * i + t) - PI;
                Vector dir = new Vector(pa.noise(10000 * i + t), pa.noise(20000 * i + t), pa.noise(30000 * i + t));
                chain.get(i).rotate(dir, angle);
            }
            targetPositions.add(chain.get(chain.size() - 1).position().get());
        }
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
                for (int i = 0; i < numSolvers; i++) {
                    if (continuousPath) generateRandomPath(numPostures, c_seed, constraintType);
                    else generateRandomReachablePositions(numPostures, c_seed, constraintType);
                    generateExperiment(solversType[i], _statisticsPerSolver.get(solversType[i]), constraintType, 100, c_seed);
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
                jsonSolverArray.append(jsonSolver);
            }
            jsonExperiment.setJSONArray("solvers", jsonSolverArray);
            //save json file
            String name = constraintType.name();
            name += continuousPath ? "continuous_path" : "discontinuous_path";
            //add the parameters to the files name
            name += "_joints_" + numJoints + "_postures_" + numPostures + "_structures_" + numStructures + "_seed_" + seed;
            jsonExperiment.save(new File(dir + name + ".json"), null);

            //Save the info in a cvs table
            Table table = new Table();
            table.addColumn("Solver");
            table.addColumn("Max error");
            table.addColumn("Min error");
            table.addColumn("Avg. error");
            table.addColumn("Std. error");
            table.addColumn("Max iterations");
            table.addColumn("Min iterations");
            table.addColumn("Avg. iterations");
            table.addColumn("Std. iterations");
            table.addColumn("Max time");
            table.addColumn("Min time");
            table.addColumn("Avg. time");
            table.addColumn("Std. time");

            for (int s = 0; s < numSolvers; s++) {
                TableRow row = table.addRow();
                row.setString("Solver", solversType[s].name());
                row.setDouble("Max error", Math.round(_statisticsPerSolver.get(solversType[s]).errorSt._max * 1000.0) / 1000.0);
                row.setDouble("Min error", Math.round(_statisticsPerSolver.get(solversType[s]).errorSt._min * 1000.0) / 1000.0);
                row.setDouble("Avg. error", Math.round(_statisticsPerSolver.get(solversType[s]).errorSt._mean * 1000.0) / 1000.0);
                row.setDouble("Std. error", Math.round(_statisticsPerSolver.get(solversType[s]).errorSt._std * 1000.0) / 1000.0);

                row.setDouble("Max iterations", Math.round(_statisticsPerSolver.get(solversType[s]).iterationSt._max * 1000.0) / 1000.0);
                row.setDouble("Min iterations", Math.round(_statisticsPerSolver.get(solversType[s]).iterationSt._min * 1000.0) / 1000.0);
                row.setDouble("Avg. iterations", Math.round(_statisticsPerSolver.get(solversType[s]).iterationSt._mean * 1000.0) / 1000.0);
                row.setDouble("Std. iterations", Math.round(_statisticsPerSolver.get(solversType[s]).iterationSt._std * 1000.0) / 1000.0);

                row.setDouble("Max time", Math.round(_statisticsPerSolver.get(solversType[s]).timeSt._max * 1000.0) / 1000.0);
                row.setDouble("Min time", Math.round(_statisticsPerSolver.get(solversType[s]).timeSt._min * 1000.0) / 1000.0);
                row.setDouble("Avg. time", Math.round(_statisticsPerSolver.get(solversType[s]).timeSt._mean * 1000.0) / 1000.0);
                row.setDouble("Std. time", Math.round(_statisticsPerSolver.get(solversType[s]).timeSt._std * 1000.0) / 1000.0);
            }

            try {
                table.save(new File(dir + name + ".csv"), null);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static class SolverStats{
        Statistics errorSt = new Statistics(), iterationSt = new Statistics(), timeSt = new Statistics();
    }

    public static class Statistics {
        protected List<Double> _values = new ArrayList<Double>();

        protected double _min = Float.MAX_VALUE, _max = Float.MIN_VALUE, _mean, _std;

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

        public float std(double mean) {
            int sum = 0;
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
        }

        public JSONObject save(){
            updateStatistics();
            JSONObject stats = new JSONObject();
            stats.setDouble("mean", _mean);
            stats.setDouble("std", _std);
            stats.setDouble("min", _min);
            stats.setDouble("max", _max);
            JSONArray values = new JSONArray();
            for(double val : _values){
                values.append(val);
            }
            stats.setJSONArray("values", values);
            return stats;
        }
    }
}
