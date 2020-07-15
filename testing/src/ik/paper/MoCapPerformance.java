package ik.paper;

import ik.basic.Util;
import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.animation.Joint;
import nub.ik.loader.bvh.BVHLoader;
import nub.ik.solver.trik.Context;
import nub.ik.solver.trik.Tree;
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
import java.util.Map;

public class MoCapPerformance {
    static String dir = "C:/Users/olgaa/Desktop/Sebas/Thesis/Results/MoCapPerformance/";

    static IKSolver.HeuristicMode heuristicsMode[] = {
            IKSolver.HeuristicMode.CCD,
            IKSolver.HeuristicMode.BACK_AND_FORTH_CCD,
            IKSolver.HeuristicMode.TRIANGULATION,
            IKSolver.HeuristicMode.BACK_AND_FORTH_TRIANGULATION,
            IKSolver.HeuristicMode.TRIK,
            IKSolver.HeuristicMode.BACK_AND_FORTH_TRIK,
            IKSolver.HeuristicMode.COMBINED,
            IKSolver.HeuristicMode.COMBINED_EXPRESSIVE,
    }; //Place Here Solvers that you want to compare

    static int iterationsChain = 10;
    static int iterations = 5;
    static float maxError = 0.01f;

    public static List<BVHPackage> generateZooPackages(){
        List<BVHPackage> packages = new ArrayList<BVHPackage>();
        String zooInput = "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/";
        String[] names = {"Parrot"};
        for(String name : names){
            packages.add(new BVHPackage(zooInput, name));
        }
        return packages;
    }


    public static BVHLoader loadBVH(String path){
        BVHLoader loader = new BVHLoader(Node.class, path, null);
        loader.setLoop(false);

        //skip first two frames
        loader.nextPose(true);
        loader.nextPose(true);
        //generate constraints
        loader.generateConstraints();
        return loader;
    }

    public static void generateExperiment(BVHLoader loader, IKSolver.HeuristicMode mode, BVHStats stats, float height){
        //1. reset the loader
        loader.poseAt(0);
        //2. create the appropriate skeleton
        Skeleton skeleton = new Skeleton(loader, mode);
        skeleton.createSolver(maxError, iterationsChain, iterations);
        //3. Try to reach the desired postures and collect the statistics
        System.out.println("Solver" + mode.name() + " Current Pose " + loader.currentPose() + " total : " + loader.poses());
        while(loader.currentPose() < loader.poses()){
            skeleton.readPose();
            //solve and collect
            skeleton.solver.change(true);
            long start = System.nanoTime();
            skeleton.solver.solve();
            stats.timeSt.addValue((System.nanoTime() - start) / 1000000.0);
            stats.positionSt.addValue(skeleton.positionDistance(height));
            stats.orientationSt.addValue(skeleton.orientationDistance());
            loader.nextPose();
        }
    }

    public static void main(String args[]) {
        //! generate BVH packages
        List<BVHPackage>  packages = generateZooPackages();
        for(BVHPackage bvhP : packages) {
            Table table = generateTablePerPackage();
            for(String bvhF : bvhP.files) {
                List<BVHStats> statsList = new ArrayList<>();
                //load the appropriate bvh
                System.out.println(bvhF);
                BVHLoader loader = loadBVH(bvhP.path + bvhP.name + "/" + bvhF);
                for (int s = 0; s < heuristicsMode.length; s++) {
                    //Stats per BVH and solver
                    BVHStats stats = new BVHStats();
                    stats.name = heuristicsMode[s].name();
                    generateExperiment(loader, heuristicsMode[s], stats, bvhP.height);
                    stats.update();
                    statsList.add(stats);
                }
                addRow(bvhF, statsList, table);
            }
            //save all this in a cvs file
            try {
                table.save(new File(dir + bvhP.name + ".csv"), null);
                //load file and replace comma and dot separators
                String[] strings = PApplet.loadStrings(new File(dir + bvhP.name + ".csv"));
                String[] newStrings = new String[strings.length];
                for(int i = 0; i < strings.length; i++){
                    newStrings[i] = strings[i].replace(",", ";");
                    newStrings[i] = newStrings[i].replace(".", ",");
                }
                //save strings
                PApplet.saveStrings(new File(dir + bvhP.name + ".csv"), newStrings);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static Table generateTablePerPackage(){
        Table table = new Table();
        table.addColumn("Animation");
        table.addColumn("Solver");
        table.addColumn("Max pos.");
        table.addColumn("Min pos.");
        table.addColumn("Avg. pos.");
        table.addColumn("Std. pos.");

        table.addColumn("Max or.");
        table.addColumn("Min or.");
        table.addColumn("Avg. or.");
        table.addColumn("Std. or.");

        table.addColumn("Max time");
        table.addColumn("Min time");
        table.addColumn("Avg. time");
        table.addColumn("Std. time");
        return table;

    }


    public static void addRow(String bvhName, List<BVHStats> stats, Table table){
        for(BVHStats stat : stats ){
            TableRow row = table.addRow();
            row.setString("Animation", bvhName);
            row.setString("Solver", stat.name);
            row.setDouble("Max pos.", stat.positionSt._max);
            row.setDouble("Min pos.", stat.positionSt._min);
            row.setDouble("Avg. pos.", stat.positionSt._mean);
            row.setDouble("Std. pos.", stat.positionSt._std);

            row.setDouble("Max or.", stat.orientationSt._max);
            row.setDouble("Min or.", stat.orientationSt._min);
            row.setDouble("Avg. or.", stat.orientationSt._mean);
            row.setDouble("Std. or.", stat.orientationSt._std);

            row.setDouble("Max time", stat.timeSt._max);
            row.setDouble("Min time", stat.timeSt._min);
            row.setDouble("Avg. time", stat.timeSt._mean);
            row.setDouble("Std. time", stat.timeSt._std);
        }
    }


    public static class Skeleton{
        Tree solver;
        IKSolver.HeuristicMode mode;
        BVHLoader loader;
        HashMap<String, Node> structure;
        HashMap<String, Node> targets;
        HashMap<Node, Node> jointToNode;
        List<String> endEffectors = new ArrayList<String>();


        Node root;

        public Skeleton(BVHLoader loader, IKSolver.HeuristicMode mode){
            this.loader = loader;
            this.mode = mode;
            createSkeleton();
        }

        void createSkeleton(){
            structure = new HashMap<>();
            jointToNode = new HashMap<>();
            HashMap<Node, Node> pairs = new HashMap<>();
            for(Node node : loader.branch()){
                Node joint = Node.detach(new Vector(), new Quaternion(), 1);
                joint.setReference(pairs.get(node.reference()));
                joint.setTranslation(node.translation().get());
                joint.setRotation(node.rotation().get());
                joint.setConstraint(node.constraint());
                pairs.put(node, joint);
                structure.put(loader.joint().get(node.id()).name(), joint);
                if(node.children() == null || node.children().size() == 0) endEffectors.add(loader.joint().get(node.id()).name());
                jointToNode.put(joint, node);
            }
            root = pairs.get(loader.root());
        }

        void createSolver(float maxError, int iterationsChain, int iterations){
            solver = new Tree(root, mode);
            //define attributes
            solver.setMaxError(maxError);
            solver.setTimesPerFrame(iterations);
            solver.setMaxIterations(iterations);
            solver.setChainTimesPerFrame(iterationsChain);
            solver.setChainMaxIterations(iterationsChain);
            targets = new HashMap<>();
            //Create a target per end effector
            for(String s : endEffectors){
                Node node = structure.get(s);
                Node target = Node.detach(new Vector(), new Quaternion(), 1);
                target.setPosition(node.position().get());
                target.setOrientation(node.orientation().get());
                targets.put(s, target);
                solver.addTarget(node, target);
            }
        }

        void readPose(){
            Constraint c = root.constraint();
            root.setConstraint(null);
            root.setPosition(loader.root().position().get());
            root.setOrientation(loader.root().orientation().get());
            root.setConstraint(c);
            //update targets if prev distance is high
            for(String s : endEffectors){
                Node joint = structure.get(s);
                Node target = targets.get(s);
                Node next = jointToNode.get(joint);
                target.setPosition(next.position().get());
                target.setOrientation(next.orientation().get());
                //modify end effector rotation
                joint.setRotation(next.rotation().get());
            }
        }

        double positionDistance(float height){
            double rms = 0;
            for(Node joint : structure.values()){
                rms += Vector.distance(joint.position(), jointToNode.get(joint).position());
            }
            rms /= structure.size();
            rms /= height;
            return rms;
        }

        double orientationDistance(){
            double dist = 0;
            for (Node joint : structure.values()) {
                dist += Math.toRadians(Context.orientationError(joint.rotation(), jointToNode.get(joint).rotation(), true));
            }
            return dist / structure.size();
        }

    }

    public static class BVHPackage{
        String path;
        String name;
        List<String> files = new ArrayList<String>();
        float height;

        public BVHPackage(String p, String n){
            path = p;
            name = n;
            getFilesFromPath();
            calculateHeight();
            System.out.println("Height " + height);
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

        void calculateHeight(){
            String t_pose = null;
            int idx = 0;
            for(String s : files){
                if(s.toUpperCase().contains("TPOSE")){
                    t_pose = s;
                    break;
                }
                idx++;
            }
            if(t_pose == null){
                //choose any file
                t_pose = files.get(0);
            } else{
                files.remove(idx);
            }
            BVHLoader auxLoader = new BVHLoader(Node.class, path + name + "/" + t_pose, null);
            Vector max = new Vector(), min = new Vector(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
            max = Vector.multiply(min, -1);
            for(Node n : auxLoader.branch()){
                Vector pos = auxLoader.root().location(n);
                if(max.x() < pos.x()) max.setX(pos.x());
                if(max.y() < pos.y()) max.setY(pos.y());
                if(max.z() < pos.z()) max.setZ(pos.z());
                if(min.x() > pos.x()) min.setX(pos.x());
                if(min.y() > pos.y()) min.setY(pos.y());
                if(min.z() > pos.z()) min.setZ(pos.z());
            }
            float mX = max.x() - min.x();
            float mY = max.y() - min.y();
            float mZ = max.z() - min.z();
            height = Math.max(Math.max(mX, mY), mZ);
        }
    }


    public static class BVHStats{
        String name;
        Statistics positionSt = new Statistics(), orientationSt = new Statistics(), timeSt = new Statistics();

        void update(){
            positionSt.updateStatistics();
            orientationSt.updateStatistics();
            timeSt.updateStatistics();
        }
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
