package ik.paper;

import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.loader.bvh.BVHLoader;
import nub.ik.solver.GHIKTree;
import nub.ik.solver.GHIK;
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

public class MoCapPerformance {
    static String dir = "C:/Users/olgaa/Desktop/Sebas/Thesis/Results/MoCapPerformance/paper/";

    static GHIK.HeuristicMode heuristicsMode[] = {
            GHIK.HeuristicMode.CCD,
            GHIK.HeuristicMode.BFIK_CCD,
            GHIK.HeuristicMode.TIK,
            GHIK.HeuristicMode.BFIK_TIK,
            GHIK.HeuristicMode.TRIK,
            GHIK.HeuristicMode.BFIK_TRIK,
            GHIK.HeuristicMode.ECTIK,
            GHIK.HeuristicMode.TRIK_ECTIK,
            //GHIK.HeuristicMode.ECTIK_DAMP,
    }; //Place Here Solvers that you want to compare

    static int iterationsChain = 5;
    static int iterations = 5;
    static float maxError = 0.01f;
    static int effs;

    static String startAt = "Cat";
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


    public static BVHLoader loadBVH(String path){
        BVHLoader loader = new BVHLoader(path, null);
        loader.setLoop(false);

        //skip first two frames
        loader.nextPosture(true);
        loader.nextPosture(true);
        //generate constraints
        loader.generateConstraints();
        return loader;
    }

    public static void generateExperiment(BVHLoader loader, GHIK.HeuristicMode mode, BVHStats stats, float height){
        //1. reset the loader
        loader.postureAt(0);
        //2. create the appropriate skeleton
        Skeleton skeleton = new Skeleton(loader, mode);
        skeleton.createSolver(maxError, iterationsChain, iterations, height);
        effs = skeleton.endEffectors.size();
        //3. Try to reach the desired postures and collect the statistics
        System.out.println("Solver" + mode.name() + " Current Pose " + loader.currentPosture() + " total : " + loader.postures());

        HashMap<Node, Vector> prevPos = skeleton.obtainPositions(), nextPos;
        HashMap<Node, Quaternion> prevOrs = skeleton.obtainOrientations(), nextOrs;
        HashMap<Node, Quaternion> prevRots = skeleton.obtainRotations(), nextRots;

        while(loader.currentPosture() < loader.postures()){
            skeleton.readPosture();
            //solve and collect
            skeleton.solver.change(true);
            long start = System.nanoTime();
            skeleton.solver.solve();
            double end = (System.nanoTime() - start) / 1000000.0;

            stats.errorSt.addValue(skeleton.solver.error() / skeleton.endEffectors.size());
            stats.iterationSt.addValue(skeleton.solver.lastIteration());
            stats.timeSt.addValue(end);

            nextPos = skeleton.obtainPositions();
            stats.distancePosSt.addValue(distanceBetweenPostures(prevPos, nextPos));
            prevPos = nextPos;

            nextOrs = skeleton.obtainOrientations();
            stats.distanceOrientationSt.addValue(distanceBetweenRotations(prevOrs, nextOrs));
            prevOrs = nextOrs;

            nextRots = skeleton.obtainRotations();
            double distRot = distanceBetweenRotations(prevRots, nextRots);
            stats.distanceRotationSt.addValue(distRot);
            if(distRot > Float.MIN_VALUE) stats.motionSt.addValue(motionDistribution(prevRots, nextRots, distRot));
            prevRots = nextRots;

            stats.positionErrorSt.addValue(skeleton.positionDistance());
            stats.rotationErrorSt.addValue(skeleton.rotationDistance());
            stats.orientationErrorSt.addValue(skeleton.orientationDistance());
            loader.nextPosture();
        }
    }

    public static void main(String args[]) {
        //! generate BVH packages
        List<BVHPackage>  packages = generateZooPackages();
        for(BVHPackage bvhP : packages) {
            System.out.println("On Package : " + bvhP.name);
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
                addRow(bvhF, statsList, table, bvhP.height, effs);
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
        table.addColumn("Height");
        table.addColumn("Effs");
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

        table.addColumn("Max pos error");
        table.addColumn("Min pos error");
        table.addColumn("Avg pos error");
        table.addColumn("Std pos error");
        table.addColumn("Med pos error");
        table.addColumn("Med Std pos error");


        table.addColumn("Max rot error");
        table.addColumn("Min rot error");
        table.addColumn("Avg rot error");
        table.addColumn("Std rot error");
        table.addColumn("Med rot error");
        table.addColumn("Med Std rot error");

        table.addColumn("Max ors error");
        table.addColumn("Min ors error");
        table.addColumn("Avg ors error");
        table.addColumn("Std ors error");
        table.addColumn("Med ors error");
        table.addColumn("Med Std ors error");


        return table;
    }


    public static void addRow(String bvhName, List<BVHStats> stats, Table table, double height, int n){
        for(BVHStats stat : stats ){
            TableRow row = table.addRow();
            row.setString("Animation", bvhName);
            row.setString("Solver", stat.name);
            row.setDouble("Height", height);
            row.setInt("Effs", n);

            row.setDouble("Max error", stat.errorSt._max);
            row.setDouble("Min error", stat.errorSt._min);
            row.setDouble("Avg error", stat.errorSt._mean);
            row.setDouble("Std error", stat.errorSt._std);
            row.setDouble("Med error", stat.errorSt._median);
            row.setDouble("Med Std error", stat.errorSt._stdMedian);

            row.setDouble("Max iterations", stat.iterationSt._max);
            row.setDouble("Min iterations", stat.iterationSt._min);
            row.setDouble("Avg iterations", stat.iterationSt._mean);
            row.setDouble("Std iterations", stat.iterationSt._std);
            row.setDouble("Med iterations", stat.iterationSt._median);
            row.setDouble("Med Std iterations", stat.iterationSt._stdMedian);


            row.setDouble("Max time", stat.timeSt._max);
            row.setDouble("Min time", stat.timeSt._min);
            row.setDouble("Avg time", stat.timeSt._mean);
            row.setDouble("Std time", stat.timeSt._std);
            row.setDouble("Med time", stat.timeSt._median);
            row.setDouble("Med Std time", stat.timeSt._stdMedian);

            row.setDouble("Max dist pos", stat.distancePosSt._max);
            row.setDouble("Min dist pos", stat.distancePosSt._min);
            row.setDouble("Avg dist pos", stat.distancePosSt._mean);
            row.setDouble("Std dist pos", stat.distancePosSt._std);
            row.setDouble("Med dist pos", stat.distancePosSt._median);
            row.setDouble("Med Std dist pos", stat.distancePosSt._stdMedian);

            row.setDouble("Max dist rot", stat.distanceRotationSt._max);
            row.setDouble("Min dist rot", stat.distanceRotationSt._min);
            row.setDouble("Avg dist rot", stat.distanceRotationSt._mean);
            row.setDouble("Std dist rot", stat.distanceRotationSt._std);
            row.setDouble("Med dist rot", stat.distanceRotationSt._median);
            row.setDouble("Med Std dist rot", stat.distanceRotationSt._stdMedian);

            row.setDouble("Max dist ors", stat.distanceOrientationSt._max);
            row.setDouble("Min dist ors", stat.distanceOrientationSt._min);
            row.setDouble("Avg dist ors", stat.distanceOrientationSt._mean);
            row.setDouble("Std dist ors", stat.distanceOrientationSt._std);
            row.setDouble("Med dist ors", stat.distanceOrientationSt._median);
            row.setDouble("Med Std dist ors", stat.distanceOrientationSt._stdMedian);

            row.setDouble("Max motion", stat.motionSt._max);
            row.setDouble("Min motion", stat.motionSt._min);
            row.setDouble("Avg motion", stat.motionSt._mean);
            row.setDouble("Std motion", stat.motionSt._std);
            row.setDouble("Med motion", stat.motionSt._median);
            row.setDouble("Med Std motion", stat.motionSt._stdMedian);

            row.setDouble("Max pos error", stat.positionErrorSt._max);
            row.setDouble("Min pos error", stat.positionErrorSt._min);
            row.setDouble("Avg pos error", stat.positionErrorSt._mean);
            row.setDouble("Std pos error", stat.positionErrorSt._std);
            row.setDouble("Med pos error", stat.positionErrorSt._median);
            row.setDouble("Med Std pos error", stat.positionErrorSt._stdMedian);

            row.setDouble("Max rot error", stat.rotationErrorSt._max);
            row.setDouble("Min rot error", stat.rotationErrorSt._min);
            row.setDouble("Avg rot error", stat.rotationErrorSt._mean);
            row.setDouble("Std rot error", stat.rotationErrorSt._std);
            row.setDouble("Med rot error", stat.rotationErrorSt._median);
            row.setDouble("Med Std rot error", stat.rotationErrorSt._stdMedian);

            row.setDouble("Max ors error", stat.rotationErrorSt._max);
            row.setDouble("Min ors error", stat.rotationErrorSt._min);
            row.setDouble("Avg ors error", stat.rotationErrorSt._mean);
            row.setDouble("Std ors error", stat.rotationErrorSt._std);
            row.setDouble("Med ors error", stat.rotationErrorSt._median);
            row.setDouble("Med Std ors error", stat.rotationErrorSt._stdMedian);

        }
    }


    public static class Skeleton{
        GHIKTree solver;
        GHIK.HeuristicMode mode;
        BVHLoader loader;
        HashMap<String, Node> structure;
        HashMap<String, Node> targets;
        HashMap<Node, Node> jointToNode;
        List<String> endEffectors = new ArrayList<String>();


        Node root;

        public Skeleton(BVHLoader loader, GHIK.HeuristicMode mode){
            this.loader = loader;
            this.mode = mode;
            createSkeleton();
        }

        void createSkeleton(){
            structure = new HashMap<>();
            jointToNode = new HashMap<>();
            HashMap<Node, Node> pairs = new HashMap<>();
            for(Node node : loader.skeleton().BFS()){
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
            root = pairs.get(loader.skeleton().reference().children().get(0));
        }

        void createSolver(float maxError, int iterationsChain, int iterations, float height){
            solver = new GHIKTree(root, mode);
            //define attributes
            solver.setMaxError(maxError * height); //1% of the skeleton height
            solver.setDirection(true);
            solver.setTimesPerFrame(iterations);
            solver.setMaxIterations(iterations);
            solver.setChainTimesPerFrame(1);
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

        void readPosture(){
            Constraint c = root.constraint();
            root.setConstraint(null);
            root.setPosition(loader.skeleton().reference().children().get(0).position().get());
            root.setOrientation(loader.skeleton().reference().children().get(0).orientation().get());
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

        double positionDistance(){
            double rms = 0;
            for(Node joint : structure.values()){
                rms += Vector.distance(joint.position(), jointToNode.get(joint).position());
            }
            rms /= structure.size();
            return rms;
        }

        double rotationDistance(){
            double dist = 0;
            for (Node joint : structure.values()) {
                dist += quaternionDistance(joint.rotation(), jointToNode.get(joint).rotation());
            }
            return dist / structure.size();
        }

        double orientationDistance(){
            double dist = 0;
            for (Node joint : structure.values()) {
                dist += quaternionDistance(joint.orientation(), jointToNode.get(joint).orientation());
            }
            return dist / structure.size();
        }

        public HashMap<Node, Vector> obtainPositions() {
            HashMap<Node, Vector> positions = new HashMap<Node, Vector>();
            for (Node node : structure.values()) {
                positions.put(node, node.position().get());
            }
            return positions;
        }

        public HashMap<Node, Quaternion> obtainRotations() {
            HashMap<Node, Quaternion> rotations = new HashMap<Node, Quaternion>();
            for (Node node : structure.values()) {
                rotations.put(node, node.rotation().get());
            }
            return rotations;
        }

        public HashMap<Node, Quaternion> obtainOrientations() {
            HashMap<Node, Quaternion> rotations = new HashMap<Node, Quaternion>();
            for (Node node : structure.values()) {
                rotations.put(node, node.orientation().get());
            }
            return rotations;
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
            if(files.size() == 0) return;
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
            BVHLoader auxLoader = new BVHLoader(path + name + "/" + t_pose, null);
            Vector max = new Vector(), min = new Vector(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
            max = Vector.multiply(min, -1);
            for(Node n : auxLoader.skeleton().BFS()){
                Vector pos = auxLoader.skeleton().reference().location(n);
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
        Statistics errorSt = new Statistics(), iterationSt = new Statistics(), timeSt = new Statistics();
        Statistics distancePosSt = new Statistics(), distanceOrientationSt = new Statistics(), distanceRotationSt = new Statistics(), motionSt = new Statistics();
        Statistics positionErrorSt = new Statistics(), rotationErrorSt = new Statistics(), orientationErrorSt = new Statistics();

        void update(){
            errorSt.updateStatistics();
            iterationSt.updateStatistics();
            timeSt.updateStatistics();
            distancePosSt.updateStatistics();
            distanceOrientationSt.updateStatistics();
            distanceRotationSt.updateStatistics();
            motionSt.updateStatistics();
            positionErrorSt.updateStatistics();
            rotationErrorSt.updateStatistics();
            orientationErrorSt.updateStatistics();
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
