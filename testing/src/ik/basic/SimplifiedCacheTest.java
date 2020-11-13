package ik.basic;

import nub.core.Node;
import nub.core.constraint.BallAndSocket;
import nub.ik.solver.NodeInformation;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class SimplifiedCacheTest {
    static Random r = new Random(1), r1 = new Random(1), r2 = new Random(1);
    static int numJoints = 20;
    static float boneLength = 50;
    //Parameters
    static Util.ConstraintType constraintType = Util.ConstraintType.MIX; //Choose what kind of constraints apply to chain
    static int randRotation = 0; //Set seed to generate initial random rotations, otherwise set to -1
    static int randLength = 0; //Set seed to generate random segment lengths, otherwise set to -1
    static boolean debug = false; //set true to debug purposes


    public void setConstraint(float down, float up, float left, float right, Node f, Vector twist, float boneLength) {
        BallAndSocket constraint = new BallAndSocket(down, up, left, right);
        constraint.setRestRotation(f.rotation().get(), f.displacement(new Vector(0, 1, 0)), f.displacement(twist));
        f.setConstraint(constraint);
    }

    public static ArrayList<NodeInformation> generateChain(int numJoints, float boneLength, int randRotation, int randLength) {
        ArrayList<NodeInformation> chain = new ArrayList<NodeInformation>();
        Random r1 = randRotation != -1 ? new Random(randRotation) : null;
        Random r2 = randLength != -1 ? new Random(randLength) : null;

        NodeInformation prevJoint = null;
        for (int i = 0; i < numJoints; i++) {
            Node joint = Node.detach(new Vector(), new Quaternion(), 1);
            if (prevJoint != null) joint.setReference(prevJoint.node());
            float x = 0;
            float y = 1;
            float z = 0;
            if (r1 != null) {
                x = 2 * r1.nextFloat() - 1;
                z = r1.nextFloat();
                y = 2 * r1.nextFloat() - 1;
            }
            Vector translate = new Vector(x, y, z);
            translate.normalize();
            if (r2 != null)
                translate.multiply(boneLength * (1 - 0.4f * r2.nextFloat()));
            else
                translate.multiply(boneLength);
            joint.setTranslation(translate);
            NodeInformation nodeInformation = new NodeInformation(prevJoint, joint);
            prevJoint = nodeInformation;
            chain.add(nodeInformation);
        }
        return chain;
    }

    public static void applyRandomRotations(List<NodeInformation> structure, Random random) {
        //first define how many rotations to apply
        int rotations = random.nextInt(structure.size()) + 1;
        if (debug) {
            System.out.println("#####Random Rots: ");
            System.out.println("\t\tnumber of rotations : \t" + rotations);
        }
        for (int i = 0; i < rotations; i++) {
            //Generate a random Quaternion
            Quaternion q = new Quaternion(random.nextFloat() * (float) Math.PI,
                    random.nextFloat() * (float) Math.PI, random.nextFloat() * (float) Math.PI);
            //Pick a random joint
            int idx = random.nextInt(structure.size());
            structure.get(idx).rotateAndUpdateCache(q, true);
            if (debug) {
                System.out.println("\t\tRotation at node : \t" + idx + " : " + structure.get(idx).node().rotation().axis() + " ang : " + Math.toDegrees((structure.get(idx).node().rotation().angle())));
            }
        }
    }

    public static long performQueries(List<NodeInformation> structure, Random random, List<Node> outl) {
        outl.clear();
        //Ask for some queries
        long queryTime = 0;
        int queries = random.nextInt(structure.size() * 2) + 1;
        if (debug) {
            System.out.println("#####Random Queries: ");
            System.out.println("\t\tnumber of queries : \t" + queries);
        }
        //        queries = 1;
        for (int i = 0; i < queries; i++) {
            Node out = Node.detach(new Vector(), new Quaternion(), 1);
            //Pick a random joint
            int idx = i % structure.size();
            if (debug) {
                System.out.println("\t\tquery at node : \t" + idx);
            }
            //System.out.println(idx);
            //Ask for position / orientation
            long startTime = System.nanoTime();
            if(idx != 0)structure.get(idx).updateCacheUsingReference();
            out.setTranslation(structure.get(idx).positionCache());
            out.setRotation(structure.get(idx).orientationCache());
            outl.add(out);
            queryTime += System.nanoTime() - startTime;
            if (debug) {
                System.out.println("\t\tobtained pos : \t" + out.position());
                System.out.println("\t\tobtained ori : \t" + out.orientation().axis() + " ang : " + out.orientation().angle());
            }
        }
        return queryTime;
    }


    public static void main(String args[]) throws Exception {
        //testing position/orientation cache on a chain of N joints
        ArrayList<NodeInformation> chain1 = generateChain(numJoints, boneLength, randRotation, randLength);
        ArrayList<NodeInformation> chain2 = generateChain(numJoints, boneLength, randRotation, randLength);
        ArrayList<Node> ch1 = new ArrayList<Node>();
        for(NodeInformation ni : chain1){ ch1.add(ni.node());}
        ArrayList<Node> ch2 = new ArrayList<Node>();
        for(NodeInformation ni : chain2){ ch2.add(ni.node());}
        Util.generateConstraints(ch1, constraintType, 0, true);
        Util.generateConstraints(ch2, constraintType, 0, true);
        NodeInformation._updateCache(chain1);
        NodeInformation._updateCache(chain2);

        List<Node> ol1 = new ArrayList<Node>();
        List<Node> ol2 = new ArrayList<Node>();

        float total_dist = 0;
        long queries1 = 0, queries2 = 0;
        for (int i = 0; i < 500000; i++) {
            //Apply rotations
            if (r.nextFloat() <= 0.3) {
                applyRandomRotations(chain1, r1);
                applyRandomRotations(chain2, r2);
            } else {
                NodeInformation.disableCache = false;
                queries1 += performQueries(chain1, r1, ol1);
                NodeInformation.disableCache = true;
                queries2 += performQueries(chain2, r2, ol2);
                for (int j = 0; j < ol1.size(); j++) {
                    Node o1 = ol1.get(j);
                    Node o2 = ol2.get(j);
                    total_dist += Vector.distance(o1.translation(), o2.translation());
                    if (Vector.distance(o1.translation(), o2.translation()) > 0.1) {
                        System.out.println("P1 " + o1.translation());
                        System.out.println("P2 " + o2.translation());
                        throw new Exception("o1 and o2 positions must be equal");
                    }
                    if (Math.abs(o1.rotation().x() - o2.rotation().x()) > 0.001 ||
                            Math.abs(o1.rotation().y() - o2.rotation().y()) > 0.001 ||
                            Math.abs(o1.rotation().z() - o2.rotation().z()) > 0.001 ||
                            Math.abs(o1.rotation().w() - o2.rotation().w()) > 0.001) {
                        System.out.println("o1 " + o1.rotation().axis() + " ang : " + Math.toDegrees(o1.rotation().angle()));
                        System.out.println("o2 " + o2.rotation().axis() + " ang : " + Math.toDegrees(o2.rotation().angle()));
                        throw new Exception("o1 and o2 orientations must be equal");
                    }
                }
            }
        }
        System.out.println("queries with cache time: \t" + queries1);
        System.out.println("queries without cache: \t" + queries2);
        System.out.println("difference: \t" + (long) (queries2 - queries1));
        System.out.println("prop: \t" + (1. * queries2 / queries1));
        System.out.println("Accumulated error after whole experiment " + total_dist);
    }
}
