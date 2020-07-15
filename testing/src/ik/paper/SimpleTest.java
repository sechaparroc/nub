package ik.paper;

import ik.basic.Util;
import nub.core.Graph;
import nub.core.Node;
import nub.ik.animation.Joint;
import nub.ik.solver.geometric.ChainSolver;
import nub.ik.solver.trik.implementations.IKSolver;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.processing.TimingTask;
import processing.core.PApplet;
import processing.core.PShape;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.List;

public class SimpleTest extends PApplet {
    Scene scene;
    //Set the scene as P3D or P2D
    String renderer = P3D;
    float jointRadius = 5;
    float length = 50;
    boolean enableSolver = true;
    //Skeleton structure defined above
    ArrayList<Node> skeleton = new ArrayList<Node>();
    IKSolver solver;

    public void settings() {
        size(700, 700, renderer);
    }

    /**Place here a Structure to verify*/
    public ArrayList<Node> generateChain(){
        ArrayList<Node> chain = new ArrayList<Node>();
        Joint prev = null;
        Joint j;
        Quaternion q;
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 35.380642f, 0.0f);
        q = new Quaternion(-0.21005954f, 2.9569835E-8f, 0.23597586f, 0.94878364f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 33.37118f, 0.0f);
        q = new Quaternion(0.0702232f, 1.3096238E-8f, -0.05162037f, 0.9961948f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 45.189274f, 0.0f);
        q = new Quaternion(-0.06865316f, 1.35041756E-8f, -0.05369193f, 0.9961947f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 37.873096f, 0.0f);
        q = new Quaternion(0.14758569f, 7.803583E-9f, -0.35866117f, 0.92172706f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 37.25165f, 0.0f);
        q = new Quaternion(0.22126411f, 3.5341259E-9f, -0.070011966f, 0.9726976f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 43.81899f, 0.0f);
        q = new Quaternion(0.05126529f, 4.504642E-9f, -0.13185364f, 0.98994267f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 38.99126f, 0.0f);
        q = new Quaternion(-0.050407086f, -3.661299E-8f, -0.0052334797f, 0.9987151f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 47.659866f, 0.0f);
        q = new Quaternion(-0.017571568f, 2.2038702E-8f, -0.029294996f, 0.99941635f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 38.049095f, 0.0f);
        q = new Quaternion(-0.21346028f, -1.7724988E-8f, -0.16556613f, 0.9628201f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 34.36931f, 0.0f);
        q = new Quaternion(-0.20913255f, 8.994192E-9f, -0.32884872f, 0.92093545f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 43.335632f, 0.0f);
        q = new Quaternion(-0.072193384f, -6.5150716E-9f, 0.098016635f, 0.9925628f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 44.944477f, 0.0f);
        q = new Quaternion(-0.16303058f, -1.9730331E-8f, -0.0720853f, 0.9839841f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 42.296215f, 0.0f);
        q = new Quaternion(-0.009818778f, 4.4372754E-8f, -0.016939068f, 0.9998084f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 37.739285f, 0.0f);
        q = new Quaternion(0.1401084f, 7.984454E-9f, 0.12167465f, 0.9826316f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 30.30317f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        return chain;
    }

    public ArrayList<Node> generateChain1(){
        ArrayList<Node> chain = new ArrayList<Node>();
        Joint prev = null;
        Joint j;
        Quaternion q;
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 35.380642f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 33.37118f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 45.189274f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 37.873096f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 37.25165f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 43.81899f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 38.99126f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 47.659866f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 38.049095f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 34.36931f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 43.335632f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 44.944477f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 42.296215f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        j = new Joint();
        j.setReference(prev);
        j.setTranslation(0.0f, 37.739285f, 0.0f);
        q = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        j.setRotation(q);
        prev = j;
        chain.add(j);
        return chain;
    }


    public void setup() {
        //Setting the scene
        scene = new Scene(this);
        if (scene.is3D()) scene.setType(Graph.Type.ORTHOGRAPHIC);
        scene.setRadius(280);
        scene.fit(1);
        //1. Create the Skeleton (chain described above)
        skeleton = generateChain();
        //2. Lets create a Target (a bit bigger than a Joint in the structure)
        Node target = Util.createTarget(scene, jointRadius * 1.5f);
        //Locate the Target on same spot of the end effectors
        //target.setPosition(110.673035f, -344.43857f, 143.98193f);
        target.setPosition(-333.4775f, 279.97168f, -81.40487f);
        //3. Relate the structure with a Solver. In this example we instantiate a solver
        //As we're dealing with a Chain Structure a Chain Solver is preferable
        //A Chain solver constructor receives an ArrayList containing the Skeleton structure
        solver = new IKSolver(skeleton, IKSolver.HeuristicMode.CCD, true);
        //4. relate targets with end effectors
        solver.setTarget(skeleton.get(skeleton.size() - 1), target);
        solver.setMaxError(-1);
        solver.setMaxIterations(100);
        solver.setTimesPerFrame(100);
        solver.setMinDistance(-1);
        //solver.context().setSingleStep(true);
        //Define Text Properties
        textAlign(CENTER);
        textSize(24);
    }

    public void draw() {
        background(0);
        if (scene.is3D()) lights();
        scene.drawAxes();
        scene.render();
    }

    @Override
    public void mouseMoved() {
        scene.mouseTag();
    }

    public void mouseDragged() {
        if (mouseButton == LEFT) {
            scene.mouseSpin();
        } else if (mouseButton == RIGHT) {
            scene.mouseTranslate();
        } else {
            scene.scale(mouseX - pmouseX);
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

    public void keyPressed() {
        if (key == 's' || key == 'S') {
            solver.solve();
        }
    }

    public static void main(String args[]) {
        PApplet.main(new String[]{"ik.paper.SimpleTest"});
    }


}
