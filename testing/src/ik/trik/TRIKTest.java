package ik.trik;

import ik.basic.Util;
import nub.core.Graph;
import nub.core.Node;
import nub.ik.solver.Solver;
import nub.ik.solver.GHIK;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.processing.TimingTask;
import processing.core.PApplet;
import processing.core.PGraphics;
import processing.core.PShape;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class TRIKTest extends PApplet {
    Scene scene;
    //Set the scene as P3D or P2D
    String renderer = P2D;
    float jointRadius = 5;
    float length = 50;
    int numJoints = 8;
    boolean enableSolver = false;
    Solver trik;
    List<Node> skeleton, idle;
    List<Node> targets = new ArrayList<Node>();
    Random random = new Random(0);

    public void settings() {
        size(700, 700, renderer);
    }

    public void setup() {
        scene = new Scene(this);
        if (scene.is3D()) scene.setType(Graph.Type.ORTHOGRAPHIC);
        scene.setBounds(280);
        scene.fit(1);
        int red = (int) random(255);
        int green = (int) random(255);
        int blue = (int) random(255);

        skeleton = Util.generateAttachedChain(numJoints, jointRadius, length, new Vector(), red, green, blue, -1, 0);
        idle = Util.generateDetachedChain(numJoints, length,-1, 0);

        Util.generateConstraints(skeleton, Util.ConstraintType.HINGE, -1, scene.is3D());
        Util.generateConstraints(idle, Util.ConstraintType.HINGE, -1, scene.is3D());
        //Util.generateConstraints(skeleton2, Util.ConstraintType.CONE_ELLIPSE, -1, scene.is3D());

        //Create solver
        trik = createSolver(skeleton);
        //Define Text Properties
        textAlign(CENTER);
        textSize(24);

        scene.enableHint(Graph.BACKGROUND | Graph.AXES);
    }

    public void draw() {
        if (scene.is3D()) lights();
        scene.render();
        scene.beginHUD();
        drawInfo(skeleton);
        //Util.printInfo(scene, trik, skeleton2.get(0).position());
        scene.endHUD();
    }

    public Solver createSolver(List<Node> skeleton) {
        Solver solver;
        Node endEffector = skeleton.get(skeleton.size() - 1);
        //2. Lets create a Target (a bit bigger than a Joint in the structure)
        Node target = createTarget(scene, jointRadius * 1.5f);
        //Locate the Target on same spot of the end effectors
        target.setPosition(endEffector.position());
        targets.add(target);
        //3. Relate the structure with a Solver. In this example we instantiate a solver
        solver = new GHIK(skeleton, GHIK.HeuristicMode.TRIK);
        //Optionally you could modify the following parameters of the Solver:
        //Maximum distance between end effector and target, If is below maxError, then we stop executing IK solver (Default value is 0.01)
        //solver.setMaxError(1);
        //Number of iterations to perform in order to reach the target (Default value is 50)
        solver.setMaxIterations(8);
        //Times a solver will iterate on a single Frame (Default value is 5)
        solver.setTimesPerFrame(1);
        //Minimum distance between previous and current solution to consider that Solver converges (Default value is 0.01)
        //solver.setMinDistance(0.5f);
        //4. relate targets with end effectors
        solver.setTarget(endEffector, target);
        //5. Create a Timing Task such that the solver executes each amount of time
        TimingTask solverTask = new TimingTask() {
            @Override
            public void execute() {
                //a solver perform an iteration when solve method is called
                if (enableSolver) {
                    solver.solve();
                }
            }
        };
        solverTask.run(40); //Execute the solverTask each 40 ms
        return solver;
    }

    public Node createTarget(Scene scene, float radius) {
        /*
         * A target is a Node, we represent a Target as a
         * Red ball.
         * */
        PShape redBall;
        if (scene.is2D()) redBall = createShape(ELLIPSE, 0, 0, radius * 2, radius * 2);
        else redBall = createShape(SPHERE, radius);
        redBall.setStroke(false);
        redBall.setFill(color(255, 0, 0));

        Node target = new Node() {
            @Override
            public void graphics(PGraphics pGraphics) {
                Scene.drawAxes(pGraphics, radius * 1.5f);
                pGraphics.shape(redBall);
            }
        };
        //Exact picking precision
        target.setBullsEyeSize(0);
        return target;
    }


    public void drawInfo(List<Node> skeleton) {
        for (int i = 0; i < skeleton.size(); i++) {
            //Print Node names
            Vector screenLocation = scene.screenLocation(skeleton.get(i).position());
            text("Node " + i, screenLocation.x(), screenLocation.y());
        }
    }

    @Override
    public void mouseMoved() {
        scene.mouseTag();
    }

    public void mouseDragged() {
        if (mouseButton == LEFT) {
            scene.mouseSpin();
        } else if (mouseButton == RIGHT) {
            if (targets.contains(scene.node())) {
                for (Node target : targets) scene.translateNode(target, scene.mouseDX(), scene.mouseDY(), 0, 0);
            } else {
                scene.mouseTranslate();
            }

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
        if (key == 'A' || key == 'a') {
            enableSolver = false;
            trik.solve();
        }

        if (key == 'S' || key == 's') {
            enableSolver = !enableSolver;
        }

        if (key == 'R' || key == 'r') {
            generateRandomReachablePosition(idle, scene.is3D());
            Node f = generateRandomReachablePosition(idle, scene.is3D());
            Vector delta = Vector.subtract(f.position(), targets.get(0).position());
            for (Node target : targets) {
                target.setPosition(Vector.add(target.position(), delta));
                target.setOrientation(f.orientation());
            }
        }
    }

    public Node generateRandomReachablePosition(List<? extends Node> chain, boolean is3D) {
        for (int i = 0; i < chain.size() - 1; i++) {
            if (is3D)
                chain.get(i).rotate(new Quaternion(Vector.random(), (float) (random.nextFloat() * 2 * PI)));
            else
                chain.get(i).rotate(new Quaternion(new Vector(0, 0, 1), (float) (random.nextFloat() * 2 * PI)));
        }
        return chain.get(chain.size() - 1);
    }


    public static void main(String args[]) {
        PApplet.main(new String[]{"ik.trik.TRIKTest"});
    }
}

