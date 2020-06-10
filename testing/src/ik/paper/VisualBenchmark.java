package ik.paper;

import ik.basic.Util;
import nub.core.Graph;
import nub.core.Node;
import nub.ik.animation.Joint;
import nub.ik.solver.Solver;
import nub.ik.solver.geometric.FABRIKSolver;
import nub.ik.solver.geometric.oldtrik.TRIK;
import nub.ik.solver.trik.heuristic.Combined;
import nub.ik.solver.trik.implementations.IKSolver;
import nub.ik.solver.trik.implementations.SimpleTRIK;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.processing.TimingTask;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class VisualBenchmark extends PApplet {
    //Scene Parameters
    Scene scene;
    String renderer = P3D; //Define a 2D/3D renderer
    int numJoints = 12; //Define the number of joints that each chain will contain
    float targetRadius = 30; //Define size of target
    float boneLength = 50; //Define length of segments (bones)

    //Benchmark Parameters
    Util.ConstraintType constraintType = Util.ConstraintType.CONE_ELLIPSE; //Choose what kind of constraints apply to chain
    Random random = new Random();
    ArrayList<Solver> solvers; //Will store Solvers
    int randRotation = -1; //Set seed to generate initial random rotations, otherwise set to -1
    int randLength = 0; //Set seed to generate random segment lengths, otherwise set to -1


    Util.SolverType solversType[] = {
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

    ArrayList<ArrayList<Node>> structures = new ArrayList<>(); //Keep Structures
    ArrayList<Node> targets = new ArrayList<Node>(); //Keep targets

    boolean solve = false;
    boolean show1 = false, show2 = false, show3 = false, show4 = false, show5 = false;

    public void settings() {
        size(1500, 800, renderer);
    }

    public void setup() {
        Joint.axes = true;
        scene = new Scene(this);
        if (scene.is3D()) scene.setType(Graph.Type.ORTHOGRAPHIC);
        scene.setRadius(numJoints * 1f * boneLength);
        scene.fit(1);
        scene.setRightHanded();

        int numSolvers = solversType.length;
        //1. Create Targets
        targets = Util.createTargets(numSolvers, scene, targetRadius);

        float alpha = 1.f * width / height > 1.5f ? 0.5f * width / height : 0.5f;
        alpha *= numSolvers / 4f; //avoid undesirable overlapping

        //2. Generate Structures
        for (int i = 0; i < numSolvers; i++) {
            int red = (int) random(255);
            int green = (int) random(255);
            int blue = (int) random(255);
            structures.add(Util.generateAttachedChain(numJoints, 0.3f * targetRadius, boneLength, new Vector(i * 2 * alpha * scene.radius() / (numSolvers - 1) - alpha * scene.radius(), 0, 0), red, green, blue, randRotation, randLength));
        }

        //3. Apply constraints
        for (ArrayList<Node> structure : structures) {
            Util.generateConstraints(structure, constraintType, 0, scene.is3D());
        }

        //4. Set eye scene
        scene.eye().rotate(new Quaternion(new Vector(1, 0, 0), PI / 2.f));
        scene.eye().rotate(new Quaternion(new Vector(0, 1, 0), PI));

        //5. generate solvers
        solvers = new ArrayList<>();
        for (int i = 0; i < numSolvers; i++) {
            Solver solver = Util.createSolver(solversType[i], structures.get(i));
            solvers.add(solver);
            //6. Define solver parameters
            //solvers.get(i).setMaxError(0.001f);
            solvers.get(i).setTimesPerFrame(1);
            solvers.get(i).setMaxIterations(200);
            //solvers.get(i).setMinDistance(0.001f);
            //7. Set targets
            solvers.get(i).setTarget(structures.get(i).get(numJoints - 1), targets.get(i));
            targets.get(i).setPosition(structures.get(i).get(numJoints - 1).position());
            //8. Register task
            TimingTask task = new TimingTask() {
                @Override
                public void execute() {
                    if (solve) solver.solve();
                }
            };
            task.run(40);
        }
    }

    public void draw() {
        background(0);
        if (scene.is3D()) lights();
        //Draw Constraints
        scene.drawAxes();


        for (Solver s : solvers) {
            if (s instanceof SimpleTRIK) {
                if (((SimpleTRIK) s).mainHeuristic() instanceof Combined) {
                    Combined hig = (Combined) ((SimpleTRIK) s).mainHeuristic();
                    hig.drawVectors(scene);
                    //hig.drawPositionContourMap(scene);
                }
            }
        }

        scene.render();
        scene.beginHUD();
        for (int i = 0; i < solvers.size(); i++) {
            Util.printInfo(scene, solvers.get(i), structures.get(i).get(0).position());
        }
        scene.endHUD();
    }

    public Node generateRandomReachablePosition(List<? extends Node> chain, boolean is3D) {
        for (int i = 0; i < chain.size() - 1; i++) {
            if (is3D) {
                chain.get(i).rotate(new Quaternion(new Vector(0, 0, 1), random.nextFloat() * 2 * PI));
                chain.get(i).rotate(new Quaternion(new Vector(0, 1, 0), random.nextFloat() * 2 * PI));
                chain.get(i).rotate(new Quaternion(new Vector(1, 0, 0), random.nextFloat() * 2 * PI));
            } else
                chain.get(i).rotate(new Quaternion(new Vector(0, 0, 1), (float) (random.nextFloat() * PI)));
        }
        return chain.get(chain.size() - 1);
    }

    public void keyPressed() {
        if (key == 'w' || key == 'W') {
            solve = !solve;
        }
        if (key == 's' || key == 'S') {
            Node f = generateRandomReachablePosition(structures.get(0), scene.is3D());
            Vector delta = Vector.subtract(f.position(), targets.get(0).position());
            for (Node target : targets) {
                target.setPosition(Vector.add(target.position(), delta));
                target.setOrientation(f.orientation());
            }
        }
        if (key == 'd' || key == 'D') {
            for (List<Node> structure : structures) {
                for (Node f : structure) {
                    f.setRotation(new Quaternion());
                }
            }
        }

        if (key == 'm' || key == 'M') {
            for (Solver s : solvers) {
                if (s instanceof SimpleTRIK)
                    ((SimpleTRIK) s).context().setSingleStep(!((SimpleTRIK) s).context().singleStep());
                if (s instanceof IKSolver)
                    ((IKSolver) s).context().setSingleStep(!((IKSolver) s).context().singleStep());
            }
        }
        // /* Uncomment this to debug a Specific Solver
        if (key == 'z' || key == 'Z') {
            for (Solver s : solvers) s.solve();
        }
        // /*

        if (key == '1') {
            TRIK._debug = !TRIK._debug;
            show1 = !show1;
        }
        if (key == '2') {
            show2 = !show2;
        }
        if (key == '3') {
            show3 = !show3;
        }
        if (key == '4') {
            show4 = !show4;
        }
        if (key == '5') {
            show5 = !show5;
        }
        if (key == '6') {
            FABRIKSolver.rand = !FABRIKSolver.rand;
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
            scene.scale(scene.mouseDX());
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

    public static void main(String args[]) {
        PApplet.main(new String[]{"ik.paper.VisualBenchmark"});
    }
}
