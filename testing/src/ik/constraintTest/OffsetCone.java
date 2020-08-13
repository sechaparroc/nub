package ik.constraintTest;

import ik.basic.Util;
import nub.core.Graph;
import nub.core.Node;
import nub.core.constraint.BallAndSocket;
import nub.core.constraint.FixedConstraint;
import nub.ik.solver.Solver;
import nub.ik.solver.geometric.ChainSolver;
import nub.ik.solver.trik.implementations.IKSolver;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.processing.TimingTask;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.util.ArrayList;

/**
 * Created by sebchaparr on 15/02/19.
 */

public class OffsetCone extends PApplet {

  int numJoints = 4;
  float targetRadius = 7;
  float boneLength = 50;

  int rows = 2;

  //Scene Parameters
  Scene scene;
  //Benchmark Parameters
  ArrayList<Solver> solvers; //Will store Solvers
  ArrayList<ArrayList<Node>> structures = new ArrayList<>(); //Keep Structures
  ArrayList<Node> targets = new ArrayList<Node>(); //Keep targets

  int numSolvers = 6; //Set number of solvers
  boolean solve = false;

  public void settings() {
    size(700, 700, P3D);
  }

  public void setup() {
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.setBounds(numJoints * boneLength * 2f);
    scene.fit(1);
    Scene.leftHanded = false;

    //1. Create Targets
    targets = Util.createTargets(numSolvers, scene, targetRadius);

    //2. Generate Structures
    int solversPerRow = (int) Math.ceil(1.f * numSolvers / rows);

    for (int i = 0; i < numSolvers; i++) {
      int row = i / solversPerRow;
      int col = i % solversPerRow;
      int cols = row == rows - 1 ? solversPerRow - (rows * solversPerRow - numSolvers) : solversPerRow;
      float xOffset = ((1.f * col) / (cols - 1)) * scene.radius() - scene.radius() / 2;
      float yOffset = -((1.f * row) / (rows - 1)) * scene.radius() + scene.radius() / 2;
      int red = (int) random(255), green = (int) random(255), blue = (int) random(255);

      structures.add(createLimb(new Vector(xOffset, yOffset, 0), red, green, blue));
    }

    //5. generate solvers
    solvers = new ArrayList<>();

    int i = 0;
    //CCD
    solvers.add(new IKSolver(structures.get(i++), IKSolver.HeuristicMode.CCD));
    //Standard FABRIK
    ChainSolver chainSolver;
    chainSolver = new ChainSolver(structures.get(i++));
    chainSolver.setKeepDirection(false);
    chainSolver.setFixTwisting(false);
    solvers.add(chainSolver);
    //FABRIK Keeping directions (H1)
    chainSolver = new ChainSolver(structures.get(i++));
    chainSolver.setFixTwisting(true);
    chainSolver.setKeepDirection(false);
    solvers.add(chainSolver);
    //FABRIK Fix Twisting (H2)
    chainSolver = new ChainSolver(structures.get(i++));
    chainSolver.setFixTwisting(false);
    chainSolver.setKeepDirection(true);
    solvers.add(chainSolver);
    //FABRIK Fix Twisting (H1 & H2)
    chainSolver = new ChainSolver(structures.get(i++));
    chainSolver.setFixTwisting(true);
    chainSolver.setKeepDirection(true);
    solvers.add(chainSolver);

    for (i = 0; i < solvers.size(); i++) {
      Solver solver = solvers.get(i);
      //6. Define solver parameters
      solver.setMaxError(0.001f);
      solver.setTimesPerFrame(5);
      solver.setMaxIterations(200);
      //7. Set targets
      solver.setTarget(structures.get(i).get(numJoints - 1), targets.get(i));
      targets.get(i).setPosition(structures.get(i).get(numJoints - 1).position());

      TimingTask task = new TimingTask() {
        @Override
        public void execute() {
          if (solve) {
            solver.solve();
          }
        }
      };
      task.run(40);
    }
  }

  public void draw() {
    background(0);
    lights();
    scene.drawAxes();
    scene.render();

    scene.beginHUD();
    for (int i = 0; i < solvers.size(); i++) {
      Util.printInfo(scene, solvers.get(i), structures.get(i).get(0).position());
    }
    scene.endHUD();

  }

  public ArrayList<Node> createLimb(Vector position, int red, int green, int blue) {
    ArrayList<Node> limb = new ArrayList<>();

    //Create a simple limb
    Node j1 = new Node();
    j1.enableHint(Node.CONSTRAINT);
    j1.setShape(pg -> {
          pg.pushStyle();
          pg.noStroke();
          pg.fill(red, green, blue);
          if(pg.is3D()) pg.sphere(targetRadius * 0.7f);
          else pg.ellipse(0,0, 2 * targetRadius * 0.7f, 2 * targetRadius * 0.7f);
          pg.popStyle();
    });
    j1.setTranslation(0, 90, 0);
    Node j2 = new Node();
    j2.enableHint(Node.CONSTRAINT);
    j2.enableHint(Node.BONE, color(red, green, blue), targetRadius * 0.7f);
    j2.setReference(j1);
    j2.setTranslation(50, 30, 0);
    Node j3 = new Node();
    j3.enableHint(Node.CONSTRAINT);
    j3.enableHint(Node.BONE, color(red, green, blue), targetRadius * 0.7f);
    j3.setReference(j2);
    j3.setTranslation(50, -80, 0);
    Node j4 = new Node();
    j4.enableHint(Node.CONSTRAINT);
    j4.enableHint(Node.BONE, color(red, green, blue), targetRadius * 0.7f);
    j4.setReference(j3);
    j4.setTranslation(0, -80, 0);

    j1.setPosition(position);

    j1.setConstraint(new FixedConstraint());
    BallAndSocket c2 = new BallAndSocket(radians(85), radians(85));
    c2.setRestRotation(j2.rotation().get(), new Vector(0, 1, 0), new Vector(1, 0, 0), j3.translation());
    j2.setConstraint(c2);

    BallAndSocket c3 = new BallAndSocket(radians(10), radians(40), radians(40), radians(40));
    c3.setRestRotation(j3.rotation().get(), new Vector(0, 0, 1), new Vector(0, -1, 0));
    j3.setConstraint(c3);

    limb.add(j1);
    limb.add(j2);
    limb.add(j3);
    limb.add(j4);

    return limb;
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

  public void keyPressed() {
    if (key == 'w' || key == 'W') {
      solve = !solve;
    }
  }


  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.constraintTest.OffsetCone"});
  }
}

