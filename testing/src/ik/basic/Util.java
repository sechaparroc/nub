package ik.basic;

import nub.core.Node;
import nub.core.constraint.BallAndSocket;
import nub.core.constraint.Constraint;
import nub.core.constraint.Hinge;
import nub.core.constraint.PlanarPolygon;
import nub.ik.solver.Solver;
import nub.ik.solver.geometric.CCDSolver;
import nub.ik.solver.geometric.ChainSolver;
import nub.ik.solver.geometric.oldtrik.TRIK;
import nub.ik.solver.trik.implementations.IKSolver;
import nub.ik.solver.trik.implementations.SimpleTRIK;
import nub.ik.animation.Joint;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PGraphics;
import processing.core.PShape;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static processing.core.PApplet.*;

public class Util {
  public enum ConstraintType {NONE, HINGE, CONE_POLYGON, CONE_ELLIPSE, CONE_CIRCLE, MIX, HINGE_ALIGNED, MIX_CONSTRAINED}

  public enum SolverType {
    FABRIK, FABRIK_H1, FABRIK_H2, FABRIK_H1_H2, CCD, CCD_V2, TRIK_V1, TRIK_V2, TRIK_V3, TRIK_V4,
    CCD_TRIK, CCD_TRIK_AND_TWIST, FORWARD_TRIANGULATION_TRIK, FORWARD_TRIANGULATION_TRIK_AND_TWIST,
    BACKWARD_TRIANGULATION_TRIK, BACKWARD_TRIANGULATION_TRIK_AND_TWIST, LOOK_AHEAD_FORWARD, LOOK_AHEAD_FORWARD_AND_TWIST,
    CCDT_BACK_AND_FORTH, CCD_BACK_AND_FORTH, BACK_AND_FORTH_TRIK_T, FINAL_TRIK, EXPRESSIVE_FINAL_TRIK,
    //Latest ones
    TRIANGULATION_HEURISTIC, BACK_AND_FORTH_TRIANGULATION_HEURISTIC,
    CCD_HEURISTIC, BACK_AND_FORTH_CCD_HEURISTIC,
    TRIK_HEURISTIC, BACK_AND_FORTH_TRIK_HEURISTIC,
    COMBINED_HEURISTIC, COMBINED_EXPRESSIVE
  }

  public static Solver createSolver(SolverType type, List<Node> structure) {
    switch (type) {
      case CCD:
        return new CCDSolver(structure);
      case CCD_V2: {
        CCDSolver solver = new CCDSolver(structure, false);
        return solver;
      }
      case FABRIK: {
        ChainSolver solver = new ChainSolver(structure);
        solver.setKeepDirection(false);
        solver.setFixTwisting(false);
        solver.explore(false);
        return solver;
      }
      case FABRIK_H1: {
        ChainSolver solver = new ChainSolver(structure);
        solver.setKeepDirection(true);
        solver.setFixTwisting(false);
        return solver;
      }
      case FABRIK_H2: {
        ChainSolver solver = new ChainSolver(structure);
        solver.setKeepDirection(false);
        solver.setFixTwisting(true);
        return solver;
      }
      case FABRIK_H1_H2: {
        ChainSolver solver = new ChainSolver(structure);
        solver.setKeepDirection(true);
        solver.setFixTwisting(true);
        return solver;
      }
      case TRIK_V1: {
        TRIK solver = new TRIK(structure);
        return solver;
      }
      case TRIK_V2: {
        TRIK solver = new TRIK(structure);
        solver.enableWeight(true);
        solver.enableDirection(true);
        return solver;
      }
      case TRIK_V3: {
        TRIK solver = new TRIK(structure);
        solver.setLookAhead(5);
        solver.enableWeight(true);
        //solver.enableDirection(true);
        return solver;
      }
      case TRIK_V4: {
        TRIK solver = new TRIK(structure);
        solver.setLookAhead(5);
        solver.enableWeight(true);
        solver.smooth(true);
        solver.enableDirection(true);
        return solver;
      }

      case CCD_TRIK: {
        SimpleTRIK solver = new SimpleTRIK(structure, SimpleTRIK.HeuristicMode.CCD);
        solver.enableTwist(false);
        return solver;
      }

      case CCD_TRIK_AND_TWIST: {
        SimpleTRIK solver = new SimpleTRIK(structure, SimpleTRIK.HeuristicMode.CCD);
        return solver;
      }

      case FORWARD_TRIANGULATION_TRIK: {
        SimpleTRIK solver = new SimpleTRIK(structure, SimpleTRIK.HeuristicMode.FORWARD_TRIANGULATION);
        //solver.enableTwist(false);
        return solver;
      }

      case FORWARD_TRIANGULATION_TRIK_AND_TWIST: {
        SimpleTRIK solver = new SimpleTRIK(structure, SimpleTRIK.HeuristicMode.FORWARD_TRIANGULATION);
        return solver;
      }

      case BACKWARD_TRIANGULATION_TRIK: {
        SimpleTRIK solver = new SimpleTRIK(structure, SimpleTRIK.HeuristicMode.BACKWARD_TRIANGULATION);
        //solver.enableTwist(false);
        return solver;
      }

      case BACKWARD_TRIANGULATION_TRIK_AND_TWIST: {
        SimpleTRIK solver = new SimpleTRIK(structure, SimpleTRIK.HeuristicMode.BACKWARD_TRIANGULATION);
        return solver;
      }

      case LOOK_AHEAD_FORWARD: {
        SimpleTRIK solver = new SimpleTRIK(structure, SimpleTRIK.HeuristicMode.LOOK_AHEAD_FORWARD);
        //solver.enableTwist(false);
        return solver;
      }

      case LOOK_AHEAD_FORWARD_AND_TWIST: {
        SimpleTRIK solver = new SimpleTRIK(structure, SimpleTRIK.HeuristicMode.LOOK_AHEAD_FORWARD);
        return solver;
      }

      case FINAL_TRIK: {
        SimpleTRIK solver = new SimpleTRIK(structure, SimpleTRIK.HeuristicMode.COMBINED);
        return solver;
      }

      case EXPRESSIVE_FINAL_TRIK: {
        SimpleTRIK solver = new SimpleTRIK(structure, SimpleTRIK.HeuristicMode.COMBINED_EXPRESSIVE);
        //solver.enableSmooth(true);
        return solver;
      }

      case CCD_HEURISTIC:{
        IKSolver solver = new IKSolver(structure, IKSolver.HeuristicMode.CCD);
        return solver;
      }

      case BACK_AND_FORTH_CCD_HEURISTIC:{
        IKSolver solver = new IKSolver(structure, IKSolver.HeuristicMode.BACK_AND_FORTH_CCD);
        return solver;
      }

      case TRIK_HEURISTIC:{
        IKSolver solver = new IKSolver(structure, IKSolver.HeuristicMode.TRIK);
        return solver;
      }

      case BACK_AND_FORTH_TRIK_HEURISTIC:{
        IKSolver solver = new IKSolver(structure, IKSolver.HeuristicMode.BACK_AND_FORTH_TRIK);
        return solver;
      }

      case TRIANGULATION_HEURISTIC:{
        IKSolver solver = new IKSolver(structure, IKSolver.HeuristicMode.TRIANGULATION);
        return solver;
      }

      case BACK_AND_FORTH_TRIANGULATION_HEURISTIC:{
        IKSolver solver = new IKSolver(structure, IKSolver.HeuristicMode.BACK_AND_FORTH_TRIANGULATION);
        return solver;
      }

      case COMBINED_EXPRESSIVE:{
        IKSolver solver = new IKSolver(structure, IKSolver.HeuristicMode.COMBINED_EXPRESSIVE);
        return solver;
      }

      case COMBINED_HEURISTIC:{
        IKSolver solver = new IKSolver(structure, IKSolver.HeuristicMode.COMBINED);
        return solver;
      }
      default:
        return null;
    }
  }

  public static Node createTarget(Scene scene, float targetRadius) {
    PShape redBall;
    if (scene.is3D())
      redBall = scene.context().createShape(SPHERE, targetRadius);
    else
      redBall = scene.context().createShape(ELLIPSE, 0, 0, targetRadius, targetRadius);

    redBall.setFill(scene.context().color(0, 255, 0));
    redBall.setStroke(false);
    return createTarget(scene, redBall, targetRadius);
  }

  public static Node createTarget(Scene scene, PShape shape, float targetRadius) {
    Node node =  new Node() {
      @Override
      public void graphics(PGraphics pg) {
        Scene.drawAxes(pg, targetRadius * 2);
        if (scene.node() == this) {
          shape.setFill(pg.color(0, 255, 0));
        } else {
          shape.setFill(pg.color(255, 0, 0));
        }
        pg.noStroke();
        pg.shape(shape);
      }
    };
    node.setPickingThreshold(0);
    return node;
  }

  public static ArrayList<Node> createTargets(int num, Scene scene, float targetRadius) {
    PGraphics pg = scene.context();
    ArrayList<Node> targets = new ArrayList<Node>();
    PShape redBall;
    if (scene.is3D())
      redBall = pg.createShape(SPHERE, targetRadius);
    else
      redBall = pg.createShape(ELLIPSE, 0, 0, targetRadius, targetRadius);
    redBall.setStroke(false);
    redBall.setFill(pg.color(255, 0, 0));

    for (int i = 0; i < num; i++) {
      Node target = createTarget(scene, redBall, targetRadius);
      target.setPickingThreshold(targetRadius * 2);
      targets.add(target);
    }
    return targets;
  }

  public static ArrayList<Node> generateAttachedChain(int numJoints, float radius, float boneLength, Vector translation, int red, int green, int blue) {
    return generateAttachedChain(numJoints, radius, boneLength, translation, red, green, blue, -1, 0);
  }

  public static ArrayList<Node> generateAttachedChain(int numJoints, float radius, float boneLength, Vector translation, int red, int green, int blue, int randRotation, int randLength) {
    Random r1 = randRotation != -1 ? new Random(randRotation) : null;
    Random r2 = randLength != -1 ? new Random(randLength) : null;

    Joint prevJoint = null;
    Joint chainRoot = null;
    for (int i = 0; i < numJoints; i++) {
      Joint joint;
      joint = new Joint(red, green, blue, radius);
      if (i == 0)
        chainRoot = joint;
      if (prevJoint != null) joint.setReference(prevJoint);

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
      prevJoint = joint;
    }
    //Consider Standard Form: Parent Z Axis is Pointing at its Child
    chainRoot.setTranslation(translation);
    //chainRoot.setupHierarchy();
    chainRoot.setRoot(true);
    return (ArrayList) Scene.branch(chainRoot);
  }

  public static List<Node> generateAttachedChain(int numJoints, float boneLength, int randRotation, int randLength) {
    List<Node> chain = new ArrayList<Node>();
    Random r1 = randRotation != -1 ? new Random(randRotation) : null;
    Random r2 = randLength != -1 ? new Random(randLength) : null;
    Node prevJoint = null;
    for (int i = 0; i < numJoints; i++) {
      Node joint = new Node();
      if (i == 0)
        if (prevJoint != null) joint.setReference(prevJoint);
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
      prevJoint = joint;
      chain.add(joint);
    }
    return chain;
  }


  public static List<Node> generateDetachedChain(int numJoints, float boneLength, int randRotation, int randLength) {
    List<Node> chain = new ArrayList<Node>();
    Random r1 = randRotation != -1 ? new Random(randRotation) : null;
    Random r2 = randLength != -1 ? new Random(randLength) : null;
    Node prevJoint = null;
    for (int i = 0; i < numJoints; i++) {
      Node joint = Node.detach(new Vector(), new Quaternion(), 1f);
      if (prevJoint != null) joint.setReference(prevJoint);
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
      prevJoint = joint;
      chain.add(joint);
    }
    return chain;
  }


  public static void generateConstraints(List<? extends Node> structure, ConstraintType type, int seed, boolean is3D) {
    Random random = new Random(seed);
    int numJoints = structure.size();
    for (int i = 0; i < numJoints - 1; i++) {
      Vector twist = structure.get(i + 1).translation().get();
      //Quaternion offset = new Quaternion(new Vector(0, 1, 0), radians(random(-90, 90)));
      Quaternion offset = new Quaternion();//Quaternion.random();
      Constraint constraint = null;
      ConstraintType current = type;
      if (type == ConstraintType.MIX) {
        int r = random.nextInt(ConstraintType.values().length - 1) + 1;
        r = is3D ? r : r % 2;
        current = ConstraintType.values()[r];
        if (current == ConstraintType.CONE_POLYGON) current = ConstraintType.CONE_ELLIPSE;
      } else if (type == ConstraintType.MIX_CONSTRAINED) {
        int r = random.nextInt(100);
        r = is3D ? r : 0;
        if (r % 2 == 0) current = ConstraintType.CONE_ELLIPSE;
        else current = ConstraintType.HINGE;
      }

      switch (current) {
        case NONE: {
          break;
        }
        case CONE_ELLIPSE: {
          if (!is3D) break;
          float down = radians((float) Math.min(Math.max(random.nextGaussian() * 30 + 30, 0), 80));
          float up = radians((float) Math.min(Math.max(random.nextGaussian() * 30 + 30, 0), 80));
          float left = radians((float) Math.min(Math.max(random.nextGaussian() * 30 + 30, 0), 80));
          float right = radians((float) Math.min(Math.max(random.nextGaussian() * 30 + 30, 0), 80));

          //down = left = right = up = radians(40);
          constraint = new BallAndSocket(down, up, left, right);
          Quaternion rest = Quaternion.compose(structure.get(i).rotation().get(), offset);
          ((BallAndSocket) constraint).setRestRotation(rest, new Vector(0, 1, 0), twist);
          break;
        }
        case CONE_CIRCLE: {
          if (!is3D) break;
          float r = radians((float) Math.min(Math.max(random.nextGaussian() * 30 + 30, 0), 80));
          //r = radians(40);
          constraint = new BallAndSocket(r, r, r, r);
          Quaternion rest = Quaternion.compose(structure.get(i).rotation().get(), offset);
          ((BallAndSocket) constraint).setRestRotation(rest, new Vector(0, 1, 0), twist);
          break;
        }

        case CONE_POLYGON: {
          if (!is3D) break;
          ArrayList<Vector> vertices = new ArrayList<Vector>();
          float v = 20;
          float w = 20;
          vertices.add(new Vector(-w, -v));
          vertices.add(new Vector(w, -v));
          vertices.add(new Vector(w, v));
          vertices.add(new Vector(-w, v));
          constraint = new PlanarPolygon(vertices);
          Quaternion rest = Quaternion.compose(structure.get(i).rotation().get(), offset);
          ((PlanarPolygon) constraint).setRestRotation(rest, new Vector(0, 1, 0), twist);
          ((PlanarPolygon) constraint).setAngle(radians(random.nextFloat() * 20 + 30));
          break;
        }
        case HINGE: {
          Vector vector = new Vector(0,0,1);
          if(is3D) {
            vector = new Vector(2 * random.nextFloat() - 1, 2 * random.nextFloat() - 1, 2 * random.nextFloat() - 1);
            vector.normalize();
            vector = Vector.projectVectorOnPlane(vector, structure.get(i + 1).translation());
          }
          float min = (float) Math.min(Math.max(random.nextGaussian() * 30 + 30, 10), 120);
          float max = (float) Math.min(Math.max(random.nextGaussian() * 30 + 30, 10), 120);

          constraint = new Hinge(radians(min),
              radians(max),
              structure.get(i).rotation().get(), new Vector(0, 1, 0), vector);
          break;
        }
        case HINGE_ALIGNED: {
          float min = (float) Math.min(Math.max(random.nextGaussian() * 30 + 30, 10), 120);
          float max = (float) Math.min(Math.max(random.nextGaussian() * 30 + 30, 10), 120);
          Vector vector = new Vector(0, 0, 1);

          constraint = new Hinge(radians(min),
              radians(max),
              structure.get(i).rotation().get(), new Vector(0, 1, 0), vector);
        }
        break;
      }
      structure.get(i).setConstraint(constraint);
    }
  }

  public static void printInfo(Scene scene, Solver solver, Vector basePosition) {
    PGraphics pg = scene.context();
    pg.pushStyle();
    pg.fill(255);
    pg.textSize(15);
    Vector pos = scene.screenLocation(basePosition);
    if (solver instanceof ChainSolver) {
      ChainSolver s = (ChainSolver) solver;
      String heuristics = "";
      if (s.keepDirection()) heuristics += "\n Keep directions";
      if (s.fixTwisting()) heuristics += "\n Fix Twisting";
      heuristics += "\nAccum error : " + solver.accumulatedError();
      pg.text("FABRIK" + heuristics + "\n Error: " + String.format("%.7f", solver.error()) + "\n Exploration : " + s.explorationTimes() + "\n iter : " + solver.lastIteration(), pos.x() - 30, pos.y() + 10, pos.x() + 30, pos.y() + 50);
    }
    if (solver instanceof CCDSolver) {
      pg.text("CCD" + "\n Error: " + String.format("%.7f", solver.error()) + "\nAccum error : " + solver.accumulatedError() + "\n iter : " + solver.lastIteration(), pos.x() - 30, pos.y() + 10, pos.x() + 30, pos.y() + 50);
    }
    if (solver instanceof TRIK) {
      TRIK trik = (TRIK) solver;
      String error = "\n Error (pos): " + String.format("%.7f", trik.positionError());
      if (trik.direction()) {
        error += "\n Error (or): " + String.format("%.7f", trik.orientationError());
      }
      error += "\n Avg actions : " + String.format("%.7f", trik._average_actions);
      error += "\nAccum error : " + solver.accumulatedError();
      pg.text("TRIK" + error + "\n iter : " + solver.lastIteration(), pos.x() - 30, pos.y() + 10, pos.x() + 30, pos.y() + 50);
    }

    if (solver instanceof SimpleTRIK) {
      SimpleTRIK trik = (SimpleTRIK) solver;

      String heuristics = String.join(" ", trik.mode().name().split("_"));
      if (trik.enableTwist()) heuristics += "\nWITH TWIST";
      String error = "\n Error (pos): " + String.format("%.7f", trik.positionError());
      if (trik.direction()) {
        error += "\n Error (or): " + String.format("%.7f", trik.orientationError());
      }
      error += "\nAccum error : " + solver.accumulatedError();
      pg.text(heuristics + error + "\n iter : " + solver.lastIteration(), pos.x() - 30, pos.y() + 10, pos.x() + 30, pos.y() + 50);
    }

    if (solver instanceof IKSolver) {
      IKSolver ikSolver = (IKSolver) solver;

      String heuristics = String.join(" ", ikSolver.mode().name().split("_"));
      if (ikSolver.enableTwist()) heuristics += "\nWITH TWIST";
      String error = "\n Error (pos): " + String.format("%.7f", ikSolver.positionError());
      if (ikSolver.direction()) {
        error += "\n Error (or): " + String.format("%.7f", ikSolver.orientationError());
      }
      error += "\nAccum error : " + solver.accumulatedError();
      pg.text(heuristics + error + "\n iter : " + solver.lastIteration(), pos.x() - 30, pos.y() + 10, pos.x() + 30, pos.y() + 50);
    }

    pg.popStyle();
  }

  public static ArrayList<Node> detachedCopy(List<? extends Node> chain) {
    ArrayList<Node> copy = new ArrayList<Node>();
    Node reference = chain.get(0).reference();
    if (reference != null) {
      reference = Node.detach(reference.position().get(), reference.orientation().get(), 1);
    }
    for (Node joint : chain) {
      Node newJoint = Node.detach(new Vector(), new Quaternion(), 1f);
      newJoint.setReference(reference);
      newJoint.setPosition(joint.position().get());
      newJoint.setOrientation(joint.orientation().get());
      newJoint.setConstraint(joint.constraint());
      copy.add(newJoint);
      reference = newJoint;
    }
    return copy;
  }

  public static void drawPositions(PGraphics pg, ArrayList<Vector> positions, int color, float str) {
    pg.sphereDetail(5);
    if (positions == null) return;
    Vector prev = null;
    for (Vector p : positions) {
      pg.pushMatrix();
      pg.pushStyle();
      pg.stroke(color);
      pg.strokeWeight(str);
      if (prev != null) pg.line(prev.x(), prev.y(), prev.z(), p.x(), p.y(), p.z());
      pg.noStroke();
      pg.fill(color);
      pg.translate(p.x(), p.y(), p.z());
      pg.sphere(3);
      pg.popStyle();
      pg.popMatrix();
      prev = p;
    }
    pg.sphereDetail(40);
  }
}
