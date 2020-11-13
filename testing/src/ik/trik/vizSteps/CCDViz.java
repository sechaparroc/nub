package ik.trik.vizSteps;

import nub.core.Node;
import nub.core.constraint.Hinge;
import nub.ik.solver.Context;
import nub.ik.solver.NodeInformation;
import nub.ik.solver.heuristic.Heuristic;
import nub.ik.solver.heuristic.Util;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;


public class CCDViz extends Heuristic {

  public Viz viz;

  public CCDViz(Context context, Viz viz) {
    super(context);
    this.viz = viz;
    this.viz.setHeuristic(this);
  }

  @Override
  public void prepare() {
    //Update cache of usable chain
    NodeInformation._updateCache(_context.usableChainInformation());
    if (viz != null){
      viz.clearFigures();
      viz.addFrame("dummy"); //Dummy frame with no information :|
      viz.sequence.clear();
      viz.addHighlightNode("P\u2080", context().usableChain().get(0), Scene.pApplet.color(0,255,0));
      viz.addHighlightNode("Pe", context().endEffectorInformation().node(), Scene.pApplet.color(0,255,0));
      viz.addHighlightNode("Pt", context().target(), Scene.pApplet.color(0,255,0));
      viz.addFrame("Initial configuration"); //Just add the initial image
    }
  }

  @Override
  public void applyActions(int i) {
    applyCCD(viz, this, i, _context.applyDelegation());
    applyOrientationalCCD(this, i);
  }

  //CCD - BASIC OPERATIONS
  public  static Quaternion findCCD(NodeInformation j_i, Vector endEffector, Vector target, boolean checkHinge){
    Vector p = endEffector;
    Vector q = target;
    if (checkHinge && j_i.node().constraint() != null && j_i.node().constraint() instanceof Hinge) {
      Util.checkHinge(j_i, endEffector, target);
    }
    //Apply desired rotation removing twist component
    Quaternion delta = new Quaternion(p, q);
    return delta;
  }

  public static Quaternion findOrientationalCCD(NodeInformation j_i, NodeInformation endEffector, Node target){
    Quaternion O_i = j_i.orientationCache();
    Quaternion O_i_inv = O_i.inverse();
    Quaternion O_eff = endEffector.orientationCache();
    Quaternion O_i1_to_eff = Quaternion.compose(O_i.inverse(), O_eff);
    O_i1_to_eff.normalize();
    Quaternion delta = Quaternion.compose(O_i_inv, target.orientation());
    delta.normalize();
    delta.compose(O_i1_to_eff.inverse());
    delta.normalize();
    return delta;
  }

  public static void applyCCD(Viz viz, Heuristic heuristic, int i, boolean applyDelegation){
    applyCCD(viz, heuristic, i, applyDelegation ? heuristic.context().clamping(i) : 1);
  }

  public static void applyCCD(Viz viz, Heuristic heuristic, int i, float clamping){
    Context context = heuristic.context();
    NodeInformation j_i = context.usableChainInformation().get(i);
    NodeInformation endEffector = context.endEffectorInformation();
    Vector eff_wrt_j_i = j_i.locationWithCache(endEffector.positionCache());
    Vector target_wrt_j_i = j_i.locationWithCache(context.worldTarget().position());

    Quaternion delta = findCCD(j_i, eff_wrt_j_i, target_wrt_j_i, true);
    delta = Util.constraintRotation(j_i, delta);
    if (clamping < 1) {
      delta = Util.clampRotation(delta, context.maxAngleAtJoint(i), clamping);
    }

    //Add information to VIZ
    if(viz != null) {
      viz.clearFigures();
      viz.addHighlightNode("Pc", j_i.node(), Scene.pApplet.color(252, 186, 3));
      viz.addHighlightNode("Pe", context.endEffectorInformation().node(), Scene.pApplet.color(0,255,0));
      viz.addHighlightNode("Pt", context.target(), Scene.pApplet.color(0,255,0));
      viz.addArrow("", j_i.positionCache(), endEffector.positionCache(), Scene.pApplet.color(0, 255, 255), Scene.pApplet.color(0));
      viz.addArrow("", j_i.positionCache(), context.worldTarget().position(), Scene.pApplet.color(0, 255, 0), Scene.pApplet.color(0));
      viz.addArc("", j_i.node().position(), endEffector.positionCache(), context.worldTarget().position(), Scene.pApplet.color(255,0,0));

      float mag = j_i.node().location(context.worldTarget()).magnitude();
      Vector finalEff = j_i.node().location(endEffector.node().position()).normalize(null);
      finalEff.multiply(mag);
      finalEff = j_i.node().worldLocation(delta.rotate(finalEff));
      viz.addArc("", j_i.node().position(), endEffector.positionCache(), finalEff, Scene.pApplet.color(0,255,0));
      viz.addFrame("Align P and Q");
    }

    j_i.rotateAndUpdateCache(delta, false, endEffector); //Apply local rotation

    if(viz != null){
      viz.clearArcs();
      viz.clearArrows();
      viz.addArrow("", j_i.positionCache(), endEffector.positionCache(), Scene.pApplet.color(0, 255, 255), Scene.pApplet.color(0));
      viz.addArrow("", j_i.positionCache(), context.worldTarget().position(), Scene.pApplet.color(0, 255, 0), Scene.pApplet.color(0));
      viz.addFrame("New Configuration after alignment");
      viz.clearHighlightingNodes();
    }
  }


  public static void applyOrientationalCCD(Heuristic heuristic, int i){
    Context context = heuristic.context();
    NodeInformation j_i = context.usableChainInformation().get(i);
    if (context.direction() && i != context.endEffectorId()) {
      Quaternion deltaDirection = findOrientationalCCD(j_i, context.usableChainInformation().get(context.endEffectorId()), context.worldTarget());
      //normalize distance
      float oerror = Context.orientationError(context.usableChain().get(context.endEffectorId()).orientation(), context.worldTarget().orientation(), false);
      float ang = (float) Math.PI * oerror * (i + 1) / context.usableChain().size();
      deltaDirection = Util.clampRotation(deltaDirection, ang);
      j_i.rotateAndUpdateCache(deltaDirection, false, context.endEffectorInformation());
    }
  }



  @Override
  public NodeInformation[] nodesToModify(int i) {
    return new NodeInformation[]{_context.usableChainInformation().get(i - 1), _context.usableChainInformation().get(i)};
  }
}
