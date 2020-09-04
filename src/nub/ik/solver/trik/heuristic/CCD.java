package nub.ik.solver.trik.heuristic;

import nub.core.Node;
import nub.core.constraint.Hinge;
import nub.ik.solver.trik.NodeInformation;
import nub.ik.solver.trik.Context;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

public class CCD extends Heuristic {
  /**
   * The idea of this heuristics is to apply popular CCD Step. Here most of the work is done by the last joint and as them could move
   * that what is truly required the final pose of the chain will not be perceived as a natural pose.
   */
  public CCD(Context context) {
    super(context);
  }


  @Override
  public void prepare() {
    //Update cache of usable chain
    NodeInformation._updateCache(_context.usableChainInformation());
  }

  @Override
  public void applyActions(int i) {
    applyCCD(this, i, _context.applyDelegation());
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

  public static void applyCCD(Heuristic heuristic, int i, boolean applyDelegation){
    applyCCD(heuristic, i, applyDelegation ? heuristic._context.clamping(i) : 1);
  }

  public static void applyCCD(Heuristic heuristic, int i, float clamping){
    Context context = heuristic._context;
    NodeInformation j_i = context.usableChainInformation().get(i);
    NodeInformation endEffector = context.endEffectorInformation();
    Vector eff_wrt_j_i = j_i.locationWithCache(endEffector.positionCache());
    Vector target_wrt_j_i = j_i.locationWithCache(context.worldTarget().position());
    Quaternion delta = findCCD(j_i, eff_wrt_j_i, target_wrt_j_i, true);
    delta = Util.constraintRotation(j_i, delta);
    if (clamping < 1) {
      delta = Util.clampRotation(delta, context.maxAngleAtJoint(i), clamping);
    }
    j_i.rotateAndUpdateCache(delta, false, endEffector); //Apply local rotation
  }


  public static void applyOrientationalCCD(Heuristic heuristic, int i){
    Context context = heuristic._context;
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
