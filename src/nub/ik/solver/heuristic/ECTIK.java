package nub.ik.solver.heuristic;

import nub.core.constraint.Constraint;
import nub.core.constraint.Hinge;
import nub.ik.solver.NodeInformation;
import nub.ik.solver.Context;
import nub.ik.solver.NodeState;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

public class ECTIK extends Heuristic {
  /**
   * The idea of this heuristics is to apply and interchange popular CCD Step along with Triangulation step. Here most of the work is done by the first joints,
   * hence the obtained solution could not seem natural when working with unconstrained chains. For this purposes a smoothing stage is required in which each
   * joint will try to do and delegate work.
   */


  public ECTIK(Context context) {
    super(context);
  }


  @Override
  public void prepare() {
    //Update cache of usable chain
    NodeInformation._updateCache(_context.usableChainInformation());
  }

  protected int _times = 2;

  public static void applyCombined(Heuristic heuristic, int i, int times){
    Context context = heuristic._context;
    Vector target = context.worldTarget().position();

    NodeInformation j_i = context.usableChainInformation().get(i);
    Vector eff_wrt_j_i = j_i.locationWithCache(context.endEffectorInformation().positionCache());
    Vector target_wrt_j_i = j_i.locationWithCache(target);

    if (i == context.endEffectorId() - 1) {
      Quaternion q_i = findCCD(context, i, j_i, eff_wrt_j_i, target_wrt_j_i, true);
      j_i.rotateAndUpdateCache(q_i, false, context.endEffectorInformation()); //Apply local rotation
      CCD.applyOrientationalCCD(heuristic, i);
      return;
    }

    NodeInformation j_i1 = context.usableChainInformation().get(i + 1);
    j_i1.updateCacheUsingReference();
    Vector eff_wrt_j_i1 = j_i1.locationWithCache(context.endEffectorInformation().positionCache());
    Vector target_wrt_j_i1 = j_i1.locationWithCache(target);
    //Find the two solutions of the triangulation problem on joint j_i1
    Quaternion[] solutions;
    solutions = findTriangulationSolutions(context, i, j_i, j_i1, eff_wrt_j_i1, target_wrt_j_i1);

    //Keep original State of J_i and J_i1
    NodeInformation endEffector = context.endEffectorInformation();

    NodeState initial_j_i = new NodeState(j_i);
    NodeState initial_j_i1 = new NodeState(j_i1);
    NodeState initial_eff = new NodeState(endEffector);

    NodeState[] final_j_i = new NodeState[solutions.length];
    NodeState[] final_j_i1 = new NodeState[solutions.length];
    NodeState[] final_eff = new NodeState[solutions.length];

    int best = 0; // keep track of best solution
    float best_dist = Float.MAX_VALUE, best_angle = Float.MAX_VALUE;
    for (int s = 0; s < solutions.length; s++) {
      float a; //amount or rotation applied
      j_i1.updateCacheUsingReference();
      //Apply solution find by triangulation
      Quaternion q = solutions[s];
      j_i.rotateAndUpdateCache(q, false, endEffector);

      for (int t = 0; t < times; t++) {
        j_i1.updateCacheUsingReference();
        Quaternion q_i1 = findCCD(context, i + 1, j_i1, j_i1.locationWithCache(endEffector.positionCache()), j_i1.locationWithCache(target), true);
        j_i1.rotateAndUpdateCache(q_i1, false, endEffector);
        Quaternion q_i = findCCD(context, i, j_i, j_i.locationWithCache(endEffector.positionCache()), j_i.locationWithCache(target), true);
        j_i.rotateAndUpdateCache(q_i, false, endEffector);
      }
      j_i1.updateCacheUsingReference();

      CCD.applyOrientationalCCD(heuristic, i + 1);
      //store state in final vector
      final_j_i[s] = new NodeState(j_i);
      final_j_i1[s] = new NodeState(j_i1);
      final_eff[s] = new NodeState(endEffector);

      a = Math.abs(context.quaternionDistance(initial_j_i.rotation(), j_i.node().rotation()) + context.quaternionDistance(initial_j_i1.rotation(), j_i1.node().rotation()));
      float dist;
      if (!context.applyDelegation()) {
        dist = context.error(endEffector, context.worldTarget());
        if(!context.direction())dist /= context.searchingAreaRadius();
      } else {
        float error = context.positionError(endEffector.positionCache(), target);
        if (context.direction()) {
          float orientationError = context.orientationError(endEffector.orientationCache(), context.worldTarget().orientation(), false);
          float weighted_error = error / context.searchingAreaRadius();
          error = context.orientationWeight() * orientationError + (1 - context.orientationWeight()) * (weighted_error);
        }
        dist = error;
      }

      if(context.applyDelegation()) dist = dist + 1f * a;
      else dist = dist + 0.1f * a;// + solutions[s].value();// + length_distance * _lengthWeight;


      if (dist < best_dist) {
        best_dist = dist;
        best = s;
      }

      //reset state
      j_i.setCache(initial_j_i.position().get(), initial_j_i.orientation().get());
      j_i.node().setRotation(initial_j_i.rotation().get());
      j_i1.setCache(initial_j_i1.position().get(), initial_j_i1.orientation().get());
      j_i1.node().setRotation(initial_j_i1.rotation().get());
      endEffector.setCache(initial_eff.position().get(), initial_eff.orientation().get());
    }

    //Apply best solution
    j_i.setCache(final_j_i[best].position().get(), final_j_i[best].orientation().get());
    Constraint c_i = j_i.node().constraint();
    j_i.node().setConstraint(null);
    j_i.node().setRotation(final_j_i[best].rotation().get());
    j_i.node().setConstraint(c_i);
    j_i1.setCache(final_j_i1[best].position().get(), final_j_i1[best].orientation().get());
    Constraint c_i1 = j_i1.node().constraint();
    j_i1.node().setConstraint(null);
    j_i1.node().setRotation(final_j_i1[best].rotation().get());
    j_i1.node().setConstraint(c_i1);
    endEffector.setCache(final_eff[best].position().get(), final_eff[best].orientation().get());
  }

  @Override
  public void applyActions(int i) {
    applyCombined(this, i, _times);
  }


  protected static Quaternion[] findTriangulationSolutions(Context context, int i, NodeInformation j_i, NodeInformation j_i1, Vector endEffector, Vector target) {
    Vector a = j_i1.node().translation();
    Vector b = j_i.locationWithCache(context.endEffectorInformation());
    b.subtract(a);
    Vector c = j_i.locationWithCache(context.worldTarget().position());

    if (j_i.node().constraint() != null && j_i.node().constraint() instanceof Hinge) {
      Hinge h = (Hinge) j_i.node().constraint();
      Quaternion quat = Quaternion.compose(j_i.node().rotation().inverse(), h.idleRotation());
      Vector tw = h.restRotation().rotate(new Vector(0, 0, 1));
      tw = quat.rotate(tw);
      //Project b & c on the plane of rot
      a = Vector.projectVectorOnPlane(a, tw);
      b = Vector.projectVectorOnPlane(b, tw);
      c = Vector.projectVectorOnPlane(c, tw);
    }

    float a_mag = a.magnitude(), b_mag = b.magnitude(), c_mag = c.magnitude();
    Quaternion[] deltas;
    if (a_mag + b_mag <= c_mag) {
      //Chain must be extended as much as possible
      deltas = new Quaternion[]{new Quaternion(a, c)};
    } else if (c_mag < Math.abs(a_mag - b_mag)) {
      //Chain must be contracted as much as possible
      deltas = new Quaternion[]{new Quaternion(a, Vector.multiply(c, -1))};
    } else {
      //Apply law of cosines
      float B = TIK.findCfromTriangle(a_mag,c_mag,b_mag);
      float angle_1 = Vector.angleBetween(a,c) - B;
      float angle_2 = -Vector.angleBetween(a,c) - B;
      Vector normal = Vector.cross(a, c, null);
      if (normal.squaredNorm() < 0.0001f) {
        normal = a.orthogonalVector();
      }
      deltas = new Quaternion[]{new Quaternion(normal, angle_1), new Quaternion(normal, angle_2)};
    }

    for (int k = 0; k < deltas.length; k++) {
      if (j_i.node().constraint() != null) {
        deltas[k] = j_i.node().constraint().constrainRotation(deltas[k], j_i.node());
      }
      if (context.applyDelegation()) {
        deltas[k] = Util.clampRotation(deltas[k], context.maxAngleAtJoint(i), context.clamping(0));
      }
      deltas[k].normalize();
    }
    return deltas;
  }

  protected static Quaternion findCCD(Context context, int i, NodeInformation j_i, Vector endEffector, Vector target, boolean checkHinge) {
    Quaternion delta = CCD.findCCD(j_i, endEffector, target, checkHinge);
    delta = Util.constraintRotation(j_i, delta);
    if (context.applyDelegation()) {
        delta = Util.clampRotation(delta, context.maxAngleAtJoint(i), context.clamping(0));
    }
    return delta;
  }

  @Override
  public NodeInformation[] nodesToModify(int i) {
    //return new NodeInformation[]{_context.usableChainInformation().get(i - 1), _context.usableChainInformation().get(i)};
    return null;
  }
}
