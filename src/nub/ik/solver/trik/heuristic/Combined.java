package nub.ik.solver.trik.heuristic;

import nub.core.constraint.Constraint;
import nub.core.constraint.Hinge;
import nub.ik.solver.trik.NodeInformation;
import nub.ik.solver.trik.Context;
import nub.ik.solver.trik.NodeState;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

public class Combined extends Heuristic {
  /**
   * The idea of this heuristics is to apply and interchange popular CCD Step along with Triangulation step. Here most of the work is done by the first joints,
   * hence the obtained solution could not seem natural when working with unconstrained chains. For this purposes a smoothing stage is required in which each
   * joint will try to do and delegate work.
   */


  public Combined(Context context) {
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
      j_i1.rotateAndUpdateCache(q, false, endEffector);
      //Apply CCD t times (best local action if joint rotation constraints are quite different)
      j_i.rotateAndUpdateCache(findCCD(context, i, j_i, j_i.locationWithCache(endEffector.positionCache()), j_i.locationWithCache(target), true), true, endEffector);
      j_i1.updateCacheUsingReference();

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
    Vector v_i = j_i1.locationWithCache(j_i.positionCache());
    Vector normal;
    //In this case we apply triangulation over j_i1
    if (j_i1.node().constraint() instanceof Hinge) {
      //Project endEffector to lie on the plane defined by the axis of rotation
      Hinge h_i1 = ((Hinge) j_i1.node().constraint());
      //1. find rotation axis
      normal = h_i1.orientation().rotate(new Vector(0, 0, 1));
      normal = j_i1.node().rotation().inverse().rotate(normal);
      normal.normalize();
      //2. project target and end effector
      v_i = Vector.projectVectorOnPlane(v_i, normal);
      endEffector = Vector.projectVectorOnPlane(endEffector, normal);
      target = Vector.projectVectorOnPlane(target, normal);
    } else {
      //find the normal vector (we know in advance that normal = Z axis if the scene is 2D)
      normal = context.is2D() ? new Vector(0,0,1) : Vector.cross(endEffector, target, null);
      //If target and eff are collinear compare v_i against target
      if (normal.squaredNorm() < 0.0001f) {
        normal = Vector.cross(target, v_i, null);
        //If v_i and target are collinear use any vector
        if (normal.squaredNorm() < 0.0001f) {
          normal = target.orthogonalVector();
        }
      }
      normal.normalize();
    }
    //Find the two solutions of the triangulation problem assuming no constraints
    Vector a = v_i;
    Vector a_neg = Vector.multiply(a, -1);
    Vector b = endEffector;
    Vector c = Vector.subtract(target, a);
    float a_mag = a.magnitude(), b_mag = b.magnitude(), c_mag = c.magnitude();

    float angle = Vector.angleBetween(a,b);

    float angle_1, angle_2;

    if (Vector.dot(Vector.cross(b, a_neg, null), normal) < 0) {
      angle = -angle;
    }

    if (a_mag + b_mag <= c_mag) {
      if (angle == 0) {
        angle_1 = (float) (Math.PI);
        angle_2 = -(float) (Math.PI);
      } else {
        angle_1 = Math.signum(angle) * (float) (Math.PI) - angle;
        angle_2 = (float) (-Math.signum(angle_1) * 2 * Math.PI + angle_1);
      }
    } else if (c_mag < Math.abs(a_mag - b_mag)) {
      angle_1 = -angle;
      angle_2 = (float) (-Math.signum(angle_1) * 2 * Math.PI + angle_1);
    } else {
      //Apply law of cosines
      float current = angle;
      float expected = Triangulation.findCfromTriangle(a_mag, b_mag, c_mag);
      angle_1 = expected - current;
      angle_2 = -current - expected;
    }

    Quaternion[] deltas = new Quaternion[2];
    deltas[0] = new Quaternion(normal, angle_1);
    deltas[1] = new Quaternion(normal, angle_2);

    for (int k = 0; k < deltas.length; k++) {
      if (j_i1.node().constraint() != null) {
        deltas[k] = j_i1.node().constraint().constrainRotation(deltas[k], j_i1.node());
      }
      if (context.applyDelegation()) {
        deltas[k] = Util.clampRotation(deltas[k], context.maxAngleAtJoint(i + 1), context.clamping(0));
      }
      deltas[k].normalize();
    }

    //If the rotation is the same then explore only one rotation
    if(Context.quaternionDistance(deltas[0], deltas[1]) < 0.0001f){
      return new Quaternion[]{deltas[0]};
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
