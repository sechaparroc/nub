package nub.ik.solver.trik.heuristic;

import nub.core.constraint.Hinge;
import nub.ik.solver.trik.NodeInformation;
import nub.ik.solver.trik.Context;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

public class Triangulation extends Heuristic {
  /**
   * The idea of this heuristics is to apply popular CCD Step. Here most of the work is done by the last joint and as them could move
   * that what is truly required the final pose of the chain will not be perceived as a natural pose.
   */
  public Triangulation(Context context) {
    super(context);
  }


  @Override
  public void prepare() {
    //Update cache of usable chain
    NodeInformation._updateCache(_context.usableChainInformation());
    _context.setTopToBottom(false);
  }

  @Override
  public void applyActions(int i) {
    applyTriangulation(this, i, i < _context.endEffectorId() - 1 && !_context.topToBottom());
    if(i == _context.endEffectorId() - 1) return;
    if(_context.topToBottom()) {
      CCD.applyOrientationalCCD(this, i);
      _context.usableChainInformation().get(i + 1).updateCacheUsingReference();
    } else{
      CCD.applyOrientationalCCD(this, i + 1);
    }
  }

  public static void applyTriangulation(Heuristic heuristic, int i, boolean updateCouple){
    Context context = heuristic._context;
    NodeInformation j_i = context.usableChainInformation().get(i);
    NodeInformation j_i1 = context.usableChainInformation().get(i + 1);
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
    Quaternion delta;
    if (a_mag + b_mag <= c_mag) {
      //Chain must be extended as much as possible
      delta = new Quaternion(a, c);
    } else if (c_mag < Math.abs(a_mag - b_mag)) {
      //Chain must be contracted as much as possible
      delta = new Quaternion(a, Vector.multiply(c, -1));
    } else {
      //Apply law of cosines
      float B = findCfromTriangle(a_mag,c_mag,b_mag);
      float angle = Vector.angleBetween(a,c) - B;
      delta = new Quaternion(Vector.cross(a, c, null), angle);
    }
    if (heuristic._smooth) delta = Util.clampRotation(delta, heuristic._smoothAngle);
    delta = Util.constraintRotation(j_i, delta);
    j_i.rotateAndUpdateCache(delta, false, context.endEffectorInformation()); //Apply local rotation

    if(!updateCouple) return;
    //update the next joint using CCD
    j_i1.updateCacheUsingReference();
    CCD.applyCCD(heuristic, i + 1);
  }


  /*
   * Robust implementation of law of cosines to find angle C
   * more info at https://people.eecs.berkeley.edu/~wkahan/Triangle.pdf
   * */
  public static float findCfromTriangle(float a, float b, float c){
    //swap if required
    if(a < b){
      float aux = a;
      a = b;
      b = aux;
    }
    //compute mu
    float mu;
    if(b >= c && c >= 0){
      mu = c - (a - b);
    } else if(c > b && b >= 0){
      mu = b - (a - c);
    } else {
      return Float.NaN;
    }
    return (float)(2*Math.atan(Math.sqrt(Math.abs(((a-b)+c)*mu/((a+(b+c))*((a-c)+b))))));
  }


  @Override
  public NodeInformation[] nodesToModify(int i) {
    return new NodeInformation[]{_context.usableChainInformation().get(i - 1), _context.usableChainInformation().get(i)};
  }
}
