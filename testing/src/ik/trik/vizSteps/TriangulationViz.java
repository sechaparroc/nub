package ik.trik.vizSteps;

import nub.core.constraint.Hinge;
import nub.ik.solver.trik.Context;
import nub.ik.solver.trik.NodeInformation;
import nub.ik.solver.trik.heuristic.CCD;
import nub.ik.solver.trik.heuristic.Heuristic;
import nub.ik.solver.trik.heuristic.Util;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;

public class TriangulationViz extends Heuristic{
  public Viz viz;

  public TriangulationViz(Context context, Viz viz) {
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
    applyTriangulation(viz, this, i, i < _context.endEffectorId() - 1 && !_context.topToBottom(), _context.applyDelegation());
    if(i == _context.endEffectorId() - 1) return;
    if(_context.topToBottom()) {
      CCD.applyOrientationalCCD(this, i);
      _context.usableChainInformation().get(i + 1).updateCacheUsingReference();
    } else{
      CCD.applyOrientationalCCD(this, i + 1);
    }
  }

  public static void applyTriangulation(Viz viz, Heuristic heuristic, int i, boolean updateCouple, boolean applyDelegation){
    Context context = heuristic.context();
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

    String triangulationCase = "";
    Quaternion delta;
    if (a_mag + b_mag <= c_mag) {
      //Chain must be extended as much as possible
      delta = new Quaternion(a, c);
      triangulationCase = "Chain must be extended as much as possible";
    } else if (c_mag < Math.abs(a_mag - b_mag)) {
      //Chain must be contracted as much as possible
      delta = new Quaternion(a, Vector.multiply(c, -1));
      triangulationCase = "Chain must be contracted as much as possible";
    } else {
      //Apply law of cosines
      float B = findCfromTriangle(a_mag,c_mag,b_mag);
      float angle = Vector.angleBetween(a,c) - B;
      Vector normal = Vector.cross(a, c, null);
      if (normal.squaredNorm() < 0.0001f) {
        normal = a.orthogonalVector();
      }

      delta = new Quaternion(normal, angle);
      triangulationCase = "It is possible to construct a triangle with a,b and c";
    }

    if(viz != null){
      viz.clearFigures();
      viz.addHighlightNode("Pe", context.endEffectorInformation().node(), Scene.pApplet.color(0,255,0));
      viz.addHighlightNode("Pt", context.target(), Scene.pApplet.color(0,255,0));
      viz.addArrow("a", j_i.node().position(), j_i1.node().position(), Scene.pApplet.color(255,255,0), Scene.pApplet.color(0));
      viz.addArrow("b", j_i1.node().position(), context.endEffectorInformation().node().position(), Scene.pApplet.color(0,255,255), Scene.pApplet.color(0));
      viz.addArrow("c", j_i.node().position(), context.worldTarget().position(), Scene.pApplet.color(255,0,255), Scene.pApplet.color(0));
      //Highlight related nodes
      viz.addHighlightNode("", j_i.node(), Scene.pApplet.color(252, 186,3));
      viz.addHighlightNode("", j_i1.node(), Scene.pApplet.color(0, 255, 255));

      viz.addFrame("Define the vectors a, b, c" + " a" + a_mag + " b " + b_mag + " c " + c_mag);
      //Draw target arrow
      Vector a_hat = j_i.node().worldLocation(delta.rotate(j_i1.node().translation()));
      viz.addArrow("a'", j_i.node().position(), a_hat, Scene.pApplet.color(255,255,0, 200), Scene.pApplet.color(0));
      Vector b_hat = Vector.subtract(context.worldTarget().position(), a_hat);
      b_hat.normalize();
      b_hat.multiply(b_mag);
      b_hat.add(a_hat);

      viz.addArrow("b'", a_hat, b_hat, Scene.pApplet.color(0,255,255, 200), Scene.pApplet.color(0));
      viz.addShape("triangle", Scene.pApplet.color(255,255,0, 100), j_i.node().position(), a_hat, b_hat);
      viz.addFrame(triangulationCase);
      viz.clearFigures();

      viz.addShape("triangle", Scene.pApplet.color(255,255,0, 100), j_i.node().position(), a_hat, b_hat);
      viz.addHighlightNode("Pe", context.endEffectorInformation().node(), Scene.pApplet.color(0,255,0));
      viz.addHighlightNode("Pt", context.target(), Scene.pApplet.color(0,255,0));
      a_hat = j_i.node().worldLocation(delta.rotate(j_i1.node().translation()));
      viz.addArrow("a'", j_i.node().position(), a_hat, Scene.pApplet.color(255,255,0, 200), Scene.pApplet.color(0));
      b_hat = Vector.subtract(context.worldTarget().position(), a_hat);
      b_hat.normalize();
      b_hat.multiply(b_mag);
      b_hat.add(a_hat);
      viz.addArrow("b'", a_hat, b_hat, Scene.pApplet.color(0,255,255, 200), Scene.pApplet.color(0));
    }

    delta = Util.constraintRotation(j_i, delta);
    if (applyDelegation) {
      delta = Util.clampRotation(delta, context.maxAngleAtJoint(i), context.clamping(i));
    }


    j_i.rotateAndUpdateCache(delta, false, context.endEffectorInformation()); //Apply local rotation

    if(updateCouple && i + 1 < context.endEffectorId()) {
      //update the next joint using CCD
      j_i1.updateCacheUsingReference();
      //Find q_i1
      CCD.applyCCD(heuristic, i + 1, applyDelegation);
    }
    if(viz != null){
      viz.addArrow("a", j_i.node().position(), j_i1.node().position(), Scene.pApplet.color(255,255,0), Scene.pApplet.color(0));
      viz.addArrow("b", j_i1.node().position(), context.endEffectorInformation().node().position(), Scene.pApplet.color(0,255,255), Scene.pApplet.color(0));
      viz.addArrow("c", j_i.node().position(), context.worldTarget().position(), Scene.pApplet.color(255,0,255), Scene.pApplet.color(0));
      //Draw target arrow
      viz.addFrame(triangulationCase);
    }
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
