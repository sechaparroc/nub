package ik.trik.vizSteps;

import nub.core.constraint.Hinge;
import nub.ik.solver.Context;
import nub.ik.solver.NodeInformation;
import nub.ik.solver.heuristic.CCD;
import nub.ik.solver.heuristic.Heuristic;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;

public class TRIKViz extends Heuristic {
  public Viz viz;

  public TRIKViz(Context context, Viz viz) {
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
      viz.addFrame("dummy");
      viz.sequence.clear();
      viz.addHighlightNode("P\u2080", context().usableChain().get(0), Scene.pApplet.color(0,255,0));
      viz.addHighlightNode("Pe", context().endEffectorInformation().node(), Scene.pApplet.color(0,255,0));
      viz.addHighlightNode("Pt", context().target(), Scene.pApplet.color(0,255,0));
      viz.addFrame("Initial configuration"); //Just add the initial image
    }
  }

  @Override
  public void applyActions(int i) {
    applyTRIK(viz, this, i);
    if(_context.topToBottom()) {
      CCD.applyOrientationalCCD(this, i);
      _context.usableChainInformation().get(i + 1).updateCacheUsingReference();
    } else{
      CCD.applyOrientationalCCD(this, i);
    }
  }

  @Override
  public NodeInformation[] nodesToModify(int i) {
    if(i == _context.endEffectorId() - 1) return new NodeInformation[]{_context.usableChainInformation().get(i)};
    else return new NodeInformation[]{_context.usableChainInformation().get(i), _context.usableChainInformation().get(i + 1)};
  }

  public static Quaternion findReachStep(Viz viz, int i, NodeInformation j_i, NodeInformation j_i1, Vector endEffector, Vector target, boolean checkHinge){
    Vector eff = endEffector;
    Vector t = target;
    if (checkHinge && j_i.node().constraint() != null && j_i.node().constraint() instanceof Hinge) {
      Hinge h = (Hinge) j_i.node().constraint();
      Quaternion quat = Quaternion.compose(j_i.node().rotation().inverse(), h.idleRotation());
      Vector tw = h.restRotation().rotate(new Vector(0, 0, 1));
      tw = quat.rotate(tw);
      eff = Vector.projectVectorOnPlane(eff, tw);
      t = Vector.projectVectorOnPlane(t, tw);
    }
    //Assume that j_i and j_i1 reaches maintains its orientation and reaches the target
    Vector v1 = j_i1.node().translation().get();
    Vector v2 = Vector.subtract(v1, eff);
    v2.add(t);

    //Apply desired rotation removing
    Quaternion delta = new Quaternion(v1, v2);
    if (j_i.node().constraint() != null) {
      delta = j_i.node().constraint().constrainRotation(delta, j_i.node());
    }
    delta.normalize();

    if (viz != null){
      Vector j_i_hat = v2.normalize(null);
      j_i_hat.multiply(j_i1.node().translation().magnitude());

      viz.clearFigures();
      //highlight end effector and target
      viz.addHighlightNode("Pe", viz.heuristic.context().endEffectorInformation().node(), Scene.pApplet.color(0,255,0));
      viz.addHighlightNode("Pt", viz.heuristic.context().target(), Scene.pApplet.color(0,255,0));
      //Highlight nodes to move
      viz.addHighlightNode("Ji", j_i.node(), Scene.pApplet.color(0,255,255));

      //1. Show translation to Target
      viz.showStructureCopy(0, i + 1); //show copy
      viz.updateStructureCopy(0); //update copy
      viz.structureCopies.get(0).get(i + 1).setTranslation(v2); //translate to a desired position
      viz.setColorCopy(0, Scene.pApplet.color(255,0,0,100)); //set color
      //Draw translation arrow from j_i1 to target
      viz.addArrow("t1", j_i1.node().position(), j_i.node().worldLocation(v2),
          Scene.pApplet.color(255,0,0), Scene.pApplet.color(0));
      viz.addFrame("Reach Target");

      if(i + 1 == viz.heuristic.context().endEffectorId()){
        viz.unhighlightNode(viz.heuristic.context().endEffectorInformation().node());
      }
      viz.addHighlightNode("Ji1", j_i1.node(), Scene.pApplet.color(0,255,255));
      viz.addHighlightNode("Ji1'", viz.structureCopies.get(1).get(i + 1), Scene.pApplet.color(0,255,255));
      //2. Fix bone length
      viz.showStructureCopy(1,i + 1); //show another copy
      viz.updateStructureCopy(1);//update copy
      viz.structureCopies.get(1).get(i + 1).setTranslation(j_i_hat);//move to final expected position
      viz.addArrow("t2", j_i.node().worldLocation(v2), j_i.node().worldLocation(j_i_hat),
          Scene.pApplet.color(0,255,255), Scene.pApplet.color(0));
      viz.addFrame("Fix Length");
      viz.clearArrows();
      viz.hideStructureCopy(0);


      //set color
      viz.setColorCopy(0, Scene.pApplet.color(255,0,0,100));
      //drawn an arrow to the expected target
      viz.addArrow("", j_i.node().position(), j_i.node().worldLocation(j_i1.node().translation()),
          Scene.pApplet.color(0,255,255), Scene.pApplet.color(0));

      viz.addArrow("", j_i.node().position(), j_i.node().worldLocation(j_i_hat),
          Scene.pApplet.color(0,255,0), Scene.pApplet.color(0));

      viz.addArc("", j_i.node().position(), j_i.node().worldLocation(j_i1.node().translation()), j_i.node().worldLocation(j_i_hat), Scene.pApplet.color(255,0,0));
      viz.addArc("", j_i.node().position(), j_i.node().worldLocation(j_i1.node().translation()), j_i.node().worldLocation(delta.rotate(j_i1.node().translation())), Scene.pApplet.color(0,255,0));
      viz.addFrame("Rotate J_i"); //Just add the initial image
    }
    return delta;
  }

  public static Quaternion findNextFixStep(NodeInformation j_i1, Quaternion q_i1, Quaternion q_i1_inv, Quaternion delta){
    Quaternion delta_i1 = Quaternion.compose(q_i1_inv, delta.inverse());
    delta_i1.normalize();
    delta_i1.compose(q_i1);
    delta_i1.normalize();
    if (j_i1.node().constraint() != null) {
      delta_i1 = j_i1.node().constraint().constrainRotation(delta_i1, j_i1.node());
    }
    return delta_i1;
  }

  public static Quaternion findCurrentFixStep(NodeInformation j_i, Quaternion q_i1, Quaternion q_i1_inv, Quaternion delta_i1, boolean constraint){
    Quaternion delta_i = Quaternion.compose(q_i1, delta_i1.inverse());
    delta_i.normalize();
    delta_i.compose(q_i1_inv);
    delta_i.normalize();
    if (constraint && j_i.node().constraint() != null) {
      delta_i = j_i.node().constraint().constrainRotation(delta_i, j_i.node());
    }
    return delta_i;
  }

  public static void applyTRIK(Viz viz, Heuristic heuristic, int i){
    Context context = heuristic.context();
    Vector target = context.worldTarget().position();
    NodeInformation j_i = context.usableChainInformation().get(i);
    Vector eff_wrt_j_i = j_i.locationWithCache(context.endEffectorInformation().positionCache());
    Vector target_wrt_j_i = j_i.locationWithCache(target);
    NodeInformation j_i1 = context.usableChainInformation().get(i + 1);
    //find the rotation that reaches the target while keeps the orientation
    Quaternion delta_i = findReachStep(viz, i, j_i, j_i1, eff_wrt_j_i, target_wrt_j_i, true);
    if(i == context.endEffectorId() - 1){
      //check just a joint instead of a couple of joints
      j_i.rotateAndUpdateCache(delta_i, false, context.endEffectorInformation());
      if (viz != null){
        viz.clearArrows();
        viz.clearArcs();
        viz.addFrame("Rotate J_i"); //Just add the initial image
      }
    } else {
      //apply fix step on i_1
      Quaternion q_i1 = j_i1.node().rotation();
      Quaternion q_i1_inv = q_i1.inverse();
      Quaternion delta_i1 = findNextFixStep(j_i1, q_i1, q_i1_inv, delta_i);
      //Perhaps in this step it is not required to check the constraint
      Quaternion delta_i_hat = findCurrentFixStep(j_i, q_i1, q_i1_inv, delta_i1, true);
      //Apply the found solutions
      j_i.rotateAndUpdateCache(delta_i_hat, false, context.endEffectorInformation());
      if (viz != null){
        viz.clearArrows();
        viz.clearArcs();
        viz.addFrame("Rotate J_i"); //Just add the initial image
      }
      j_i1.updateCacheUsingReference();
      if (viz != null){
        viz.clearFigures();
        //highlight end effector and target
        viz.addHighlightNode("Pe", viz.heuristic.context().endEffectorInformation().node(), Scene.pApplet.color(0,255,0));
        viz.addHighlightNode("Pt", viz.heuristic.context().target(), Scene.pApplet.color(0,255,0));
        //Highlight nodes to move
        viz.addHighlightNode("Ji1", j_i1.node(), Scene.pApplet.color(0,255,255));

        viz.addHighlightNode("Ji2", viz.heuristic.context().usableChain().get(i + 2), Scene.pApplet.color(0,255,255));
        viz.addHighlightNode("Ji2'", viz.structureCopies.get(1).get(i + 2), Scene.pApplet.color(0,255,255));

        viz.showStructureCopy(1,i + 2);
        viz.structureCopies.get(1).get(i + 1).setPosition(j_i1.node().position().get());
        viz.addArrow("", j_i1.node().position(),
            context.usableChainInformation().get(i + 2).node().position(),
            Scene.pApplet.color(0,255,255), Scene.pApplet.color(0));

        viz.addArrow("'", j_i1.node().position(),
            viz.structureCopies.get(1).get(i + 2).position(),
            Scene.pApplet.color(0,255,0), Scene.pApplet.color(0));

        viz.addArc("", j_i1.node().position(),
            context.usableChainInformation().get(i + 2).node().position(),
            viz.structureCopies.get(1).get(i + 2).position(), Scene.pApplet.color(255,0,0));
        viz.addArc("", j_i1.node().position(),
            context.usableChainInformation().get(i + 2).node().position(),
            j_i1.node().worldLocation(delta_i1.rotate(context.usableChainInformation().get(i + 2).node().translation())),
            Scene.pApplet.color(0,255,0));

        viz.addFrame("Define target for J_i1"); //Just add the initial image
      }
      j_i1.rotateAndUpdateCache(delta_i1, false, context.endEffectorInformation());
      if (viz != null){
        viz.clearFigures();
        viz.structureCopies.get(0).get(i + 1).setPosition(j_i1.node().position().get());
        viz.addFrame("Rotate J_i1 to maintain previous orientation"); //Just add the initial image
      }
    }
  }
}
