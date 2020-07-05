package nub.ik.solver.trik.heuristic;

import nub.core.constraint.Hinge;
import nub.ik.solver.trik.NodeInformation;
import nub.ik.solver.trik.Context;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

public class TRIK extends Heuristic {
    /**
     * The idea of this heuristics is to Apply a local action similar to FABRIK
     */

    public TRIK(Context context) {
        super(context);
    }


    @Override
    public void prepare() {
        //Update cache of usable chain
        NodeInformation._updateCache(_context.usableChainInformation());
    }

    @Override
    public void applyActions(int i) {
        applyTRIK(this, i);
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

    public static Quaternion findReachStep(NodeInformation j_i, NodeInformation j_i1, Vector endEffector, Vector target, boolean checkHinge){
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

    public static void applyTRIK(Heuristic heuristic, int i){
        Context context = heuristic._context;
        Vector target = context.worldTarget().position();
        NodeInformation j_i = context.usableChainInformation().get(i);
        Vector eff_wrt_j_i = j_i.locationWithCache(context.endEffectorInformation().positionCache());
        Vector target_wrt_j_i = j_i.locationWithCache(target);
        NodeInformation j_i1 = context.usableChainInformation().get(i + 1);
        //find the rotation that reaches the target while keeps the orientation
        Quaternion delta_i = findReachStep(j_i, j_i1, eff_wrt_j_i, target_wrt_j_i, true);
        if(i == context.endEffectorId() - 1){
            //check just a joint instead of a couple of joints
            j_i.rotateAndUpdateCache(delta_i, false, context.endEffectorInformation());
        } else {
            //apply fix step on i_1
            Quaternion q_i1 = j_i1.node().rotation();
            Quaternion q_i1_inv = q_i1.inverse();
            Quaternion delta_i1 = findNextFixStep(j_i1, q_i1, q_i1_inv, delta_i);
            //Perhaps in this step it is not required to check the constraint
            Quaternion delta_i_hat = findCurrentFixStep(j_i, q_i1, q_i1_inv, delta_i1, true);
            //Apply the found solutions
            j_i.rotateAndUpdateCache(delta_i_hat, false, context.endEffectorInformation());
            j_i1.updateCacheUsingReference();
            j_i1.rotateAndUpdateCache(delta_i1, false, context.endEffectorInformation());
        }
    }
}
