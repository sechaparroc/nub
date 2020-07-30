package nub.ik.solver.trik.heuristic;

import nub.core.Node;
import nub.core.constraint.Hinge;
import nub.ik.solver.trik.NodeInformation;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

public class Util {
    /**
     * This class contains convenient actions that are quite common
     * when implementing a geometric heuristic.
     * */
    public static Quaternion constraintRotation(NodeInformation j_i, Quaternion rotation){
        if (j_i.node().constraint() != null) {
            // See: https://stackoverflow.com/questions/2886606/flipping-issue-when-interpolating-rotations-using-quaternions
            if (Quaternion.dot(rotation, j_i.node().rotation()) < 0) {
                // change sign
                rotation.negate();
            }
            rotation = j_i.node().constraint().constrainRotation(rotation, j_i.node());
            rotation.normalize();
        }
        return rotation;
    }

    public static Quaternion clampRotation(Quaternion rotation, float maxAngle, float factor){
        float angle = rotation.angle();
        float angleVal = Math.abs(angle);
        float angleSign = Math.signum(angle);
        Vector axis = rotation.axis();
        if (angleVal > Math.PI) {
            axis.multiply(-1);
            angle = angleSign * (float) (2 * Math.PI - angleVal);
        }
        angle *= factor;
        if (Math.abs(angle) > maxAngle) {
            angle = angleSign * maxAngle;
        }
        return new Quaternion(axis, angle);
    }

    public static Quaternion clampRotation(Quaternion rotation, float maxAngle) {
        return clampRotation(rotation, maxAngle, 1);
    }

    protected static Quaternion clampRotation(Quaternion current, Quaternion initial, Quaternion end, float maxAngle, float factor) {
        Quaternion diff = Quaternion.compose(initial.inverse(), end);
        diff.normalize();
        float angle = diff.angle();
        float angleVal = Math.abs(angle);
        float angleSign = Math.signum(angle);
        Vector axis = diff.axis();
        if (Math.abs(angle) > Math.PI) {
            axis.multiply(-1);
            angle = angleSign * (float) (2 * Math.PI - angleVal);
        }
        angle *= factor;
        if (Math.abs(angle) > maxAngle) {
            angle = angleSign * maxAngle;
        }
        diff = new Quaternion(axis, angle);
        Quaternion delta = Quaternion.compose(current.inverse(), initial);
        delta.compose(diff);
        delta.normalize();
        return delta;
    }

    protected static Quaternion clampRotation(Quaternion current, Quaternion initial, Quaternion end, float maxAngle) {
        return clampRotation(current, initial, end, maxAngle, 1);
    }

    //Set the position of the end effector and the target to lie in the plane defined by the Hinge twist axis
    public static void checkHinge(NodeInformation j_i, Vector endEffector, Vector target){
        Hinge h = (Hinge) j_i.node().constraint();
        Vector normal = h.orientation().rotate(new Vector(0, 0, 1));
        normal = j_i.node().rotation().inverse().rotate(normal);
        normal.normalize();
        //Project end effector & target on the plane of rot
        endEffector.set(Vector.projectVectorOnPlane(endEffector, normal));
        target.set(Vector.projectVectorOnPlane(target, normal));
    }

    //Try to approach to target final position by means of twisting
    public static Quaternion findCCDTwist(NodeInformation j_i1, Vector endEffector, Vector target) {
        Vector j_i_to_eff_proj, j_i_to_target_proj;
        Vector tw = j_i1.node().translation(); // w.r.t j_i
        //Project the given vectors in the plane given by twist axis
        try {
            j_i_to_target_proj = Vector.projectVectorOnPlane(target, tw);
            j_i_to_eff_proj = Vector.projectVectorOnPlane(endEffector, tw);
        } catch (Exception e) {
            return new Quaternion(tw, 0);
        }

        //Perform this operation only when Projected Vectors have not a despicable length
        if (j_i_to_target_proj.magnitude() < 0.3 * target.magnitude() && j_i_to_eff_proj.magnitude() < 0.3 * endEffector.magnitude()) {
            return new Quaternion(tw, 0);
        }

        //Find the angle between projected vectors
        float angle = Vector.angleBetween(j_i_to_eff_proj, j_i_to_target_proj);
        //clamp angle
        if (Vector.cross(j_i_to_eff_proj, j_i_to_target_proj, null).dot(tw) < 0)
            angle *= -1;
        Quaternion twist = new Quaternion(tw, angle);
        return twist;
    }

    public static float findMaxDirectionalAngle(NodeInformation j_i, NodeInformation endEffector, Node target, float searchingAreaRadius){
        float max_dist = searchingAreaRadius;
        float radius = Vector.distance(endEffector.positionCache(), j_i.positionCache());
        float rado = Vector.distance(target.position(), j_i.positionCache());
        float p = Math.min(rado, radius) / Math.max(rado, radius);
        float norm_error = Vector.distance(endEffector.positionCache(), target.position());
        norm_error /= searchingAreaRadius;

        //max_dist = searchingAreaRadius * (1 + p);

        //find max theta allowed
       return  (float) Math.acos(Math.max(Math.min(1 - (max_dist * max_dist) / (2 * radius * radius), 1), -1));
    }



}
