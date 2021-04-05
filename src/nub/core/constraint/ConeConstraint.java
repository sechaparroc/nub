/***************************************************************************************
 * nub
 * Copyright (c) 2019-2020 Universidad Nacional de Colombia
 * @author Sebastian Chaparro Cuevas, https://github.com/VisualComputing
 * @author Jean Pierre Charalambos, https://github.com/VisualComputing
 *
 * All rights reserved. A simple, expressive, language-agnostic, and extensible visual
 * computing library, featuring interaction, visualization and animation frameworks and
 * supporting advanced (onscreen/offscreen) (real/non-real time) rendering techniques.
 * Released under the terms of the GPLv3, refer to: http://www.gnu.org/licenses/gpl.html
 ***************************************************************************************/

package nub.core.constraint;

import nub.core.Node;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import processing.core.PGraphics;

/**
 * A Cone constraint allows 3-DOF rotational motion and by default no translation is
 * allowed (0-DOF), however, translation motion could be modified using an {@link AxisPlaneConstraint}.
 *
 * To define a Cone constraint you must provide a Reference Quaternion, a Twist axis of rotation, an Up Vector orthogonal to this axis
 * (in order to draw the constraint properly @see {@link nub.processing.Scene#drawConstraint(PGraphics, Node)}) and the boundaries of the
 * constraint.
 *
 * As the reference rotation give us the transformation against which any further rotation must be compared, we will call it the
 * idle rotation.
 *
 * The Up and Twist vector defines a unique rotation w.r.t the idle rotation that we named rest rotation i.e. the transformation
 * required to align the Z and Y Axes of the idle rotation with the Twist and Up vector respectively:
 *    Twist = rest^-1 * Z * rest
 *    Up = rest^-1 * Y * rest
 *
 * The composition transformation of the idle rotation and the rest rotation gives the orientation of the constraint:
 *    orientation = idle * rest
 *
 * It is possible to include an Offset rotation in cases in which the initial rotation to compare does not corresponds with the
 * idle rotation.
 *
 * If a rotation rot is applied to the node rotation the following equations are satisfied:
 *    (1)  node.rotation() * rot * offset = idle * alpha
 *    (2)  idle * alpha * rest = idle * rest * beta
 *    (3)  beta = rest^-1 * alpha * rest
 *    (4)  beta = rest^-1 * idle^-1 * node.rotation() * rot * offset * rest
 *    (5)  beta = orientation^-1 * node.rotation() * rot * offset * rest
 *
 * Equation (5) is required to compare if the rotation applied to the node satisfies the constraint, and if that is not the case
 * a clamping action must be performed.
 */

public abstract class ConeConstraint extends Constraint {
  protected Quaternion _restRotation = new Quaternion();
  protected Quaternion _idleRotation = new Quaternion();
  protected Quaternion _orientation = new Quaternion();
  protected Quaternion _offset = new Quaternion();
  protected float _min = (float) Math.PI, _max = (float) Math.PI;

  protected AxisPlaneConstraint.Type transConstraintType = AxisPlaneConstraint.Type.FORBIDDEN;
  protected Vector transConstraintDir = new Vector();


  public Quaternion restRotation() {
    return _restRotation;
  }

  public Quaternion idleRotation() {
    return _idleRotation;
  }

  public Quaternion offset() {
    return _offset;
  }

  public Quaternion orientation() {
    return _orientation;
  }

  public void setRestRotation(Quaternion restRotation) {
    this._restRotation = restRotation.get();
  }

  /**
   * Defines the orientational parameters of this constraint:
   * @param reference Is the reference rotation in which the Up and Twist vectors will be defined. Usually this quaternion
   *                  is the same as the (@see {@link Node#rotation()}) to constraint.
   * @param up  Represents a Orthogonal vector to the axis of rotation useful to draw properly a Hinge constraint in a
   *            scene (@see {@link nub.processing.Scene#drawConstraint(PGraphics, Node)}).
   * @param twist Represents the Axis of rotation of this 1-DOF rotational constraint.
   */
  public void setRestRotation(Quaternion reference, Vector up, Vector twist, Vector offset) {
    _orientation = reference.get();
    _idleRotation = reference.get();
    //Align z-Axis with twist vector around new up Vector
    Quaternion delta = new Quaternion(new Vector(0, 0, 1), twist);
    Vector tw = delta.inverseRotate(up);
    //Align y-Axis with up vector
    //Assume that up and twist are orthogonal
    float angle = Vector.angleBetween(tw, new Vector(0, 1, 0));
    if (Vector.cross(new Vector(0, 1, 0), tw, null).dot(new Vector(0, 0, 1)) < 0)
      angle *= -1;
    delta.compose(new Quaternion(new Vector(0, 0, 1), angle));
    delta.normalize();
    _orientation.compose(delta); // orientation = idle * rest
    _offset = new Quaternion(twist, offset); // TODO : check offset
    _restRotation = delta;
  }

  public void setRotations(Quaternion reference, Quaternion rest) {
    _idleRotation = reference.get();
    _restRotation = rest.get();
    _orientation = Quaternion.compose(reference, rest);
  }


  public void setTwistLimits(float min, float max) {
    _min = min;
    _max = max;
  }

  public float minTwistAngle() {
    return _min;
  }

  public float maxTwistAngle() {
    return _max;
  }

  public void setRestRotation(Quaternion reference, Vector up, Vector twist) {
    setRestRotation(reference, up, twist, twist);
  }

  public abstract Vector apply(Vector target);

  @Override
  public Vector constrainTranslation(Vector translation, Node node) {
    Vector res = new Vector(translation._vector[0], translation._vector[1], translation._vector[2]);
    Vector proj;
    switch (translationConstraintType()) {
      case FREE:
        break;
      case PLANE:
        proj = node.rotation().rotate(translationConstraintDirection());
        // proj = node._localInverseTransformOf(translationConstraintDirection());
        res = Vector.projectVectorOnPlane(translation, proj);
        break;
      case AXIS:
        proj = node.rotation().rotate(translationConstraintDirection());
        // proj = node._localInverseTransformOf(translationConstraintDirection());
        res = Vector.projectVectorOnAxis(translation, proj);
        break;
      case FORBIDDEN:
        res = new Vector(0.0f, 0.0f, 0.0f);
        break;
    }
    return res;
  }

  public AxisPlaneConstraint.Type translationConstraintType() {
    return transConstraintType;
  }

  public Vector translationConstraintDirection() {
    return transConstraintDir;
  }

  public void setTranslationConstraint(AxisPlaneConstraint.Type type, Vector direction) {
    setTranslationConstraintType(type);
    setTranslationConstraintDirection(direction);
  }

  public void setTranslationConstraintType(AxisPlaneConstraint.Type type) {
    transConstraintType = type;
  }


  public void setTranslationConstraintDirection(Vector direction) {
    if ((translationConstraintType() != AxisPlaneConstraint.Type.FREE) && (translationConstraintType()
        != AxisPlaneConstraint.Type.FORBIDDEN)) {
      float norm = direction.magnitude();
      if (norm == 0) {
        System.out
            .println("Warning: AxisPlaneConstraint.setTranslationConstraintDir: null vector for translation constraint");
        transConstraintType = AxisPlaneConstraint.Type.FREE;
      } else
        transConstraintDir = Vector.multiply(direction, (1.0f / norm));
    }
  }

  //Convenient methods to apply stereographic projection.
  /*more info at http://www.ams.org/publicoutreach/feature-column/fc-2014-02*/
  protected Vector _stereographicProjection(Vector v){float d = (1 + v.z());
    float X = 2 * v.x() / d;
    float Y = 2 * v.y() / d;
    return new Vector(X, Y, 0);
  }

  protected Vector _inverseStereographicProjection(Vector v){
    float s = 4 / (v.x() * v.x() + v.y() * v.y() + 4);
    return new Vector(s * v.x(), s * v.y(), 2 * s - 1);
  }

  protected static void decomposeQuaternion(Quaternion q, Vector twistAxis, Quaternion twist, Quaternion swing){
    Vector r = new Vector(q.x(), q.y(), q.z());
    Vector p = Vector.projectVectorOnAxis(r, twistAxis);
    twist.set(new Quaternion(p.x(), p.y(), p.z(), q.w()));
    swing.set(Quaternion.compose(q, twist.inverse()));
    swing.normalize();
  }

  //Define how much idle rotation must change according to this rules:
  //(1) idle * idle_change = node * rotation * offset
  //(2) idle * idle_change * rest_rotation = idle * rest_rotation * rest_change
  //(3) offset is applied w.r.t idle space

  protected Quaternion fromReferenceToRest(Quaternion rotation){
      Quaternion delta_idle = Quaternion.compose(_idleRotation.inverse(), rotation);
      delta_idle.normalize();
      delta_idle.compose(_offset);
      delta_idle.normalize();
      Quaternion delta_rest = Quaternion.compose(_restRotation.inverse(), delta_idle);
      delta_rest.normalize();
      delta_rest.compose(_restRotation);
      delta_rest.normalize();
      //Choose the closest path
      if (Quaternion.dot(delta_rest, _restRotation) < 0) {
          delta_rest.negate();
      }
      return delta_rest;
  }

  protected Quaternion fromRestToNode(Quaternion node, Quaternion rotation){
      Quaternion delta = Quaternion.compose(node.inverse(), _idleRotation);
      delta.normalize();
      delta.compose(_restRotation);
      delta.normalize();
      delta.compose(rotation);
      delta.normalize();
      delta.compose(_restRotation.inverse());
      delta.normalize();
      delta.compose(_offset.inverse());
      delta.normalize();
      return delta;
  }

  public Quaternion constrainRotation(Quaternion rotation, Node node){
      //1. Find the rotation in terms of rest coordinate frame
      Vector z_axis = new Vector(0,0,1);
      Quaternion desired = Quaternion.compose(node.rotation(), rotation);
      desired.normalize();
      Quaternion delta_rest = fromReferenceToRest(desired);
      if (Quaternion.dot(delta_rest, _restRotation) < 0) {
          delta_rest.negate();
      }

      //3. Constraint swing - Here we must work on rest space
      Vector new_twist_dir = delta_rest.rotate(z_axis);
      Vector new_twist_constrained_dir = apply(new_twist_dir);
      Quaternion swing_wrt_rest_constrained = new Quaternion(z_axis, new_twist_constrained_dir);
      //4. fix twist
      Quaternion diff = Quaternion.compose(delta_rest, swing_wrt_rest_constrained.inverse());
      //Get twist component
      Quaternion twist = new Quaternion();
      twist.normalize();
      decomposeQuaternion(diff, new_twist_constrained_dir, twist, new Quaternion());

      float twistAngle = twist.angle();
      Vector rotationAxis = twist.axis();
      if (rotationAxis.dot(new_twist_constrained_dir) < 0) twistAngle = -twistAngle;

      if(Math.abs(twistAngle) > Math.PI) //express as shortest quaternion
          twistAngle = (float) (twistAngle - Math.signum(twistAngle) * 2 * Math.PI);

      if(Math.abs(twistAngle) > 0.005) {
          //Check that twist angle is in range [-min, max]
          if (-_min > twistAngle || twistAngle > _max) {
              twistAngle = twistAngle < 0 ? (float) (twistAngle + 2 * Math.PI) : twistAngle;
              twistAngle = twistAngle - _max < (float) (-_min + 2 * Math.PI) - twistAngle ? _max : -_min;
          }
          twist = new Quaternion(new_twist_constrained_dir, twistAngle); //w.r.t rest
      } else{
          twist = new Quaternion();
      }

      Quaternion constrained = Quaternion.compose(twist, swing_wrt_rest_constrained);
      constrained.normalize();
      //5. Find the rotation in terms of node
      return fromRestToNode(node.rotation(), constrained);
  }
}

