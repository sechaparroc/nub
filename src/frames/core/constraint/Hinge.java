/****************************************************************************************
 * frames
 * Copyright (c) 2018 National University of Colombia, https://visualcomputing.github.io/
 * @author Sebastian Chaparro, https://github.com/sechaparroc
 * @author Jean Pierre Charalambos, https://github.com/VisualComputing
 *
 * All rights reserved. A 2D or 3D scene graph library providing eye, input and timing
 * handling to a third party (real or non-real time) renderer. Released under the terms
 * of the GPL v3.0 which is available at http://www.gnu.org/licenses/gpl.html
 ****************************************************************************************/

package frames.core.constraint;

//TODO: CHECK FORWARD STEP WITH HINGE 3D

import frames.core.Frame;
import frames.primitives.Quaternion;
import frames.primitives.Vector;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.min;

/**
 * A Frame is constrained to disable translation and
 * allow 1-DOF rotation limiting Rotation by defining an
 * Axis (according to Local Frame Coordinates) a Rest Rotation
 * and upper and lower bounds with values between 0 and PI
 */
public class Hinge extends Constraint {
  /*
  With this Kind of Constraint no Translation is allowed
  * and the rotation depends on 2 angles this kind of constraint always
  * look for the reference frame (local constraint), if no initial position is
  * set Identity is assumed as rest position
  * */
  protected float _max;
  protected float _min;
  protected Quaternion _restRotation = new Quaternion();
  protected Quaternion _idleRotation = new Quaternion();
  protected Quaternion _orientation = new Quaternion();


  public Hinge(float min, float max, Quaternion rotation, Vector up, Vector twist) {
    _min = min;
    _max = max;
    setRestRotation(rotation, up, twist);
  }

  /**
   * reference is a Quaternion that will be aligned to point to the given Basis Vectors
   * result will be stored on restRotation.
   * twist and up axis are defined locally on reference rotation
   */
  public void setRestRotation(Quaternion reference, Vector up, Vector twist) {
    _orientation = reference.get();
    _idleRotation = reference.get();
    //Align y-Axis with up vector
    System.out.println("up : " + up);
    System.out.println("twist : " + twist);
    Quaternion delta = new Quaternion(new Vector(0, 1, 0), up);
    System.out.println("d1 :  " + delta.axis() + " " + delta.angle());
    Vector tw = delta.inverseRotate(twist);
    //Align z-Axis with twist vector
    delta.compose(new Quaternion(new Vector(0, 0, 1), tw));
    System.out.println("d2 :  " + delta.axis() + " " + delta.angle());

    _orientation.compose(delta); // orientation = idle * rest
    System.out.println("Or :  " + orientation().axis() + " " + Math.toDegrees(orientation().angle()));
    System.out.println("up :  " + orientation().rotate(new Vector(0,1,0)));
    System.out.println("tw :  " + orientation().rotate(new Vector(0,0,1)));

    _restRotation = delta;
  }

  public Quaternion orientation(){
    return _orientation;
  }

  public float maxAngle() {
    return _max;
  }

  public void setMaxAngle(float max) {
    this._max = max;
  }

  public float minAngle() {
    return _min;
  }

  public void setMinAngle(float min) {
    this._min = min;
  }

  @Override
  public Quaternion constrainRotation(Quaternion rotation, Frame frame) {
    Quaternion desired = Quaternion.compose(frame.rotation(), rotation); //w.r.t reference

    desired = Quaternion.compose(_orientation.inverse(), desired);
    desired = Quaternion.compose(desired, _restRotation);
    System.out.println("desired : " + desired.axis() + "  " + Math.toDegrees(desired.angle()));
    //desired = Quaternion.compose(_restRotation.inverse(), desired); //w.r.t rest
    System.out.println("up :  " + orientation().rotate(new Vector(0,1,0)));
    System.out.println("tw :  " + orientation().rotate(new Vector(0,0,1)));
    System.out.println("idle : " + _idleRotation.axis() + "  " + Math.toDegrees(_idleRotation.angle()));
    System.out.println("desired : " + desired.axis() + "  " + Math.toDegrees(desired.angle()));
    System.out.println("des up :  " + desired.rotate(new Vector(0,1,0)));
    System.out.println("des tw :  " + desired.rotate(new Vector(0,0,1)));

    Vector rotationAxis = new Vector(desired._quaternion[0], desired._quaternion[1], desired._quaternion[2]);
    rotationAxis = Vector.projectVectorOnAxis(rotationAxis, new Vector(0,0,1));
    //Get rotation component on Axis direction w.r.t reference
    Quaternion rotationTwist= new Quaternion(rotationAxis.x(), rotationAxis.y(), rotationAxis.z(), desired.w());
    float deltaAngle = rotationTwist.angle();
    if (rotationAxis.dot(new Vector(0,0,1)) < 0) deltaAngle *= -1;
    //get signed distance
    float change = deltaAngle;
    if(-_min > change || change > _max ){
      change = change < 0 ? (float) (change + 2*Math.PI) : change;
      change = change - _max < (float) (-_min + 2*Math.PI) - change ? _max : -_min;
    }
    System.out.println("Change : " + change);

    //apply constrained rotation orientation * constrained_change = frame * rot
    Quaternion rot = Quaternion.compose(frame.rotation().inverse(),
            Quaternion.compose(_orientation, Quaternion.compose(new Quaternion(new Vector(0,0,1), change), _restRotation.inverse())));
    return rot;
  }
  @Override
  public Vector constrainTranslation(Vector translation, Frame frame) {
    return new Vector();
  }
}
