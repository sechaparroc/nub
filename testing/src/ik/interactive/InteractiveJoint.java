package ik.interactive;

import nub.core.Graph;
import nub.core.Node;
import nub.ik.animation.Skeleton;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PConstants;

public class InteractiveJoint extends Node {
  protected Vector _desiredTranslation;

  public InteractiveJoint(boolean root, int color, float radius, boolean depth) {
    super();
    _boneColor = color;
    _boneRadius = radius;
    _boneDepth = depth;
    if(root) {
      setShape(pg -> {
        if (!_boneDepth) pg.hint(PConstants.DISABLE_DEPTH_TEST);
        pg.pushStyle();
        pg.noStroke();
        pg.fill(_boneColor);
        if (pg.is3D()) pg.sphere(_boneRadius);
        else pg.ellipse(0, 0, 2 * _boneRadius, 2 * _boneRadius);
        pg.popStyle();

        if (_desiredTranslation != null) {
          pg.pushStyle();
          pg.stroke(pg.color(0, 255, 0));
          pg.line(0, 0, 0, _desiredTranslation.x() / this.scaling(), _desiredTranslation.y() / this.scaling(), _desiredTranslation.z() / this.scaling());
          pg.popStyle();
        }

        if (!_boneDepth) pg.hint(PConstants.ENABLE_DEPTH_TEST);
      });
    } else{
      setShape(pg -> {
        if (!_boneDepth) pg.hint(PConstants.DISABLE_DEPTH_TEST);
        if (_desiredTranslation != null) {
          pg.pushStyle();
          pg.stroke(pg.color(0, 255, 0));
          pg.line(0, 0, 0, _desiredTranslation.x() / this.scaling(), _desiredTranslation.y() / this.scaling(), _desiredTranslation.z() / this.scaling());
          pg.popStyle();
        }
        if (!_boneDepth) pg.hint(PConstants.ENABLE_DEPTH_TEST);
      });
      enableHint(Node.BONE, color, radius, radius / 4f, _boneDepth);
    }
    enableHint(Node.CONSTRAINT);
  }

  public InteractiveJoint(boolean root, int color, float radius){
    this(root, color, radius, true);
  }

  public InteractiveJoint(boolean root, int color){
    this(root, color, 5, true);
  }

  public InteractiveJoint(boolean root, float radius){
    this(root, -1, radius, true);
  }

  public InteractiveJoint(int color, float radius, boolean depth){
    this(false, color, radius, depth);
  }

  public InteractiveJoint(int color, float radius){
    this(false, color, radius, true);
  }

  public InteractiveJoint(int color){
    this(false, color, 5, true);
  }


  @Override
  public void interact(Object... gesture) {
    String command = (String) gesture[0];
    if (command.matches("Add")) {
      if (_desiredTranslation != null) {
        if (gesture.length == 3)
          addChild((Scene) gesture[1], (Vector) gesture[2]);
        else
          addChild((Scene) gesture[1], (Vector) gesture[2], (Skeleton) gesture[3]);
      }
      _desiredTranslation = null;
    } else if (command.matches("OnAdding")) {
      _desiredTranslation = translateDesired((Scene) gesture[1], (Vector) gesture[2]);
    } else if (command.matches("Reset")) {
      _desiredTranslation = null;
    } else if (command.matches("Remove")) {
      if (gesture.length == 1)
        removeChild();
      else
        removeChild((Skeleton) gesture[1]);
    }
  }

  public void addChild(Scene focus, Vector mouse) {
    addChild(focus, mouse, null);
  }

  public void addChild(Scene focus, Vector mouse, Skeleton skeleton) {
    InteractiveJoint joint = new InteractiveJoint(false, Scene.pApplet.color((float) Math.random() * 255, (float) Math.random() * 255, (float) Math.random() * 255), _boneRadius, _boneDepth);
    joint.setBullsEyeSize(this.bullsEyeSize());
    if (skeleton != null) {
      skeleton.addJoint("J" + skeleton.joints().size(), joint);
    }
    joint.setReference(this);
    joint.setTranslation(joint.translateDesired(focus, mouse));
  }

  public void removeChild() {
    Graph.prune(this);
  }

  public void removeChild(Skeleton skeleton) {
    skeleton.prune(this);
  }

  public Vector desiredTranslation() {
    return _desiredTranslation;
  }

  protected Vector translateDesired(Scene scene, Vector point) {
    Vector delta = Vector.subtract(point, scene.screenLocation(position()));
    delta.setZ(0);
    Vector vector = scene.displacement(delta, this);
    return reference() == null ? worldDisplacement(vector) : reference().displacement(vector, this);
  }
}
