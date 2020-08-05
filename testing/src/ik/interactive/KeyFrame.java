package ik.interactive;

import nub.core.Graph;
import nub.core.Node;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;


public class KeyFrame extends Node {
  Target _target;
  protected float time; //TODO Use time attribute
  protected Vector _desiredTranslation;

  public KeyFrame(Target target) {
    super();
    _target = target;
  }

  public KeyFrame(Target target, Vector translation, Quaternion rotation) {
    super(translation, rotation, 1);
    _target = target;
  }

  @Override
  public void interact(Object... gesture) {
    String command = (String) gesture[0];
    if (command.matches("Add")) {
      if (_desiredTranslation != null) {
        add((boolean) gesture[3], (Scene) gesture[1], (Vector) gesture[2]);
      }
      _desiredTranslation = null;
    } else if (command.matches("OnAdding")) {
      _desiredTranslation = translateDesired((Scene) gesture[1], (Vector) gesture[2]);
    } else if (command.matches("Reset")) {
      _desiredTranslation = null;
    } else if (command.matches("Remove")) {
      remove();
    }
  }

  public void add(boolean left, Scene scene, Vector mouse) {
    KeyFrame frame = new KeyFrame(this._target);
    frame.setTranslation(this.worldLocation(this.translateDesired(scene, mouse)));
    _target.addKeyFrame(frame, this, left);
  }

  public void remove() {
    _target.removeKeyFrame(this);
    Graph.prune(this);
  }

  protected Vector translateDesired(Scene scene, Vector point) {
    Vector delta = Vector.subtract(point, scene.screenLocation(position()));
    delta.setZ(0);
    Vector vector = scene.displacement(delta, this);
    return reference() == null ? worldDisplacement(vector) : reference().displacement(vector, this);
  }
}
