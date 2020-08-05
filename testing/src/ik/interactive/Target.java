package ik.interactive;

import nub.core.Interpolator;
import nub.core.Node;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.processing.TimingTask;
import processing.core.PConstants;

import java.util.ArrayList;
import java.util.List;

public class Target extends Node {
  protected static List<Target> _selectedTargets = new ArrayList<Target>();
  protected Interpolator _interpolator;
  protected List<KeyFrame> _path = new ArrayList<KeyFrame>();
  protected Vector _desiredTranslation;
  protected ArrayList<Vector> _last = new ArrayList<>();
  protected float _radius;
  protected boolean _selected;

  public Interpolator interpolator() {
    return _interpolator;
  }

  public Target(float radius) {
    super();
    _radius = radius;
    setShape(pg ->{
      pg.hint(PConstants.DISABLE_DEPTH_TEST);
      pg.pushStyle();
      pg.noStroke();
      if(_selected) pg.fill(0,255,0);
      else pg.fill(255,0,0);
      if(pg.is2D()) pg.ellipse(0,0, _radius * 2, _radius * 2);
      else pg.sphere(_radius * 2);
      pg.popStyle();
      pg.hint(PConstants.ENABLE_DEPTH_TEST);
    });
    enableHint(AXES, _radius * 1.5f);

    _interpolator = new Interpolator(this);
    setBullsEyeSize(0);

    Target t = this;
    TimingTask task = new TimingTask() {
      @Override
      public void execute() {
        _last.add(t.position());
        while (_last.size() > 50) _last.remove(0);
      }
    };
    task.run(150);
  }

  public Target(float radius, Vector position, Quaternion orientation) {
    this(radius);
    setPosition(position);
    setOrientation(orientation);
  }



  public ArrayList<Vector> last() {
    return _last;
  }

  public void drawPath() {
    _interpolator.configHint(Interpolator.SPLINE, Scene.pApplet.color(255, 255, 0));
  }

  @Override
  public void interact(Object... gesture) {
    String command = (String) gesture[0];
    if (command.matches("KeepSelected")) {
      if (!_selectedTargets.contains(this)) {
        _selected = true;
        _selectedTargets.add(this);
      } else {
        _selected = false;
        _selectedTargets.remove(this);
      }
    } else if (command.matches("Add")) {
      if (_desiredTranslation != null) {
        if (!_path.isEmpty()) removeKeyFrame(_path.get(0));
        KeyFrame frame = new KeyFrame(this);
        frame.translate(this.worldLocation(this.translateDesired((Scene) gesture[1], (Vector) gesture[2])));
        addKeyFrame(frame, 0);
      }
      _desiredTranslation = null;
    } else if (command.matches("OnAdding")) {
      _desiredTranslation = translateDesired((Scene) gesture[1], (Vector) gesture[2]);
    } else if (command.matches("Reset")) {
      _desiredTranslation = null;
    } else if (command.matches("AddCurve")) {
      Scene scene = (Scene) gesture[1];
      FitCurve fitCurve = (FitCurve) gesture[2];
      fitCurve.getCatmullRomCurve(scene, this, scene.screenLocation(this.position()).z());
      _interpolator = fitCurve._interpolator;
      _interpolator.setNode(this);
      _interpolator.setSpeed(5f);
      _desiredTranslation = null;
    }
  }

  public static List<Target> selectedTargets() {
    return _selectedTargets;
  }

  public static void multipleTranslate(Scene scene) {
    for (Target target : _selectedTargets) {
      if (scene.node() != target)
        scene.translateNode(target, scene.mouseDX(), scene.mouseDY(), 0, 0);
    }
  }

  public static void clearSelectedTargets() {
    for (int i = _selectedTargets.size() - 1; i >= 0; i--) {
      _selectedTargets.get(i).interact("KeepSelected");
    }
  }

  public void updateInterpolator() {
    _interpolator.clear();
    for (KeyFrame frame : _path) {
      _interpolator.addKeyFrame(frame);
      //_interpolator.addKeyFrame(frame, frame.time);
    }
  }

  public void addKeyFrame(KeyFrame frame, int index) {
    _path.add(index, frame);
    updateInterpolator();
  }

  public void addKeyFrame(KeyFrame frame, KeyFrame adjacent, boolean left) {
    int index = _path.indexOf(adjacent);
    if (!left) index = index + 1;
    addKeyFrame(frame, index);
  }

  public void removeKeyFrame(KeyFrame frame) {
    _path.remove(frame);
    updateInterpolator();
  }

  protected Vector translateDesired(Scene scene, Vector point) {
    Vector delta = Vector.subtract(point, scene.screenLocation(position()));
    delta.setZ(0);
    Vector vector = scene.displacement(delta, this);
    return reference() == null ? worldDisplacement(vector) : reference().displacement(vector, this);
  }
}
