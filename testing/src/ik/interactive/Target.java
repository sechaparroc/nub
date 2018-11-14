package ik.interactive;

import frames.core.Frame;
import frames.core.Graph;
import frames.core.Interpolator;
import frames.primitives.Vector;
import frames.processing.Scene;
import frames.processing.Shape;
import ik.common.Joint;
import processing.core.PApplet;
import processing.core.PConstants;
import processing.core.PShape;

import java.util.ArrayList;
import java.util.List;

public class Target extends Shape {
    //TODO : Reduce code Duplicaion by means of Abstract Class
    //TODO : Improve multiple translation
    protected static List<Target> _selectedTargets = new ArrayList<Target>();
    protected Interpolator _interpolator;
    protected List<KeyFrame> _path = new ArrayList<KeyFrame>();
    protected PShape _redBall;
    protected Vector _desiredTranslation;


    public Target(Scene scene, Frame frame) {
        super(scene);
        _redBall = scene.is3D() ? scene.frontBuffer().createShape(PConstants.SPHERE, ((Joint) scene.trackedFrame()).radius() * 2f) :
                scene.frontBuffer().createShape(PConstants.ELLIPSE, 0,0, ((Joint) scene.trackedFrame()).radius() * 4f, ((Joint) scene.trackedFrame()).radius() * 4f);
        _redBall.setStroke(false);
        _redBall.setFill(scene.pApplet().color(255, 0, 0));

        _interpolator = new Interpolator(this);
        setGraphics(_redBall);
        setReference(scene.trackedFrame().reference());
        setPosition(frame.position());
        setOrientation(frame.orientation());
    }

    public void drawPath(){
        ((Scene) _graph).drawPath(_interpolator, 5);
    }

    @Override
    public void interact(Object... gesture) {
        String command = (String) gesture[0];
        if (command.matches("KeepSelected")) {
            if(!_selectedTargets.contains(this)){
                _redBall.setFill(graph().pApplet().color(0,255,0));
                _selectedTargets.add(this);
            }
            else{
                _redBall.setFill(graph().pApplet().color(255,0,0));
                _selectedTargets.remove(this);
            }
        } else if(command.matches("Add")){
            if(_desiredTranslation != null) {
                if(!_path.isEmpty())removeKeyFrame(_path.get(0));
                KeyFrame frame = new KeyFrame(this);
                frame.setTranslation(frame.translateDesired());
                addKeyFrame(frame, 0);
            }
            _desiredTranslation = null;
        } else if(command.matches("OnAdding")){
            _desiredTranslation = translateDesired();
        } else if(command.matches("Reset")){
            _desiredTranslation = null;
        }
    }

    public static List<Target> selectedTargets(){
        return _selectedTargets;
    }

    public static void multipleTranslate(){
        for(Target target : _selectedTargets){
            if(target.graph().defaultFrame() != target)
                target.graph().translate(target);
        }
    }

    public static void clearSelectedTargets(){
        for(int i = _selectedTargets.size()-1; i >= 0; i--){
            _selectedTargets.get(i).interact("KeepSelected");
        }
    }

    public void updateInterpolator(){
        _interpolator.clear();
        for(KeyFrame frame : _path){
            _interpolator.addKeyFrame(frame);
            //_interpolator.addKeyFrame(frame, frame.time);
        }
    }

    public void addKeyFrame(KeyFrame frame, int index){
        _path.add(index, frame);
        updateInterpolator();
    }

    public void addKeyFrame(KeyFrame frame, KeyFrame adjacent, boolean left){
        int index =  _path.indexOf(adjacent);
        if(!left) index = index + 1;
        addKeyFrame(frame, index);
    }

    public void removeKeyFrame(KeyFrame frame){
        _path.remove(frame);
        updateInterpolator();
    }

    //------------------------------------
    //Interactive actions - same method found in Graph Class
    public Vector translateDesired(){
        Scene scene = (Scene) _graph;
        PApplet pApplet = scene.pApplet();
        float dx = pApplet.mouseX - scene.screenLocation(position()).x();
        float dy = pApplet.mouseY - scene.screenLocation(position()).y();

        dy = scene.isRightHanded() ? -dy : dy;
        if(scene.type() == Graph.Type.PERSPECTIVE){
            float k = (float) Math.tan(scene.fieldOfView() / 2.0f) * Math.abs(
                    scene.eye().location(scene.isEye(this) ? scene.anchor() : this.position())._vector[2] * scene.eye().magnitude());
            dx *= 2.0 * k / scene.height();
            dy *= 2.0 * k / scene.height();
        }
        else {
            float[] wh = scene.boundaryWidthHeight();
            dx *= 2.0 * wh[0] / scene.width();
            dy *= 2.0 * wh[1] / scene.height();
        }
        Vector eyeVector = new Vector(dx / scene.eye().magnitude(), dy / scene.eye().magnitude(), 0);
        return this.reference() == null ? scene.eye().worldDisplacement(eyeVector) : this.reference().displacement(eyeVector, scene.eye());
    }
}
