package ik.interactive;

import nub.core.Graph;
import nub.core.Interpolator;
import nub.core.Node;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.timing.TimingTask;
import processing.core.PConstants;
import processing.core.PShape;

import java.util.ArrayList;
import java.util.List;

public class Target extends Node {
    //TODO : Reduce code Duplicaion by means of Abstract Class
    //TODO : Improve multiple translation
    protected static List<Target> _selectedTargets = new ArrayList<Target>();
    protected Interpolator _interpolator;
    protected List<KeyFrame> _path = new ArrayList<KeyFrame>();
    protected PShape _redBall;
    protected Vector _desiredTranslation;
    protected ArrayList<Vector> _last = new ArrayList<>();

    public Interpolator interpolator(){
        return _interpolator;
    }

    public Target(Scene scene, float radius) {
        super(scene);
        _redBall = scene.is3D() ? scene.context().createShape(PConstants.SPHERE, radius * 2f) :
                scene.context().createShape(PConstants.ELLIPSE, 0,0, radius * 4f, radius * 4f);
        _redBall.setStroke(false);
        _redBall.setFill(scene.pApplet().color(255, 0, 0));

        _interpolator = new Interpolator(this);
        //setShape(_redBall);
        setPickingThreshold(0);

        Target t = this;
        TimingTask task = new TimingTask() {
            @Override
            public void execute() {
                _last.add(t.position());
                while(_last.size() > 50) _last.remove(0);
            }
        };
        scene.registerTask(task);
        task.run(150);
    }

    public Target(Scene scene, float radius, Node frame) {
        this(scene, radius);
        setPosition(frame.position());
        setOrientation(frame.orientation());
    }

    @Override
    public void graphics(processing.core.PGraphics pGraphics){
        pGraphics.hint(PConstants.DISABLE_DEPTH_TEST);
        pGraphics.shape(_redBall);
        pGraphics.hint(PConstants.ENABLE_DEPTH_TEST);
    }


    public ArrayList<Vector> last(){ return _last; }
    public void drawPath(){
        ((Scene) _graph).drawPath(_interpolator, 5);
    }

    @Override
    public void interact(Object... gesture) {
        String command = (String) gesture[0];
        if (command.matches("KeepSelected")) {
            if(!_selectedTargets.contains(this)){
                _redBall.setFill(((Scene)graph()).pApplet().color(0,255,0));
                _selectedTargets.add(this);
            }
            else{
                _redBall.setFill(((Scene)graph()).pApplet().color(255,0,0));
                _selectedTargets.remove(this);
            }
        } else if(command.matches("Add")){
            if(_desiredTranslation != null) {
                if(!_path.isEmpty())removeKeyFrame(_path.get(0));
                KeyFrame frame = new KeyFrame(this);
                frame.translate(this.worldLocation(this.translateDesired((Vector) gesture[1])));
                addKeyFrame(frame, 0);
            }
            _desiredTranslation = null;
        } else if(command.matches("OnAdding")){
            _desiredTranslation = translateDesired((Vector) gesture[1]);
        } else if(command.matches("Reset")){
            _desiredTranslation = null;
        } else if(command.matches("AddCurve")){
            FitCurve fitCurve = (FitCurve) gesture[1];
            fitCurve.getCatmullRomCurve((Scene)this.graph(), ((Scene)this.graph()).screenLocation(this.position()).z());
            _interpolator = fitCurve._interpolator;
            _interpolator.setNode(this);
            _interpolator.setSpeed(5f);
            _desiredTranslation = null;
        }
    }

    public static List<Target> selectedTargets(){
        return _selectedTargets;
    }

    public static void multipleTranslate(){
        for(Target target : _selectedTargets){
            if(target.graph().defaultNode() != target)
                ((Scene)target.graph()).translate(target);
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
    //Interactive actions - same method found in Graph Class (duplicated cause of visibility)
    protected Vector _translateDesired(float dx, float dy, float dz, int zMax, Node node) {
        Scene scene = (Scene) _graph;
        if (scene.is2D() && dz != 0) {
            System.out.println("Warning: graph is 2D. Z-translation reset");
            dz = 0;
        }
        dx = scene.isEye(node) ? -dx : dx;
        dy = scene.isRightHanded() ^ scene.isEye(node) ? -dy : dy;
        dz = scene.isEye(node) ? dz : -dz;
        // Scale to fit the screen relative vector displacement
        if (scene.type() == Graph.Type.PERSPECTIVE) {
            float k = (float) Math.tan(scene.fov() / 2.0f) * Math.abs(
                    scene.eye().location(scene.isEye(node) ? scene.anchor() : node.position())._vector[2] * scene.eye().magnitude());
            //TODO check me weird to find height instead of width working (may it has to do with fov?)
            dx *= 2.0 * k / (scene.height() * scene.eye().magnitude());
            dy *= 2.0 * k / (scene.height() *scene. eye().magnitude());
        }
        // this expresses the dz coordinate in world units:
        //Vector eyeVector = new Vector(dx, dy, dz / eye().magnitude());
        Vector eyeVector = new Vector(dx, dy, dz * 2 * scene.radius() / zMax);
        return node.reference() == null ? scene.eye().worldDisplacement(eyeVector) : node.reference().displacement(eyeVector, scene.eye());
    }


    public Vector translateDesired(Vector mouse){
        Scene scene = (Scene) _graph;
        float dx = mouse.x() - scene.screenLocation(position()).x();
        float dy = mouse.y() - scene.screenLocation(position()).y();
        return _translateDesired(dx, dy, 0, Math.min(scene.width(), scene.height()), this);
    }
}
