package intellij.simpleModel.interactiveAreas;

import intellij.simpleModel.gizmos.*;
import nub.core.Node;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.core.PConstants;
import processing.core.PShape;
import processing.event.MouseEvent;

import java.util.ArrayList;

import static processing.core.PApplet.*;

public class MainArea implements InteractiveArea{

    //Defines the action that must be performed when a Component (Node) is Clicked
    public enum Mode { SHAPESELECTION, SCALING_GIZMO, TRANSLATION_GIZMO, ROTATION_GIZMO }

    //Defines a main scene and two auxiliar scenes corresponding to the XZ Plane and the YZ plane
    public Scene scene, sceneXZ, sceneYZ;
    Scene scenes[];

    public Node reference;
    public ShapeSelectorArea shapeSelectorArea;

    //Click mode
    public Mode mode = Mode.SHAPESELECTION;
    public ArrayList<Component> components = new ArrayList<Component>();

    //Define Gizmos
    public ScaleGizmo scaleGizmo;
    public TranslateGizmo translateGizmo;
    public RotateGizmo rotateGizmo;

    int colors [];
    int x, y, w, h;

    public MainArea(PApplet pApplet, ShapeSelectorArea shapeSelectorArea, int x, int y, int w, int h){
        this.x = x;
        this.y = y;
        this.w = w;
        this.h = h;
        this.shapeSelectorArea = shapeSelectorArea;
        scene = new Scene(pApplet.createGraphics(2 * w / 3, h, P3D));
        sceneXZ = new Scene(pApplet.createGraphics(w / 3, h/2, P3D));
        sceneYZ = new Scene(pApplet.createGraphics(w / 3, h/2, P3D));
        scenes = new Scene[]{scene, sceneXZ, sceneYZ};

        colors = new int[]{Scene.pApplet.color(255),Scene.pApplet.color(150), Scene.pApplet.color(150)};

        for(Scene scene : scenes){
            scene.setBounds(100);
            scene.fit(0);
            scene.setType(Scene.Type.ORTHOGRAPHIC);
        }

        reference = new Node();
        Component component = generateComponent();

        scaleGizmo = new ScaleGizmo();
        translateGizmo = new TranslateGizmo();
        rotateGizmo = new RotateGizmo(scene);
    }

    public void display(){

        //rotateGizmo.ref.setWorldOrientation(new Quaternion());
        //rotateGizmo.ref.setWorldMagnitude(1);

        if(translateGizmo.current != null)translateGizmo.ref.setWorldPosition(translateGizmo.current);
        translateGizmo.ref.setWorldOrientation(scene.eye().worldOrientation());

        //if(rotateGizmo.current != null)rotateGizmo.ref.setWorldPosition(rotateGizmo.current);
        //rotateGizmo.ref.setWorldOrientation(scene.eye().worldOrientation());

        sceneXZ.eye().set(scene.eye());
        sceneXZ.eye().orbit(scene.eye().worldDisplacement(new Vector(1,0,0)), PI / 2, 0);
        sceneXZ.eye().setMagnitude(sceneXZ.eye().magnitude() * 2);

        sceneYZ.eye().set(scene.eye());
        sceneYZ.eye().orbit(scene.eye().worldDisplacement(new Vector(0,1,0)), PI / 2, 0);
        sceneYZ.eye().setMagnitude(sceneYZ.eye().magnitude() * 2);

        int c = 0;
        for(Scene scene : scenes){
            scene.openContext();
            scene.context().lights();
            scene.context().background(colors[c++]);
            scene.drawAxes();
            scene.render(reference);
            scene.closeContext();
        }

        scene.image(x,y);
        sceneXZ.image(x + 2 * w / 3, y);
        sceneYZ.image(x + 2 * w / 3, y + h / 2);
    }

    //Define an initial component
    public Component generateComponent(){
        Component component = new Component(this, shapeSelectorArea.currentShapeType, 20);
        component.setReference(reference);
        return component;
    }

    public void resetGizmos(){
        scaleGizmo.detach();
        translateGizmo.detach();
        rotateGizmo.detach();
    }


    public void savePosture(){
        for(Node node : components){
            node.addKeyFrame();
        }
    }

    public void play(){
        for(Node node : Scene.branch(reference)){
            node.animate();
        }
    }


    public void mouseMoved() {
        scene.updateTag();
    }

    public void mouseClicked(MouseEvent event){
        if (event.getCount() == 1){
            switch(mode){
                case SHAPESELECTION : {
                    scene.interact(Component.Interaction.CHANGE_SHAPE, shapeSelectorArea.currentShapeType);
                    break;
                }
                case SCALING_GIZMO : {
                    if(scene.node() instanceof Component){
                        scaleGizmo.attachToComponent((Component) scene.node());
                    }
                    break;
                }
                case TRANSLATION_GIZMO : {
                    if(scene.node() instanceof Component){
                        translateGizmo.attachToComponent(this, (Component) scene.node());
                    }
                    break;
                }
                case ROTATION_GIZMO : {
                    if(scene.node() instanceof Component){
                        rotateGizmo.attachToComponent(this, (Component) scene.node());
                    }
                    break;
                }
            }
        }
        else if (event.getCount() == 2){
            if (event.getButton() == LEFT)
                scene.focus();
            else
                scene.align();
        }
    }


    public void mousePressed(){
        if(scaleGizmo.hasFocus(scene.node())){
            scaleGizmo.start = scene.node().position().copy();
        }
        else if(scene.node() != null && Scene.pApplet.mouseButton == PConstants.RIGHT){
            if(Scene.pApplet.keyPressed &&
                    Scene.pApplet.key == PConstants.CODED &&
                    Scene.pApplet.keyCode == PConstants.SHIFT){
                scene.interact(Component.Interaction.EXTRUDE);
            } else if(Scene.pApplet.keyPressed &&
                    Scene.pApplet.key == PConstants.CODED &&
                    Scene.pApplet.keyCode == PConstants.ALT){
                scene.interact(Component.Interaction.EXTRUDE);
            }
        }
    }

    public void mouseDragged() {
        if (Scene.pApplet.mouseButton == PConstants.LEFT) {
            scene.spin();
        } else if (Scene.pApplet.mouseButton == PConstants.RIGHT) {
            if(scaleGizmo.hasFocus(scene.node()) || translateGizmo.hasFocus(scene.node()) || rotateGizmo.hasFocus(scene.node())){
                scene.interact();
            } else if(scene.node() != null &&
                    Scene.pApplet.keyPressed &&
                    Scene.pApplet.key == PConstants.CODED &&
                    Scene.pApplet.keyCode == PConstants.CONTROL){
                scene.interact(Component.Interaction.ON_CHANGING_REF, scene);
            } else {
                scene.shift();
            }
        } else
            scene.zoom(Scene.pApplet.mouseX - Scene.pApplet.pmouseX);
    }

    public void mouseReleased(){
        if(scaleGizmo.hasFocus(scene.node())){
            scaleGizmo.start = null;
        }
        else if(scene.node() != null &&
                Scene.pApplet.keyPressed &&
                Scene.pApplet.key == PConstants.CODED &&
                Scene.pApplet.keyCode == PConstants.CONTROL){
            scene.interact(Component.Interaction.UPDATE_REF, this);
        } else {
            //Clear auxiliary line
            scene.interact(Component.Interaction.CANCEL_REF_UPDATE, this);
        }
    }

    public void mouseWheel(MouseEvent event) {
        scene.moveForward(event.getCount() * 5);
    }
}