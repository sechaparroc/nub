package intellij.simpleModel;

import intellij.simpleModel.interactiveAreas.InteractiveArea;
import intellij.simpleModel.interactiveAreas.MainArea;
import intellij.simpleModel.interactiveAreas.ShapeSelectorArea;
import processing.core.PApplet;
import processing.event.MouseEvent;

public class SimpleModel extends PApplet {

    MainArea mainArea;
    ShapeSelectorArea shapeSelectorArea;
    InteractiveArea currentArea = null;

    public void settings(){
        size(1400,800, P3D);
    }

    public void setup(){
        shapeSelectorArea = new ShapeSelectorArea(this, 0, height - height / 6, width, height / 6);
        mainArea = new MainArea(this, shapeSelectorArea, 0, 0, width, height - height / 6);
        currentArea = mainArea;
    }

    public void draw(){
        mainArea.display();
        shapeSelectorArea.display(color(0));
    }

    public void mouseClicked(MouseEvent event){
        currentArea.mouseClicked(event);
    }

    public void mouseMoved() {
        updateCurrentArea();
        currentArea.mouseMoved();
    }

    public void mousePressed(){
        currentArea.mousePressed();
    }

    public void mouseReleased(){
        currentArea.mouseReleased();
    }

    public void mouseDragged() {
        currentArea.mouseDragged();
    }

    public void mouseWheel(MouseEvent event) {
        currentArea.mouseWheel(event);
    }

    public void updateCurrentArea(){
        currentArea = shapeSelectorArea.scene.hasFocus() ? shapeSelectorArea : mainArea;
    }

    public void keyPressed(){
        if(key == '1'){
            mainArea.mode = MainArea.Mode.SHAPESELECTION;
            mainArea.resetGizmos();
        }
        else if(key == '2'){
            mainArea.mode = MainArea.Mode.SCALING_GIZMO;
            mainArea.resetGizmos();
        }
        else if(key == '3'){
            mainArea.mode = MainArea.Mode.TRANSLATION_GIZMO;
            mainArea.resetGizmos();
        }
        else if(key == '4'){
            mainArea.mode = MainArea.Mode.ROTATION_GIZMO;
            mainArea.resetGizmos();
        }
        else if(key == '0'){
            mainArea.savePosture();
        }
        else if(key == '9'){
            mainArea.play();
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{"intellij.simpleModel.SimpleModel"});
    }
}
