package intellij.simpleModel.interactiveAreas;

import intellij.simpleModel.Utils;
import nub.core.*;
import nub.processing.*;
import processing.core.*;
import processing.event.MouseEvent;
import java.util.HashMap;
import static processing.core.PApplet.*;

public class ShapeSelectorArea implements InteractiveArea{
    //Define custom Interaction actions
    enum ShapeSelectionInteraction { //ADD MORE INTERACTIONS IF REQUIRED
        SELECT
    }

    public Scene scene;
    public Node reference;
    public Node cursor;

    int x, y;
    public Utils.Shape currentShapeType;

    HashMap<Node, Utils.Shape> shapeOptions = new HashMap<>();
    public ShapeSelectorArea(PApplet pApplet, int x, int y, int w, int h){
        this.x = x;
        this.y = y;
        scene = new Scene(pApplet.createGraphics(w, h, PConstants.P3D));
        scene.setBounds(0.5f * h);
        scene.fit(0);
        scene.setType(Scene.Type.ORTHOGRAPHIC);
        reference = new Node();
        generateOptions();
        locateOptions();
        generateCursor();

        Node node = shapeOptions.keySet().iterator().next();
        scene.tag(node);
        currentShapeType = shapeOptions.get(node);
    }

    public void generateOption(Utils.Shape shapeType, float dim){ //pshape must be in scene dims
        PShape pshape = Utils.generateShape(shapeType, dim);
        Node option = new Node(reference, pshape);
        option.setHighlight(0.5f);
        option.enableHint(Node.AXES);
        option.setInteraction(this::interact);

        shapeOptions.put(option, shapeType);
    }

    public void locateOptions(){
        float w_step = 1 * scene.radius() * scene.aspectRatio() / shapeOptions.size();
        float x = - scene.radius() * scene.aspectRatio() * 0.5f + w_step * 0.5f;
        for(Node n : shapeOptions.keySet()){
            n.translate(x, 0, 0);
            x += w_step;
        }
    }

    public void generateCursor(){
        cursor = new Node();
        cursor.setShape(Utils.createSphere(scene.radius() * 0.05f));
        cursor.tagging = false;
    }

    public void generateOptions(){
        float option_size = min( 0.25f * 1.0f * scene.radius() * scene.aspectRatio() / Utils.Shape.values().length, scene.radius() * 0.8f);
        print(option_size);
        generateOption(Utils.Shape.BOX, option_size);
        generateOption(Utils.Shape.SPHERE, option_size * 0.5f);
        generateOption(Utils.Shape.CYLINDER, option_size);
        generateOption(Utils.Shape.CONE, option_size);
    }

    public void display(int c){
        scene.openContext();
        //scene.context().lights();
        scene.context().background(c);
        scene.render(reference);
        scene.closeContext();
        scene.image(x,y);
    }

    public void interact(Object[] gesture) {
        switch((ShapeSelectionInteraction) gesture[0]){
            case SELECT : {
                currentShapeType = shapeOptions.get(scene.node());
                break;
            }
        }
    }


    public void mouseMoved(){

    }

    public void mouseClicked(MouseEvent event){
        scene.updateTag();
        scene.interact(ShapeSelectionInteraction.SELECT);
    }

    public void mousePressed(){

    }

    public void mouseReleased(){

    }

    public void mouseDragged(){

    }

    public void mouseWheel(MouseEvent event){

    }
}