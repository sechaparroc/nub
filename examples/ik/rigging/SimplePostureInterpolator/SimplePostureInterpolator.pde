/**
 * Simple Animation Controls
 * by Sebastian Chaparro Cuevas.
 *
 * In this example a mesh is loaded (shapePath) along with a skeleton (jsonPath) and the idea is to define
 * an Interpolator posture that can be saved and loaded.

 * Press 'C' to clear the interpolator.
 * Press 'S' to save the interpolator posture in a json file (interpolatorPath).
 * Press 'L' to load the interpolator posture from a json file (interpolatorPath).
 * Press 'K' to add a key posture
 * Press 'E' and 'D' while put the mouse over a Node to enable/disable it.  
 */


import nub.core.*;
import nub.core.constraint.*;
import nub.ik.animation.*;
import nub.ik.skinning.GPULinearBlendSkinning;
import nub.primitives.*;
import nub.processing.Scene;
import java.util.Map;


Scene scene;
Skeleton skeleton;
PostureInterpolator interpolator;
GPULinearBlendSkinning skinning;
boolean loadFromFile = true;
boolean loadMesh = true;
boolean mode = false; // change the interaction mode (load or save a posture)



String skeletonPath = "Kangaroo/Kangaroo_constrained.json";
String initialPosturePath = "Kangaroo/Kangaroo_initial.json";
String finalPosturePath = "Kangaroo/Kangaroo_final.json";
String shapePath = "Kangaroo/Kangaroo.obj";
String texturePath = "Kangaroo/Kangaroo_diff.jpg";
String interpolatorPath = "Kangaroo/Kangaroo_interpolator.json";



void settings() {
    size(800, 600, P3D);
}


void setup(){
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.leftHanded = false;
    scene.fit(1);
    //Load Skeleton
    skeleton = new Skeleton(skeletonPath);
    skeleton.updateConstraints();
    //Enable IK
    skeleton.enableIK();
    skeleton.addTargets();
    skeleton.setTargetRadius(0.03f * scene.radius());
    //Load posteures
    interpolator = new PostureInterpolator(skeleton);
    
    //Enable skinning
    if(loadMesh){
        skinning = new GPULinearBlendSkinning(skeleton, shapePath, texturePath, scene.radius(), false);
    }
    scene.enableHint(Scene.AXES);
}

void draw(){
    background(0);
    lights();
    if(loadMesh) skinning.render(scene, skeleton.reference());
    scene.render();
    pushStyle();
    noLights();
    scene.beginHUD();
    text("Mode : " + (mode ? "Load posture" : "Save posture"), 50, 50);
    scene.endHUD();
    popStyle();
}

public void mouseMoved() {
    scene.mouseTag();
}

public void mouseDragged() {
    if (mouseButton == LEFT){
        scene.mouseSpin();
    } else if (mouseButton == RIGHT) {
        scene.mouseTranslate();
    } else {
        scene.scale(mouseX - pmouseX);
    }
}

public void mouseWheel(MouseEvent event) {
    scene.scale(event.getCount() * 20);
}

public void mouseClicked(MouseEvent event) {
    if (event.getCount() == 2)
        if (event.getButton() == LEFT)
            scene.focus();
        else
            scene.align();
}

public void keyPressed(){
    if(key == 'c' || key == 'C'){
        interpolator.clear();
    }
    if(key == 's' || key == 'S'){
        interpolator.save(this, interpolatorPath);
    }
    if(key == 'l' || key == 'L'){
        interpolator.load(this, interpolatorPath);
    }
    if(key == 'k' || key == 'K'){
        interpolator.addKeyPosture(new Posture(skeleton), 1);
    }
    if(key == ' '){
        interpolator.run();
    }


    if(key == 'd' || key == 'D'){
        //fix
        Node current = scene.node();
        if(current != null){
            Constraint fix = new Constraint() {
                @Override
                public Vector constrainTranslation(Vector translation, Node node) {
                    return translation;
                }

                @Override
                public Quaternion constrainRotation(Quaternion rotation, Node node) {
                    return new Quaternion();
                }
            };
            current.setConstraint(fix);
            current.configHint(Node.BONE, color(255,255,255));
        }
    }
    if(key == 'e' || key == 'E'){
        Node current = scene.node();
        if(current != null){
            current.setConstraint(null);
            current.configHint(Node.BONE, color(random(255), random(255), random(255)));
        }
    }

    if(key == 'r' || key == 'R'){
        //restore target position
        skeleton.restoreTargetsState();
    }
}
