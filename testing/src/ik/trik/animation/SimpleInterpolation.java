package ik.trik.animation;

import nub.core.*;
import nub.core.constraint.*;
import nub.ik.animation.*;
import nub.ik.skinning.GPULinearBlendSkinning;
import nub.ik.skinning.Skinning;
import nub.primitives.*;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.util.Map;

public class SimpleInterpolation extends PApplet {
    Scene scene;
    Skeleton skeleton;
    Posture initialPosture, finalPosture;
    Skinning skinning;
    boolean loadFromFile = true;
    boolean loadMesh = true;
    boolean mode = false; // change the interaction mode (load or save a posture)



    String skeletonPath = "/testing/data/skeletons/Kangaroo_constrained.json";
    String initialPosturePath = "/testing/data/skeletons/Kangaroo_initial.json";
    String finalPosturePath = "/testing/data/skeletons/Kangaroo_final.json";
    String shapePath = "/testing/data/objs/Kangaroo.obj";
    String texturePath = "/testing/data/objs/Kangaroo_diff.jpg";




    public void settings() {
        size(800, 600, P3D);
    }


    public void setup(){
        scene = new Scene(this);
        scene.setType(Graph.Type.ORTHOGRAPHIC);
        scene.setRightHanded();
        scene.fit(1);
        //Load Skeleton
        skeleton = new Skeleton(scene, skeletonPath);
        skeleton.updateConstraints();
        //Enable IK
        skeleton.enableIK();
        skeleton.addTargets();
        skeleton.setTargetRadius(0.03f * scene.radius());
        //Load posteures
        if(loadFromFile){
            initialPosture = new Posture(this, initialPosturePath);
            finalPosture = new Posture(this, finalPosturePath);
        }

        //Enable skinning
        if(loadMesh){
            skinning = new GPULinearBlendSkinning(skeleton, shapePath, texturePath, scene.radius(), false);
        }
    }

    public void draw(){
        background(0);
        lights();
        scene.drawAxes();
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
        if(key == '1'){
            if(mode){
                initialPosture.loadValues(skeleton);
            }
            else initialPosture = new Posture(skeleton);

        }
        if(key == '2'){
            if(mode){
                finalPosture.loadValues(skeleton);
            }
            else{
                finalPosture = new Posture(skeleton);
            }
        }
        if(key == 'm' || key == 'M'){
            mode = !mode;
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
                if(current instanceof Joint) ((Joint) current).setColor(255,0,0);
            }
        }
        if(key == 'e' || key == 'E'){
            Node current = scene.node();
            if(current != null){
                current.setConstraint(null);
                if(current instanceof Joint) ((Joint) current).setColor((int)random(0,255),(int)random(0,255),(int)random(0,255));
            }
        }

        if(key == 'r' || key == 'R'){
            //restore target position
            skeleton.restoreTargetsState();
        }
        if(key == 's' || key == 'S'){
            //save postures
            if(initialPosture != null)
                initialPosture.save(this, initialPosturePath);
            if(finalPosture != null)
                finalPosture.save(this, finalPosturePath);
        }

        if(key == ' '){
            catmullInterpolation(1);
        }
    }

    public void catmullInterpolation(float duration){
        //Apply catmull interpolation between the 2 given postures
        PostureInterpolator interpolator = new PostureInterpolator(skeleton);
        interpolator.addKeyPosture(initialPosture, 0);
        interpolator.addKeyPosture(finalPosture, duration);
        interpolator.addKeyPosture(initialPosture, duration);
        interpolator.run();
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{"ik.trik.animation.SimpleInterpolation"});
    }

}
