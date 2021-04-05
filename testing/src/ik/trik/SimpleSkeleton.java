package ik.trik;

import nub.core.Graph;
import nub.core.Node;
import nub.core.constraint.Hinge;
import nub.ik.animation.Skeleton;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.event.MouseEvent;

public class SimpleSkeleton extends PApplet {
  int n = 5;
  Scene scene;
  Skeleton skeleton, copy;
  float boneLenght = 50;

  public void settings(){
    size(1900,1000, P3D);
    smooth(8);
    randomSeed(0);
  }

  public void setup(){
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.setBounds(n * boneLenght * 0.6f);
    float radius = scene.radius() * 0.05f;
    skeleton = createSkeleton(radius, color(0, 100, 150,100));
    copy = createSkeleton(radius, color(68, 219, 144,100));
    scene.enableHint(Graph.BACKGROUND, color(255));
  }

  public Skeleton createSkeleton(float radius, int col){
    Skeleton skeleton = new Skeleton(null, radius, radius, col, false);
    Node root = skeleton.addJoint("" + 0);
    Hinge h = new Hinge(PI, PI);
    h.setRestRotation(root.rotation(), new Vector(1,0,0), new Vector(0,0,1));
    root.enableHint(Node.AXES, radius * 2);
    root.disableHint(Node.CONSTRAINT);
    for(int i = 1; i < n; i++){
      Node node = skeleton.addJoint("" + i, "" + (i - 1));
      node.enableHint(Node.AXES, radius * 2);
      h = new Hinge(PI, PI);
      h.setRestRotation(node.rotation(), new Vector(1,0,0), new Vector(0,0,1));
      node.translate(0, boneLenght, 0);
      node.setConstraint(h);
      node.disableHint(Node.CONSTRAINT);
    }
    skeleton.enableIK();
    skeleton.addTargets();
    skeleton.restoreTargetsState();
    skeleton.disableIK();
    return skeleton;
  }

  public void draw(){
    scene.render();
  }
  @Override
  public void mouseMoved() {
    scene.mouseTag();
  }

  public void mouseDragged() {
    if (mouseButton == LEFT) {
      scene.mouseSpin();
    } else if (mouseButton == RIGHT) {
      scene.mouseTranslate();
    } else {
      scene.scale(scene.mouseDX());
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

  boolean cull = true;
  public void keyPressed(){
    if(key == ' ') {
      copy.cull(cull);
      cull = !cull;
    } else{
      skeleton.solveIK();
      copy.solveIK();
    }
  }


  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.trik.SimpleSkeleton"});
  }


}
