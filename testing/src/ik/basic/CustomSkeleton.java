package ik.basic;

import intellij.Luxo;
import nub.core.Node;
import nub.core.constraint.*;
import nub.ik.animation.Skeleton;
import nub.primitives.*;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.core.PGraphics;
import processing.event.MouseEvent;

public class CustomSkeleton extends PApplet {
  int w = 1200;
  int h = 1200;

  Scene scene;
  float length = 50;
  //Skeleton structure defined above
  Skeleton skeleton;

  public void settings() {
    size(w, h, P3D);
  }

  public void setup() {
    //Setting the scene
    scene = new Scene(this);
    scene.setBounds(200);
    scene.fit(1);
    //1. Create the Skeleton (Luxo Lamp)
    skeleton = new Skeleton();

    Piece base = new Piece();
    base.mode = 1;
    skeleton.addJoint("Base", base);

    Piece arm1 = new Piece();
    skeleton.addJoint("Arm 1", "Base", arm1);
    arm1.mode = 2;
    arm1.setTranslation(0, 0, 8); // Base height
    arm1.setRotation(new Quaternion(new Vector(1, 0, 0), 0.6f));

    Piece arm2 = new Piece();
    skeleton.addJoint("Arm 2", "Arm 1", arm2);
    arm2.mode = 2;
    arm2.setTranslation(0, 0, 50); // Arm length
    arm2.setRotation(new Quaternion(new Vector(1, 0, 0), -2));

    Piece end = new Piece();
    skeleton.addJoint("End", "Arm 2", end);
    end.mode = 3;
    end.setTranslation(0, 0, 50); // Arm length
    end.setRotation(new Quaternion(new Vector(1, -0.3f, 0), -1.7f));

    //Set constraints
    Hinge h0 = new Hinge(radians(180), radians(180), base.rotation().get(), arm1.translation(), new Vector(0,0,1));
    h0.setTranslationConstraint(AxisPlaneConstraint.Type.PLANE, new Vector(0, 0, 1));
    base.setConstraint(h0);
    Hinge h1 = new Hinge(radians(140), radians(5), arm1.rotation().get(), arm2.translation(), new Vector(1,0,0));
    Hinge h2 = new Hinge(radians(30), radians(30), arm2.rotation().get(), end.translation(), new Vector(1,0,0));
    arm1.setConstraint(h1);
    arm2.setConstraint(h2);

    LocalConstraint headConstraint = new LocalConstraint();
    headConstraint.setTranslationConstraint(AxisPlaneConstraint.Type.FORBIDDEN, new Vector(0.0f, 0.0f, 0.0f));
    end.setConstraint(headConstraint);

    //2. Enable IK functionallity
    skeleton.enableIK();
    //3. Lets create a Targets related with the end of the lamp.
    Node t = skeleton.addTarget("End");
    t.enableHint(Node.AXES);
    skeleton.joint("End").enableHint(Node.AXES);
    t.setOrientation(skeleton.joint("End").orientation().get());
    //4. if desired you could set the target position and orientation to be the same as the leaves of the structure
    skeleton.restoreTargetsState();
  }

  public void draw() {
    background(0);
    lights();

    //draw the lamp
    scene.render();

    //draw the ground
    noStroke();
    fill(120, 120, 120);
    float nbPatches = 100;
    normal(0, 0, 1);
    for (int j = 0; j < nbPatches; ++j) {
      beginShape(QUAD_STRIP);
      for (int i = 0; i <= nbPatches; ++i) {
        vertex((200 * (float) i / nbPatches - 100), (200 * j / nbPatches - 100));
        vertex((200 * (float) i / nbPatches - 100), (200 * (float) (j + 1) / nbPatches - 100));
      }
      endShape();
    }
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

  class Piece extends Node {
    int mode;

    void drawCone(PGraphics pg, float zMin, float zMax, float r1, float r2, int nbSub) {
      pg.translate(0.0f, 0.0f, zMin);
      Scene.drawCone(pg, nbSub, 0, 0, r1, r2, zMax - zMin);
      pg.translate(0.0f, 0.0f, -zMin);
    }

    @Override
    public void graphics(PGraphics pGraphics) {
      switch (mode) {
        case 1:
          pGraphics.fill(isTagged(scene) ? 255 : 0, 0, 255);
          drawCone(pGraphics, 0, 3, 15, 15, 30);
          drawCone(pGraphics, 3, 5, 15, 13, 30);
          drawCone(pGraphics, 5, 7, 13, 1, 30);
          drawCone(pGraphics, 7, 9, 1, 1, 10);
          break;
        case 2:
          pGraphics.pushMatrix();
          pGraphics.rotate(HALF_PI, 0, 1, 0);
          drawCone(pGraphics, -5, 5, 2, 2, 20);
          pGraphics.popMatrix();

          pGraphics.translate(2, 0, 0);
          drawCone(pGraphics, 0, 50, 1, 1, 10);
          pGraphics.translate(-4, 0, 0);
          drawCone(pGraphics, 0, 50, 1, 1, 10);
          pGraphics.translate(2, 0, 0);
          break;
        case 3:
          pGraphics.fill(0, 255, isTagged(scene) ? 0 : 255);
          drawCone(pGraphics, -2, 6, 4, 4, 30);
          drawCone(pGraphics, 6, 15, 4, 17, 30);
          drawCone(pGraphics, 15, 17, 17, 17, 30);
          pGraphics.spotLight(155, 255, 255, 0, 0, 0, 0, 0, 1, THIRD_PI, 1);
          break;
      }
    }
  }
  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.basic.CustomSkeleton"});
  }
}
