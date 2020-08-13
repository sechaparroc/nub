package ik.trik;

import nub.core.Graph;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.util.ArrayList;

public class LissajousViewer extends PApplet {
  Scene scene;
  int n = 500;
  float x_speed = 1;
  float y_speed = 1;
  float z_speed = 0;
  ArrayList<Vector> positions;


  public void settings(){
    size(800,600, P3D);
  }

  public void setup(){
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.setRadius(100);
    scene.enableHint(Graph.BACKGROUND | Graph.AXES);
    scene.setShape(pg ->{
      pg.pushStyle();
      int s = 0;
      for(Vector p : positions){
        pg.pushMatrix();
        pg.noStroke();
        pg.fill(255,0,0, 40 + 215.f * s++ / n);
        pg.translate(p.x(), p.y(), p.z());
        pg.sphere(scene.radius() * 0.05f);
        pg.popMatrix();
      }
      pg.popStyle();
    });
    scene.setHUD(pg -> {
      pg.pushStyle();
      pg.fill(255);
      pg.stroke(255);
      pg.text("X " + x_speed + " Y " + y_speed + " Z " + z_speed, 50, 50);
      pg.popStyle();
    });


    positions = generateLissajousCurve(n, x_speed, y_speed, z_speed, scene.radius() * 0.6f);

  }

  public void draw(){
    //lights();
    scene.render();
  }

  public ArrayList<Vector> generateLissajousCurve(int n, float x_speed, float y_speed, float z_speed, float radius) {
    ArrayList<Vector> targetPositions = new ArrayList<Vector>();
    Vector init = new Vector();
    float step = 360f / n;
    for (float angle = 0; angle < 360 + step; angle += step) {
      float rad = radians(angle);
      float x = radius * cos(x_speed * rad);
      float y = radius * sin(y_speed * rad);
      float z = radius * sin(z_speed * rad);
      targetPositions.add(new Vector(init.x() + x, init.y() - (y + radius * 1.2f), init.z() + z));
    }
    return targetPositions;
  }

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

  public void keyPressed(){
    x_speed = random(1,5);
    y_speed = random(1,5);
    z_speed = random(1,5);
    positions = generateLissajousCurve(n, x_speed, y_speed, z_speed, scene.radius() * 0.6f);
  }


  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.trik.LissajousViewer"});
  }

}
