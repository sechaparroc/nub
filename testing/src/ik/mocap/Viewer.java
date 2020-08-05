package ik.mocap;

import nub.core.Graph;
import nub.ik.loader.bvh.BVHLoader;
import nub.primitives.Quaternion;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.event.MouseEvent;

/**
 * Created by sebchaparr on 23/03/18.
 */
public class Viewer extends PApplet {
  Scene scene;
  String path = "/testing/data/bvh/mocap.bvh";

  BVHLoader parser;
  float[] exploration;
  boolean read = false;

  public void settings() {
    size(700, 700, P3D);
  }

  public void setup() {
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.setRadius(200);
    scene.eye().rotate(new Quaternion(0, 0, PI));
    scene.fit(1);
    scene.enableHint(Graph.BACKGROUND | Graph.AXES);
    parser = new BVHLoader(sketchPath() + path, scene, null);
  }

  public void draw() {
    ambientLight(102, 102, 102);
    lightSpecular(204, 204, 204);
    directionalLight(102, 102, 102, 0, 0, -1);
    specular(255, 255, 255);
    shininess(10);
    scene.render();
    if (read) {
      parser.nextPosture();
    }
  }


  public void keyPressed() {
    if (key == ' ') {
      read = !read;
    }
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

  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.mocap.Viewer"});
  }

}
