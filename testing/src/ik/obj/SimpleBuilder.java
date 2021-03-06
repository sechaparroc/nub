package ik.obj;

import ik.interactive.InteractiveJoint;
import nub.core.Graph;
import nub.ik.animation.Skeleton;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.core.PShape;
import processing.event.MouseEvent;

/**
 * Simple Builder
 * by Sebastian Chaparro Cuevas.
 * <p>
 * In this example a mesh is loaded from an .obj file (specified by shapePath) and the idea is to
 * generate a Skeleton Structure over the .obj shape to use later in other sketches.
 * <p>
 * To do so, it is possible to interact with a Joint (InteractiveJoint) in different ways:
 * Using the mouse:
 * Drag with RIGTH button to translate the Joint.
 * Drag with LEFT button to rotate the Joint.
 * Drag with RIGTH button while pressing CTRL to extrude a Joint from the selected one. Release to create a Joint.
 * Double click with LEFT button while pressing SHIFT key to remove the Branch from the selected Joint.
 * Using the keyboard:
 * Press 'P' to save the skeleton on a JSON file (you could require this info in other Sketch).
 * Press 'E' when the mouse is over a Joint to set its translation to (0,0,0). It is useful to mantain Chains of a Structure independent.
 */

public class SimpleBuilder extends PApplet {

  //Build easily a Skeleton to relate to a Mesh
  Scene scene;
  Skeleton skeleton;

  String lastCommand = "None";
  //Shape variables
  PShape model;

  //Set this path to load your objs
  String jsonPath = "/testing/data/skeletons/Dummy.json";
  String shapePath = "/testing/data/objs/Rigged_Hand.obj";
  String texturePath = "/testing/data/objs/HAND_C.jpg";

  float radius = 0;
  int w = 1000, h = 700;
  /*Create different skeletons to interact with*/
  String renderer = P3D;

  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.obj.SimpleBuilder"});
  }

  public void settings() {
    size(w, h, renderer);
  }

  public void setup() {
    // Create a scene
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    model = loadShape(shapePath);
    model.setTexture(loadImage(texturePath));
    //Scale scene
    float size = max(model.getHeight(), model.getWidth());
    scene.leftHanded = false;
    scene.setBounds(size);
    scene.fit();
    scene.enableHint(Graph.BACKGROUND | Graph.AXES);
    scene.enableHint(Graph.SHAPE);
    scene.setShape(pg ->{
      pg.shape(model);
    });
    scene.enableHint(Graph.HUD);
    scene.setHUD(pg ->{
      pg.noLights();
      pg.stroke(255);
      pg.stroke(255, 0, 0);
      pg.text("Last action: " + lastCommand, width / 2, 50);
    });

    //Create the Skeleton and add an Interactive Joint at the center of the scene
    skeleton = new Skeleton();
    //Create the interactive joint
    radius = scene.radius() * 0.01f;
    InteractiveJoint initial = new InteractiveJoint(true, color(random(255),random(255),random(255)), radius, false);
    //Add the joint to the skeleton
    skeleton.addJoint("J0", initial);
    textSize(18);
    textAlign(CENTER, CENTER);
  }

  public void draw() {
    ambientLight(102, 102, 102);
    lightSpecular(204, 204, 204);
    directionalLight(102, 102, 102, 0, -1,  0.4f);
    specular(255, 255, 255);
    shininess(10);
    scene.render();
  }

  //mouse events
  public void mouseMoved() {
    scene.mouseTag();
  }

  public void mouseDragged(MouseEvent event) {
    if (mouseButton == RIGHT && event.isControlDown()) {
      Vector vector = new Vector(scene.mouseX(), scene.mouseY());
      if (scene.node() != null) {
        if (scene.node() instanceof InteractiveJoint) {
          scene.interact(scene.node(), "OnAdding", scene, vector);
          lastCommand = "Extruding from a Joint";
        } else {
          scene.interact(scene.node(),"OnAdding", vector);
        }
      }
    } else if (mouseButton == LEFT) {
      scene.mouseSpin();
    } else if (mouseButton == RIGHT) {
      scene.mouseTranslate(0);
    } else if (mouseButton == CENTER) {
      scene.scale(scene.mouseDX());
    }
  }

  public void mouseReleased(MouseEvent event) {
    Vector vector = new Vector(scene.mouseX(), scene.mouseY());
    if (scene.node() != null)
      if (scene.node() instanceof InteractiveJoint) {
        if (((InteractiveJoint) scene.node()).desiredTranslation() != null) lastCommand = "Adding Joint";
        scene.interact(scene.node(),"Add", scene, vector, skeleton);
      }
  }

  public void mouseWheel(MouseEvent event) {
    scene.scale(event.getCount() * 20);
  }

  public void mouseClicked(MouseEvent event) {
    if (event.getCount() == 2) {
      if (event.getButton() == LEFT) {
        if (event.isShiftDown())
          if (scene.node() != null) {
            lastCommand = "Removing Joint and its children";
            scene.interact(scene.node(),"Remove");
          } else
            scene.focus();
      } else {
        scene.align();
      }
    }
  }

  public void keyPressed() {
    if (key == 'J' || key == 'j') {
      lastCommand = "Adding Joint on the middle of the scene";
      InteractiveJoint initial = new InteractiveJoint(true, color(random(255),random(255),random(255)), radius, false);
    } else if (key == 'P' || key == 'p') {
      lastCommand = "Skeleton information saved on : " + sketchPath() + jsonPath;
      skeleton.save(sketchPath() + jsonPath);
    } else if (key == 'E' || key == 'e') {
      if (scene.node() != null) {
        lastCommand = "Setting Joint translation to (0,0,0)";
        scene.node().setTranslation(new Vector());
        scene.node().tagging = false;
      }
    }
  }
}

