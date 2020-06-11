package intellij;

import nub.core.Node;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.core.PShape;
import processing.event.MouseEvent;

public class MiniMap extends PApplet {
  Scene scene, minimap, focus;
  Node[] models;
  boolean displayMinimap = true;
  // whilst scene is either on-screen or not, the minimap is always off-screen
  // test both cases here:
  boolean onScreen = true;
  boolean interactiveEye;

  int w = 1200;
  int h = 1200;

  //Choose P2D or P3D
  String renderer = P2D;

  @Override
  public void settings() {
    size(w, h, renderer);
  }

  @Override
  public void setup() {
    scene = onScreen ? new Scene(this) : new Scene(this, renderer);
    scene.setRadius(1000);
    rectMode(CENTER);
    scene.fit(1);
    scene.eye().setBullsEyeSize(50);
    scene.eye().disableHint(Node.HIGHLIGHT);
    scene.eye().enableHint(Node.BULLSEYE);
    scene.eye().enableHint(Node.FRUSTUM, scene, color(255, 0, 0, 125));
    scene.enableHint(Scene.BACKGROUND, color(75, 25, 15));
    models = new Node[30];
    for (int i = 0; i < models.length; i++) {
      if ((i & 1) == 0) {
        models[i] = new Node(shape());
        models[i].setBullsEyeSize(50);
        models[i].enableHint(Node.BULLSEYE);
      } else {
        models[i] = new Node();
        models[i].enableHint(Node.BULLSEYE);
        models[i].enableHint(Node.TORUS);
        models[i].setBullsEyeSize(50);
        models[i].scale(3);
      }
      // set picking precision to the pixels of the node projection
      //models[i].setBullsEyeSize(0);
      scene.randomize(models[i]);
    }
    // Note that we pass the upper left corner coordinates where the minimap
    // is to be drawn (see drawing code below) to its constructor.
    minimap = new Scene(this, renderer, w / 2, h / 2);
    minimap.setRadius(2000);
    if (renderer == P3D)
      minimap.togglePerspective();
    minimap.fit(1);
    minimap.enableHint(Scene.BACKGROUND, color(125, 80, 90));
  }

  PShape shape() {
    PShape shape = renderer == P3D ? createShape(BOX, 60) : createShape(RECT, 0, 0, 80, 100);
    shape.setFill(color(random(0, 255), random(0, 255), random(0, 255)));
    return shape;
  }

  @Override
  public void keyPressed() {
    if (key == ' ')
      displayMinimap = !displayMinimap;
    if (key == 'i') {
      interactiveEye = !interactiveEye;
      if (interactiveEye)
        minimap.tag(scene.eye());
      else
        minimap.untag(scene.eye());
    }
    if (key == 'f')
      focus.fit(1);
    if (key == 't')
      focus.togglePerspective();
  }

  @Override
  public void mouseMoved() {
    if (!interactiveEye || focus == scene)
      focus.mouseTag();
  }

  @Override
  public void mouseDragged() {
    if (mouseButton == LEFT)
      focus.mouseSpin();
    else if (mouseButton == RIGHT)
      focus.mouseTranslate();
    else
      focus.scale(focus.mouseDX());
  }

  @Override
  public void mouseWheel(MouseEvent event) {
    if (renderer == P3D)
      focus.moveForward(event.getCount() * 40);
    else
      focus.scale(event.getCount() * 40);
  }

  @Override
  public void mouseClicked(MouseEvent event) {
    if (event.getCount() == 2)
      if (event.getButton() == LEFT)
        focus.focus();
      else
        focus.align();
  }

  @Override
  public void draw() {
    focus = minimap.hasMouseFocus() ? minimap : scene;
    if (scene.isOffscreen()) {
      scene.beginDraw();
      scene.drawAxes();
      scene.render();
      scene.endDraw();
      scene.display();
    } else {
      scene.drawAxes();
      scene.render();
    }
    if (displayMinimap) {
      if (!scene.isOffscreen())
        scene.beginHUD();
      minimap.beginDraw();
      minimap.drawAxes();
      minimap.render();
      //minimap.context().stroke(255);
      //minimap._drawBullsEye(scene.eye());
      minimap.endDraw();
      minimap.display(w / 2, h / 2);
      if (!scene.isOffscreen())
        scene.endHUD();
    }
  }

  public static void main(String[] args) {
    PApplet.main(new String[]{"intellij.MiniMap"});
  }
}
