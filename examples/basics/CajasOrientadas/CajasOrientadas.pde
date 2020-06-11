/**
 * Cajas Orientadas.
 * by Jean Pierre Charalambos.
 *
 * This example illustrates some basic Node properties, particularly how to
 * orient them.
 *
 * The sphere and the boxes are interactive. Pick and drag them with the
 * right mouse button. Use also the arrow keys to select and move the sphere.
 * See how the boxes will always remain oriented towards the sphere.
 *
 * Press ' ' the change the picking policy.
 * Press 'c' to change the bullseye shape space.
 * Press 'c' to change the bullseye shape space.
 */

import nub.primitives.*;
import nub.core.*;
import nub.processing.*;

Scene scene;
Box[] cajas;
Sphere esfera;

void setup() {
  size(800, 800, P3D);
  // Set the inertia for all interactivity methods to 0.85. Default is 0.8.
  scene = new Scene(this);
  scene.setRadius(200);
  scene.togglePerspective();
  scene.fit();
  esfera = new Sphere(color(random(0, 255), random(0, 255), random(0, 255)), 10);
  esfera.setPosition(new Vector(0, 1.4, 0));
  cajas = new Box[15];
  for (int i = 0; i < cajas.length; i++)
    cajas[i] = new Box(color(random(0, 255), random(0, 255), random(0, 255)),
      random(10, 40), random(10, 40), random(10, 40));
  scene.fit();
  scene.tag("keyboard", esfera);
}

void draw() {
  background(0);
  scene.render();
}

void mouseClicked() {
  scene.focusEye();
}

void mouseMoved() {
  scene.mouseTag();
}

void mouseDragged() {
  if (!scene.mouseTranslateTag()) {
    if (mouseButton == LEFT)
      scene.mouseSpinEye();
    else if (mouseButton == RIGHT)
      scene.mouseTranslateEye();
    else
      scene.scaleEye(mouseX - pmouseX);
  }
}

void mouseWheel(MouseEvent event) {
  scene.moveForward(event.getCount() * 20);
}

void keyPressed() {
  if (key == ' ')
    for (Box caja : cajas)
      caja.setPickingPolicy(caja.pickingPolicy() == Node.PickingPolicy.PRECISE ?
        Node.PickingPolicy.BULLSEYE :
        Node.PickingPolicy.PRECISE);
  if (key == 'c')
    for (Box caja : cajas)
      if (caja.bullsEyeSize() < 1)
        caja.setBullsEyeSize(caja.bullsEyeSize() * 200);
      else
        caja.setBullsEyeSize(caja.bullsEyeSize() / 200);
  if (key == 'd')
    for (Box caja : cajas)
      caja.setBullsEyeShape(caja.bullsEyeShape() == Node.BullsEyeShape.CIRCLE ?
        Node.BullsEyeShape.SQUARE :
        Node.BullsEyeShape.CIRCLE);
  if (key == 'a')
    for (Box caja : cajas)
      caja.toggleHint(Node.AXES);
  if (key == 'p')
    for (Box caja : cajas)
      caja.toggleHint(Node.BULLSEYE);
  if (key == 'e')
    scene.togglePerspective();
  if (key == 's')
    scene.fit(1);
  if (key == 'S')
    scene.fit();
  if (key == 'u')
    if (scene.isTagValid("keyboard"))
      scene.removeTag("keyboard");
    else
      scene.tag("keyboard", esfera);
  if (key == CODED)
    if (keyCode == UP)
      scene.translate("keyboard", 0, -10, 0);
    else if (keyCode == DOWN)
      scene.translate("keyboard", 0, 10, 0);
    else if (keyCode == LEFT)
      scene.translate("keyboard", -10, 0, 0);
    else if (keyCode == RIGHT)
      scene.translate("keyboard", 10, 0, 0);
}
