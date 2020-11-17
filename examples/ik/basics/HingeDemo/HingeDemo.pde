/**
 * Drawing a Hinge constraint.
 * by Sebastian Chaparro Cuevas.
 *
 * This demo shows visually the meaning of each parameter of a Hinge constraint.
 * (See https://github.com/sechaparroc/nub/wiki/4.-Rotational-constraints)
 * 
 * Here two scenes are displayed:
 * ConstraintScene: The leftmost scene shows a simple hierarchy of nodes where the first joint contains the constraint
 * to modify,
 * ThetaScene: The rightmost scene allows you to change the. Min and Max. parameters of the constraint. 
 */

import nub.core.*;
import nub.core.constraint.*;
import nub.primitives.*;
import nub.processing.*;

Scene constraintScene, thetaScene, focus;
Node constraintRoot, thetaRoot;


ThetaControl control;
Node j0, j1;
PFont font;

public void setup() {
    size(800, 600, P3D);
    font = createFont("Arial", 38);
    constraintScene = new Scene(createGraphics( width / 2, height, P3D));
    constraintScene.setType(Graph.Type.ORTHOGRAPHIC);
    constraintScene.fit(1);
    constraintRoot = new Node();
    constraintRoot.tagging = false;

    thetaScene = new Scene(createGraphics( width / 2, height, P2D)); //w / 2, 0
    thetaScene.fit(1);
    thetaRoot = new Node();
    thetaRoot.tagging = false;
    //Create a Joint
    j0 = new Node(){
        public void graphics(PGraphics pGraphics) {
          float radius = constraintScene.radius();
          Hinge constraint = (Hinge) constraint();
          pGraphics.push();
          pGraphics.lights();
          pGraphics.noStroke();
          pGraphics.fill(62, 203, 55, 150);
          Node reference = Node.detach(new Vector(), new Quaternion(), 1f);
          reference.setTranslation(new Vector());
          reference.setRotation(j0.rotation().inverse());
          Quaternion referenceRotation = rotation().inverse();
          referenceRotation.compose(constraint.orientation());
          referenceRotation.compose(new Quaternion(new Vector(1, 0, 0), new Vector(0, 1, 0)));
          referenceRotation.normalize();
          pGraphics.rotate(referenceRotation.angle(), (referenceRotation).axis()._vector[0], (referenceRotation).axis()._vector[1], (referenceRotation).axis()._vector[2]);
          //Draw axis
          pGraphics.pushStyle();
          pGraphics.fill(255, 154, 31);
          Scene.drawArrow(pGraphics, new Vector(), new Vector(constraintScene.radius() / 2, 0, 0), 1f);
          pGraphics.fill(79, 196, 61);
          Scene.drawArrow(pGraphics, new Vector(), new Vector(0, 0, constraintScene.radius() / 2), 1f);
          pGraphics.popStyle();
          //Write names
          Vector v = new Vector(radius * (float) Math.cos(-constraint.minAngle()) + 5, radius * (float) Math.sin(-constraint.minAngle()));
          Vector u = new Vector(radius * (float) Math.cos(constraint.maxAngle()) + 5, radius * (float) Math.sin(constraint.maxAngle()));
          Vector w = new Vector(radius / 2, 0, 0);
          Vector s = new Vector(0, 0, radius / 2);
          pGraphics.pushStyle();
          pGraphics.noLights();
          pGraphics.fill(0);
          pGraphics.textFont(font, 12);
          pGraphics.text("\u03B8 " + "max", v.x(), v.y());
          pGraphics.text("\u03B8 " + "min", u.x(), u.y());
          pGraphics.fill(255, 154, 31);
          pGraphics.text("Up vector", w.x(), w.y() - 5, w.z() + 5);
          pGraphics.textAlign(RIGHT, BOTTOM);
          pGraphics.fill(79, 196, 61);
          pGraphics.text("Twist vector", s.x() - radius / 4, s.y(), s.z());
          pGraphics.lights();
          pGraphics.pop();

        }
    };
    j0.setReference(constraintRoot);
    j0.enableHint(Node.BONE | Node.CONSTRAINT);
    j0.configHint(Node.BONE, color(66, 135, 245), 0.1f * constraintScene.radius(), true);
    j0.translate(-constraintScene.radius() * 0.5f, 0, 0);
    j1 = new Node();//(constraintScene, color(66, 135, 245), 0.1f * constraintScene.radius());
    j1.enableHint(Node.BONE, color(66, 135, 245), 0.1f * constraintScene.radius(), true);
    j1.setReference(j0);
    Vector v = new Vector(1f, 0, 0);
    v.normalize();
    v.multiply(constraintScene.radius());
    j1.translate(v);

    //Add constraint to joint j0
    Hinge constraint = new Hinge(radians(30), radians(30), j0.rotation(), new Vector(1, 0, 0), new Vector(0, 0, 1));
    j0.setConstraint(constraint);

    //Create controllers
    control = new ThetaControl(thetaScene, color(100, 203, 30));
    control.setReference(thetaRoot);
    control.setNames("Min", "Max");

    //Update controllers
    updateControllers(constraint, control);

    constraintScene.enableHint(Graph.BACKGROUND, color(100));
    thetaScene.enableHint(Graph.BACKGROUND, color(150));
}

public void draw() {
    constraintScene.context().lights();
    handleMouse();
    drawScene(constraintScene, constraintRoot, "Constraint View", 0,0);
    drawScene(thetaScene, thetaRoot, "Hinge Control", width / 2 , 0);
    //thetaScene.drawBullsEye(control);
    updateCostraint((Hinge) j0.constraint(), control);
}

public void updateCostraint(Hinge constraint, ThetaControl control) {
  if (control.modified()) {
    constraint.setMaxAngle(control.maxAngle());
    constraint.setMinAngle(control.minAngle());
    updateControllers(constraint, control);
    control.setModified(false);
  }
}

public void updateControllers(Hinge constraint, ThetaControl control) {
  control.update(constraint.minAngle(), constraint.maxAngle());
}

public void drawScene(Scene scene, Node root, String title, int x, int y) {
  scene.context().lights();
  scene.render(root);
  scene.beginHUD();
  scene.context().noLights();
  scene.context().pushStyle();
  scene.context().fill(0);
  scene.context().stroke(0);
  scene.context().textAlign(CENTER, CENTER);
  scene.context().textFont(font, 24);
  scene.context().text(title, scene.context().width / 2, 20);
  scene.context().noFill();
  scene.context().strokeWeight(3);
  scene.context().rect(0, 0, constraintScene.context().width, constraintScene.context().height);
  scene.context().popStyle();
  scene.endHUD();
  scene.image(x, y);
}



  public void handleMouse() {
    Scene prev = focus;
    focus = mouseX < width / 2 ? constraintScene : thetaScene;
    if (prev != focus && prev != null) {
      if (prev.node() != null) prev.node().interact(new Object[]{"Clear"});
      if (focus != null && focus.node() != null) focus.node().interact(new Object[]{"Clear"});
    }

  }

  public void mouseMoved() {
    focus.mouseTag();
  }

public void mouseDragged() {
  if (focus == thetaScene) {
    if (focus.node() != null) focus.node().interact(new Object[]{"OnScaling", new Vector(focus.mouseX(), focus.mouseY())});
    return;
  }
  if (mouseButton == LEFT)
    focus.mouseSpin();
  else if (mouseButton == RIGHT) {
    focus.mouseTranslate();
  } else
    focus.moveForward(mouseX - pmouseX);
}

public void mouseReleased() {
  if (focus == thetaScene) {
    if (focus.node() != null) focus.node().interact(new Object[]{"Scale"});
    return;
  }
}

public void mouseWheel(MouseEvent event) {
  focus.scale(event.getCount() * 20);
  //focus.zoom(event.getCount() * 50);
}

public void mouseClicked(MouseEvent event) {
  if (event.getCount() == 2)
    if (event.getButton() == LEFT)
      focus.focus();
    else
      focus.align();
}
