/**
 * Drawing a Ball & Socket constraint.
 * by Sebastian Chaparro Cuevas.
 *
 * This demo shows visually the meaning of each parameter of a Ball & Socket constraint.
 * (See https://github.com/sechaparroc/nub/wiki/4.-Rotational-constraints)
 * 
 * Here three scenes are displayed:
 * ConstraintScene: The leftmost scene shows a simple hierarchy of nodes where the first joint contains the constraint
 * to modify,
 * ThetaScene: The middle scene allows you to change the Up, Down, Left and Right parameters of the constraint. 
 * Base Scene: The rightmost scene allows you to change the shape of the base of the contraint. 
 */


import nub.core.*;
import nub.core.constraint.*;
import nub.primitives.*;
import nub.processing.*;

Scene constraintScene, thetaScene, baseScene, focus;
Node constraintRoot, thetaRoot, baseRoot;

ThetaControl t_lr, t_ud;
BaseControl base;
Joint j0, j1, target;

PFont font;

public void setup() {
    size(800, 600, P3D);
    font = createFont("Arial", 38);
    constraintScene = new Scene(createGraphics(width / 3, height, P3D));
    constraintScene.setType(Graph.Type.ORTHOGRAPHIC);
    constraintScene.leftHanded = true;
    constraintScene.fit(1);
    constraintRoot = new Node();

    constraintRoot.tagging = false;

    thetaScene = new Scene(createGraphics(width / 3 + 1, height,  P2D)); //w / 3, 0
    thetaScene.leftHanded = true;
    thetaScene.fit(1);
    thetaRoot = new Node();
    thetaRoot.tagging = false;

    baseScene = new Scene(createGraphics(width / 3 + 1, height, P2D)); //2 * w / 3, 0
    baseScene.leftHanded = true;
    baseScene.fit(1);
    baseRoot = new Node();
    baseRoot.tagging = false;

    //Create a Joint
    j0 = new Joint(constraintScene, color(66, 135, 245), 0.1f * constraintScene.radius());
    j0.setRoot(true);
    j0.setReference(constraintRoot);
    j0.translate(-constraintScene.radius() * 0.5f, 0, 0);
    j1 = new Joint(constraintScene, color(66, 135, 245), 0.1f * constraintScene.radius());
    j1.setReference(j0);

    Vector v = new Vector(1f, 0.f, 0);
    v.normalize();
    v.multiply(constraintScene.radius());
    j1.translate(v);
    j1.tagging = false;
    j1.tagging = false;

    //Add constraint to joint j0
    BallAndSocket constraint = new BallAndSocket(radians(30), radians(30), radians(30), radians(30));
    constraint.setRestRotation(j0.rotation(), new Vector(0, 1, 0), new Vector(1, 0, 0), j1.translation());
    j0.setConstraint(constraint);
    j0.rotate(new Vector(1,0,0), PI);
    //Create controllers
    t_lr = new ThetaControl(thetaScene, color(255, 154, 31));
    t_lr.setReference(thetaRoot);
    t_lr.translate(-thetaScene.radius() * 0.3f, -thetaScene.radius() * 0.7f, 0);
    t_lr.setNames("Right", "Left");
    t_ud = new ThetaControl(thetaScene, color(31, 132, 255));
    t_ud.setReference(thetaRoot);
    t_ud.translate(-thetaScene.radius() * 0.3f, thetaScene.radius() * 0.8f, 0);
    t_ud.setNames("Up", "Down");
    base = new BaseControl(baseScene, color(100, 203, 30));
    base.setReference(baseRoot);
    //Update controllers
    updateControllers(constraint, t_lr, t_ud, base);

    //Define scene hints
    constraintScene.enableHint(Graph.BACKGROUND, color(200));
    thetaScene.enableHint(Graph.BACKGROUND, color(150));
    baseScene.enableHint(Graph.BACKGROUND, color(100));
}

public void draw() {
    handleMouse();
    background(0);
    drawScene(constraintScene, constraintRoot, "Constraint View", 0, 0);
    drawScene(thetaScene, thetaRoot, "Side / Top View", width / 3, 0);
    drawScene(baseScene, baseRoot, "Front View", 2 * width / 3, 0);
    updateCostraint((BallAndSocket) j0.constraint(), t_lr, t_ud, base);
}

public void updateCostraint(BallAndSocket constraint, ThetaControl lr, ThetaControl ud, BaseControl b) {
  if (lr.modified()) {
    constraint.setLeft(lr.maxAngle());
    constraint.setRight(lr.minAngle());
    updateControllers(constraint, lr, ud, b);
    lr.setModified(false);
  } else if (ud.modified()) {
    constraint.setUp(ud.maxAngle());
    constraint.setDown(ud.minAngle());
    ud.setModified(false);
    updateControllers(constraint, lr, ud, b);

  } else if (b.modified()) {
    constraint.setLeft(b.toAngle(b.left()));
    constraint.setRight(b.toAngle(b.right()));
    constraint.setUp(b.toAngle(b.up()));
    constraint.setDown(b.toAngle(b.down()));
    b.setModified(false);
    updateControllers(constraint, lr, ud, b);
  }
}

public void updateControllers(BallAndSocket constraint, ThetaControl lr, ThetaControl ud, BaseControl b) {
  lr.update(constraint.right(), constraint.left());
  ud.update(constraint.down(), constraint.up());
  b.update(constraint.left(), constraint.right(), constraint.up(), constraint.down());
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
  focus = mouseX < width / 3 ? constraintScene : mouseX < 2 * width / 3 ? thetaScene : baseScene;
  if (prev != focus && prev != null && prev.node() != null) {
    prev.interact(prev.node(), "clear");
    if (focus != null && focus.node() != null) focus.node().interact(new Object[]{"Clear"});
  }

}

public void mouseMoved() {
  focus.mouseTag();
}

public void mouseDragged() {
  if (focus == thetaScene || focus == baseScene) {
    if (focus.node() != null) focus.interact(focus.node(), "OnScaling", new Vector(focus.mouseX(), focus.mouseY()));
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
  if (focus == thetaScene || focus == baseScene) {
    if (focus.node() != null) focus.interact(focus.node(), "Scale");
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

public static void drawCone(PGraphics pGraphics, int detail, float x, float y, float height, float left_radius, float up_radius, float right_radius, float down_radius, boolean is3D) {
  pGraphics.pushStyle();
  detail = detail % 4 != 0 ? detail + (4 - detail % 4) : detail;
  detail = Math.min(64, detail);

  float unitConeX[] = new float[detail + 1];
  float unitConeY[] = new float[detail + 1];

  int d = detail / 4;

  for (int i = 0; i <= d; i++) {
    float a1 = (PApplet.PI * i) / (2.f * d);
    unitConeX[i] = right_radius * (float) Math.cos(a1);
    unitConeY[i] = up_radius * (float) Math.sin(a1);
    unitConeX[i + d] = left_radius * (float) Math.cos(a1 + PApplet.HALF_PI);
    unitConeY[i + d] = up_radius * (float) Math.sin(a1 + PApplet.HALF_PI);
    unitConeX[i + 2 * d] = left_radius * (float) Math.cos(a1 + PApplet.PI);
    unitConeY[i + 2 * d] = down_radius * (float) Math.sin(a1 + PApplet.PI);
    unitConeX[i + 3 * d] = right_radius * (float) Math.cos(a1 + 3 * PApplet.PI / 2);
    unitConeY[i + 3 * d] = down_radius * (float) Math.sin(a1 + 3 * PApplet.PI / 2);
  }
  pGraphics.pushMatrix();
  pGraphics.translate(x, y);
  pGraphics.beginShape(PApplet.TRIANGLE_FAN);
  if (is3D) pGraphics.vertex(0, 0, 0);
  else pGraphics.vertex(0, 0);
  for (int i = 0; i <= detail; i++) {
    if (is3D) pGraphics.vertex(unitConeX[i], unitConeY[i], height);
    else pGraphics.vertex(unitConeX[i], unitConeY[i]);
  }
  pGraphics.endShape();
  pGraphics.popMatrix();
  pGraphics.popStyle();
}
