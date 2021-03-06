package ik.basic;

import nub.core.Graph;
import nub.core.Node;
import nub.ik.solver.Solver;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.core.PGraphics;
import processing.core.PShape;
import processing.event.MouseEvent;

/**
 * Created by sebchaparr on 01/06/19.
 */

public class RegisterIK extends PApplet {

    /*
    In this example an IK solver will be related
    with a Y-Shape structure on the XY-Plane:
                             World
                               ^
                               |\
                               0 eye
                               ^
                              / \
                             1   2                               ^
                            /     \
                           3       4
                          /         \
                         5           6
    As Nodes 5 and 6 are the End effectors of the structure (leaf nodes)
    we will add a Target for each one of them.
    Note: Nodes 1 and 2 lie on the same position (Visually it seems like a single Node).
    This is done in order to make the motion of branches 1,3,5 and 2,4,6 independent .
    So if Node 1 is rotated Nodes 2,4,6 will not move.
    */

  Scene scene;
  //Set the scene as P3D or P2D
  String renderer = P3D;
  float jointRadius = 5;
  float length = 50;
  //Skeleton structure defined above
  Node[] skeleton = new Node[7];


  public void settings() {
    size(700, 700, renderer);
  }

  public void setup() {
    //Setting the scene
    scene = new Scene(this);
    if (scene.is3D()) scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.setBounds(200);
    scene.fit(1);
    //1. Create the Skeleton (Y-Shape described above)
    skeleton[0] = createJoint(scene, null, new Vector(), jointRadius, false);
    skeleton[1] = createJoint(scene, skeleton[0], new Vector(0, length), jointRadius, true);
    skeleton[2] = createJoint(scene, skeleton[0], new Vector(0, length), jointRadius, true);
    skeleton[3] = createJoint(scene, skeleton[1], new Vector(-length, length), jointRadius, true);
    skeleton[4] = createJoint(scene, skeleton[2], new Vector(length, length), jointRadius, true);

    //Left End Effector
    skeleton[5] = createJoint(scene, skeleton[3], new Vector(-length, length), jointRadius, true);
    //Right End Effector
    skeleton[6] = createJoint(scene, skeleton[4], new Vector(length, length), jointRadius, true);

    //As targets and effectors lie on the same spot, is preferable to disable End Effectors tracking
    skeleton[5].tagging = false;
    skeleton[6].tagging = false;

    //2. Lets create two Targets (a bit bigger than a Joint structure)
    Node leftTarget = createTarget(scene, jointRadius * 1.1f);
    Node rightTarget = createTarget(scene, jointRadius * 1.1f);

    //Locate the Targets on same spot of the end effectors
    leftTarget.setPosition(skeleton[5].position());
    rightTarget.setPosition(skeleton[6].position());

    //3. Relate the structure with a Solver. In this example we register a solver in the graph scene
    Solver solver = scene.registerTreeSolver(skeleton[0]);

    //Optionally you could modify the following parameters of the Solver:
    //Maximum distance between end effector and target, If is below maxError, then we stop executing IK solver (Default value is 0.01)
    solver.setMaxError(1);
    //Number of iterations to perform in order to reach the target (Default value is 50)
    solver.setMaxIterations(15);
    //Times a solver will iterate on a single Frame (Default value is 5)
    solver.setTimesPerFrame(5);
    //Minimum distance between previous and current solution to consider that Solver converges (Default value is 0.01)
    solver.setMinDistance(0.5f);

    //4. relate targets with end effectors
    scene.addIKTarget(skeleton[5], leftTarget);
    scene.addIKTarget(skeleton[6], rightTarget);

    //Define Text Properties
    textAlign(CENTER);
    textSize(24);
  }

  public void draw() {
    background(0);
    if (scene.is3D()) lights();
    scene.drawAxes();
    scene.render();
    scene.beginHUD();
    for (int i = 0; i < skeleton.length; i++) {
      if (i == 2) continue;
      //Print Node names
      Vector screenLocation = scene.screenLocation(skeleton[i].position());
      String s = "";
      if (i == 1) {
        s += ", " + (i + 1);
      }
      text("Node " + i + s, screenLocation.x(), screenLocation.y());

    }
    scene.endHUD();

  }


  public Node createTarget(Scene scene, float radius) {
    /*
     * A target is a Node, we represent a Target as a
     * Red ball.
     * */
    PShape redBall;
    if (scene.is2D()) redBall = createShape(ELLIPSE, 0, 0, radius * 2, radius * 2);
    else redBall = createShape(SPHERE, radius);
    redBall.setStroke(false);
    redBall.setFill(color(255, 0, 0));

    Node target = new Node(redBall);
    //Exact picking precision
    target.setBullsEyeSize(0);
    return target;
  }

  public Node createJoint(Scene scene, Node node, Vector translation, float radius, boolean drawLine) {
    /*
     * A Joint will be represented as a ball
     * that is joined to its reference Node
     * */

    Node joint = new Node();

    joint.setShape(pg -> {
        pg.pushStyle();
        if (drawLine) {
          pg.stroke(255);
          Vector v = joint.location(new Vector(), joint.reference());
          if (pg.is2D()) {
            pg.line(0, 0, v.x(), v.y());
          } else {
            pg.line(0, 0, 0, v.x(), v.y(), v.z());
          }
        }
        pg.fill(color(0, 255, 0));
        pg.noStroke();
        if (pg.is2D()) pg.ellipse(0, 0, radius * 2, radius * 2);
        else pg.sphere(radius);
        pg.popStyle();
    });

    joint.setReference(node);
    //Exact picking precision
    joint.setBullsEyeSize(0);
    joint.setTranslation(translation);
    return joint;
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

  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.basic.RegisterIK"});
  }
}
