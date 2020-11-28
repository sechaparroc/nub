/**
 * Skeleton IK
 * by Sebastian Chaparro.
 *
 * This example illustrates how to create and manipulate a skeleton structure using Skeleton class .
 * The skeleton will consist on a Y-Shape structure on the XY-Plane:
 *                            World
 *                              ^
 *                              |
 *                              0
 *                              ^
 *                             / \
 *                            1   2
 *                           /     \
 *                          3       4
 *                         /         \
 *                        5           6
 * As Nodes 5 and 6 are the End effectors of the structure (leaf nodes)
 * we will add a Target for each one of them.
 */

import nub.primitives.*;
import nub.core.*;
import nub.processing.*;
//this packages are required for ik behavior
import nub.ik.animation.*;
import nub.ik.solver.*;

int w = 1200;
int h = 1200;

//Choose a renderer P2D or P3D
String renderer = P3D;

Scene scene;
float length = 50;
//Skeleton structure defined above
Skeleton skeleton;

void settings() {
    size(w, h, renderer);
}

void setup() {
    //Setting the scene
    scene = new Scene(this);
    scene.setBounds(200);
    scene.fit(1);
    //1. Create the Skeleton (Y-Shape described above)
    skeleton = new Skeleton();
    /*
      A joint is a node with a predefined visual representation.
      To add a joint to the Skeleton you must use either the method skeleton.addJoint(name)
      or skeleton.addJoint(name, reference_name).
      Each joint has a unique name that will be use later by the IK solver.            
    */
    Node joint0 = skeleton.addJoint("Joint 0");
    joint0.translate(new Vector(0,-scene.radius()/2));
    Node joint1 = skeleton.addJoint("Joint 1", "Joint 0"); 
    joint1.translate(new Vector(0,length));
    Node joint2 = skeleton.addJoint("Joint 2","Joint 0");
    joint2.translate(new Vector(0,length));
    Node joint3 = skeleton.addJoint("Joint 3", "Joint 1");
    joint3.translate(new Vector(-length,length));
    Node joint4 = skeleton.addJoint("Joint 4", "Joint 2");
    joint4.translate(new Vector(length,length));
    Node joint5 = skeleton.addJoint("Joint 5", "Joint 3");
    joint5.translate(new Vector(-length,length));
    Node joint6 = skeleton.addJoint("Joint 6", "Joint 4");
    joint6.translate(new Vector(length,length));

    //2. Enable IK functionallity
    /*
      Choose among these solvers: 
        * GHIK.HeuristicMode.CCD
        * GHIK.HeuristicMode.BFIK_CCD
        * GHIK.HeuristicMode.TIK
        * GHIK.HeuristicMode.BFIK_TIK
        * GHIK.HeuristicMode.TRIK
        * GHIK.HeuristicMode.BFIK_TRIK
        * GHIK.HeuristicMode.ECTIK
        * GHIK.HeuristicMode.TRIK_ECTIK
   */

    skeleton.enableIK(GHIK.HeuristicMode.TRIK);
    //3. Lets create two Targets indicating the name of the leaf nodes. 
    skeleton.addTarget("Joint 5");
    skeleton.addTarget("Joint 6");
    //4. if desired you could set the target position and orientation to be the same as the leaves of the structure 
    skeleton.restoreTargetsState();
    //Define Text Properties
    textAlign(CENTER);
    textSize(24);
}

void draw() {
    background(0);
    if(scene.is3D()) lights();
    scene.drawAxes();
    scene.render();
    
    noLights();
    scene.beginHUD();
    text("Basic Skeleton Structure", width /2, 100);
    for(Node joint : skeleton.joints().values()){
        Vector screenLocation = scene.screenLocation(joint.position());
        String s = !joint.children().isEmpty() ? "" : "End effector: ";
        s += skeleton.jointName(joint);
        if(skeleton.jointName(joint).equals("Joint 1"))
          s += ", Joint 2";
        if(skeleton.jointName(joint).equals("Joint 2")) continue;
        text(s , screenLocation.x(), screenLocation.y() + 30);
    }
    scene.endHUD();
    
}

void mouseMoved() {
    scene.mouseTag();
}

void mouseDragged() {
    if (mouseButton == LEFT){
        scene.mouseSpin();
    } else if (mouseButton == RIGHT) {
        scene.mouseTranslate();
    } else {
        scene.scale(mouseX - pmouseX);
    }
}

void mouseWheel(MouseEvent event) {
    scene.scale(event.getCount() * 20);
}

void mouseClicked(MouseEvent event) {
    if (event.getCount() == 2)
        if (event.getButton() == LEFT)
            scene.focus();
        else
            scene.align();
}
