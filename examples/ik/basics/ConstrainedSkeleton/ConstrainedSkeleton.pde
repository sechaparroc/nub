import nub.core.*;
import nub.primitives.*;
import nub.processing.*;
import nub.ik.solver.*;
import nub.ik.animation.*;
import nub.core.constraint.*;

Scene scene;
void setup(){
  size(800,600,P3D);
  float length = 50;
  //Setting the scene
  scene = new Scene(this);
  scene.setBounds(200);
  scene.fit(1);
  //Create the Skeleton
  Skeleton skeleton = new Skeleton();
  Node joint0 = skeleton.addJoint("Joint 0");
  joint0.translate(new Vector(0,-scene.radius()/2));
  Node joint1 = skeleton.addJoint("Joint 1", "Joint 0"); 
  joint1.translate(new Vector(0,length, 0));
  Node joint2 = skeleton.addJoint("Joint 2","Joint 1");
  joint2.translate(new Vector(0,length, 0));
  Node joint3 = skeleton.addJoint("Joint 3", "Joint 2");
  joint3.translate(new Vector(0,length, 0));
  Node joint4 = skeleton.addJoint("Joint 4", "Joint 3");
  joint4.translate(new Vector(0,length, 0));
  //Apply a Ball & Socket constraint to node0:
  BallAndSocket constraint0 = new BallAndSocket(radians(40), radians(60));
  constraint0.setRestRotation(joint0.rotation(), new Vector(1,0,0), new Vector(0,1,0));
  constraint0.setTwistLimits(radians(50), radians(50));
  joint0.setConstraint(constraint0);
  //Apply a Ball & Socket constraint to node1:
  BallAndSocket constraint1 = new BallAndSocket(radians(60), radians(40));
  constraint1.setRestRotation(joint1.rotation(), new Vector(1,0,0), new Vector(0,1,0));
  constraint1.setTwistLimits(radians(5), radians(5));
  joint1.setConstraint(constraint1);
  //Apply a Hinge constraint to node2:
  Hinge constraint2 = new Hinge(radians(40), radians(60));
  constraint2.setRestRotation(joint2.rotation(), new Vector(0,1,0), new Vector(1,0,0));
  joint2.setConstraint(constraint2);
  //Apply a Hinge constraint to node3:
  Hinge constraint3 = new Hinge(radians(60), radians(40));
  constraint3.setRestRotation(joint4.rotation(), new Vector(0,1,0), new Vector(0,0,1));
  joint3.setConstraint(constraint3);
  //---------------------------------------------------
  //Enable IK functionallity
  skeleton.enableIK();
  //Lets create a Targets indicating the name of the leaf nodes. 
  skeleton.addTarget("Joint 4");
  //If desired you could set the target position and orientation to be the same as the leaves of the structure 
  skeleton.restoreTargetsState();
  scene.enableHint(Scene.BACKGROUND | Scene.AXES);
}
void draw(){
  lights();
  scene.render();
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
