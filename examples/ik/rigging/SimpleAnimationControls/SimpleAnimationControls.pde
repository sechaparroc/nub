/**
 * Simple Animation Controls
 * by Sebastian Chaparro Cuevas.
 *
 * In this example a mesh is loaded (shapePath) along with a skeleton (jsonPath) and the idea is to define
 * key postures at different times (represented in the timeline at the bottom of the window) that will be 
 * interpolated to produce a smooth movement.
 *
 * To do so you could interact with the scene and the time line as follows:
 * Interact with the main scene and the skeleton as usual either manipulating the skeleton itself or the IK targets.
 * Click on a key frame to set it as the current key frame. Note that the current key frame is highlighted with a green border
 * and any action performed with the keyboard will be applied to it.
 * Press 'S' to save the posture of the skeleton on the current key frame.
 * Press 'E' to enable / disable the current key frame if it contains a posture. If a key frame is enabled (green fill) 
 * then it will be used to generate the animation otherwise it will be ignored (red fill).
 * Press 'D' to delete the posture saved in the current key frame.
 * Press 'R' to play the current animation.
 * Press 'T' to stop the animation.
 * Press 'I' to invert the speed direction (if is negative the animation will run backwards).
 * Press 'L' to make the animation loop.
 * Press 'N' to load an animation from a json file (interpolatorPath)
 * Press 'M' to save an animation on a json file (interpolatorPath) 
 * Press any digit to change the sped of the animation.
 * Press the space bar to show / hide the skeleton.
 * Translate the key frames to change their time position.
 */


import nub.core.Graph;
import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.animation.Skeleton;
import nub.ik.skinning.GPULinearBlendSkinning;
import nub.ik.skinning.Skinning;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.ik.animation.PostureInterpolator;
import nub.ik.animation.Posture;

import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

    
  Scene mainScene, controlScene, focus;
  String jsonPath = "Hand_constrained.json";
  String shapePath = "Rigged_Hand.obj";
  String texturePath = "HAND_C.jpg";
  String interpolatorPath = "HAND_interpolator.json";

  AnimationPanel panel;
  Skeleton skeleton;
  Skinning skinning;
  boolean showSkeleton = true;


  public void settings() {
    size(800, 600, P3D);
  }

  public void setup() {
    //Kinematic scene
    mainScene = new Scene(createGraphics(width, height, P3D));
    mainScene.fit(0);
    mainScene.leftHanded = true;
    mainScene.enableHint(Scene.BACKGROUND | Scene.AXES);

    skeleton = new Skeleton(jsonPath);
    //skeleton.enableIK();
    skeleton.addTargets();
    skeleton.setTargetRadius(0.03f * mainScene.radius());
    //Relate the shape with a skinning method (CPU or GPU)
    skinning = new GPULinearBlendSkinning(skeleton, shapePath, texturePath, mainScene.radius());
    //Set the control scene
    controlScene = new Scene(createGraphics(width, (int) (height * 0.3), P2D)); //0, (int) (height * 0.7f)
    controlScene.setBounds(height * 0.3f / 2.f);
    controlScene.fit();
    controlScene.enableHint(Graph.BACKGROUND, color(150));

    //Setting the panel
    panel = new AnimationPanel(controlScene, skeleton);
    //set eye constraint
    controlScene.eye().tagging = false;
    controlScene.eye().setConstraint(new Constraint() {
      @Override
      public Vector constrainTranslation(Vector translation, Node node) {
        return new Vector(translation.x(), 0); //no vertical translation allowed
      }

      @Override
      public Quaternion constrainRotation(Quaternion rotation, Node node) {
        return new Quaternion(); //no rotation is allowed
      }
    });
  }


  public void draw() {
    mainScene.openContext();
    mainScene.context().lights();
    skinning.render(mainScene);
    if (showSkeleton) mainScene.render(skeleton.reference());
    mainScene.closeContext();

    mainScene.image(0,0);
    controlScene.display(panel,0, (int) (height * 0.7f));
  }


  //Interaction methods

  public void mouseMoved() {
    focus = mouseY > 0.7 * height ? controlScene : mainScene;
    focus.mouseTag();
  }

  public void mouseDragged() {
      if (mouseButton == LEFT) {
        focus.mouseSpin();
      } else if (mouseButton == RIGHT)
        focus.mouseTranslate(0);
      else
        focus.moveForward(mouseX - pmouseX);
  }


  public void mouseWheel(MouseEvent event) {
    if (focus != controlScene && focus.node() == null) focus.scale(event.getCount() * 50);
  }

  float speed = 1, direction = 1;

  public void keyPressed() {
    if (key == 'r' || key == 'R') {
      panel.play(direction * speed);
    }

    if (key == 't' || key == 'T') {
      panel.stop();
    }

    if (key == 's' || key == 'S') {
      //save skeleton posture
      panel.savePosture();
    }

    if (key == 'e' || key == 'E') {
      panel.toggleCurrentKeyPoint();
    }

    if (key == 'd' || key == 'D') {
      panel.deletePostureAtKeyPoint();
    }

    if (key == 'i' || key == 'I') {
      direction *= -1;
    }

    if (key == 'l' || key == 'L') {
      panel.enableRecurrence(!panel.isRecurrent());
    }

    if (Character.isDigit(key)) {
      speed = Float.valueOf("" + key);
    }

    if (key == ' ') {
      showSkeleton = !showSkeleton;
    }

    if(key == 'n' || key == 'N'){
      try {
        panel.loadInterpolator(this, interpolatorPath);

      } catch (Exception e){
        e.printStackTrace();
      }
    }
    if(key == 'm' || key == 'M'){
      panel.saveInterpolator(this, interpolatorPath);
    }
  }

  public void mouseClicked(MouseEvent event) {
    if (focus == mainScene) {
      if (event.getCount() == 2)
        if (event.getButton() == LEFT)
          focus.focus();
        else
          focus.align();
    } else if (focus == controlScene) {
      if (focus.node() != null) focus.interact(focus.node(),"onClicked", event.getButton());
    }
  }
