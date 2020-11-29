
package ik.trik.animation;

import nub.core.Graph;
import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.animation.Skeleton;
import nub.ik.skinning.GPULinearBlendSkinning;
import nub.ik.skinning.Skinning;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.event.MouseEvent;

public class AnimationTest extends PApplet {
  Scene mainScene, controlScene, focus;
  String jsonPath = "/testing/data/skeletons/Dummy_constrained.json";
  String shapePath = "/testing/data/objs/Rigged_Hand.obj";
  String texturePath = "/testing/data/objs/HAND_C.jpg";
  AnimationPanel panel;
  Skeleton skeleton;
  Skinning skinning;
  boolean showSkeleton = true;


  public void settings() {
    size(1200, 800, P3D);
  }

  public void setup() {
    //Kinematic scene
    mainScene = new Scene(createGraphics(width, height, P3D));
    mainScene.fit(0);
    mainScene.leftHanded = false;
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

  public static void main(String[] args) {
    PApplet.main(new String[]{"ik.trik.animation.AnimationTest"});
  }


}
