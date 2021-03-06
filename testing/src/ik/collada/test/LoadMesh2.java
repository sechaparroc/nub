package ik.collada.test;

import ik.interactive.Target;
import nub.core.Graph;
import nub.core.Node;
import nub.ik.loader.collada.BlenderLoader;
import nub.ik.loader.collada.data.Model;
import nub.ik.skinning.GPULinearBlendSkinning;
import nub.ik.solver.Solver;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by sebchaparr on 18/05/19.
 */
public class LoadMesh2 extends PApplet {
  Scene scene;
  String path = "/testing/data/dae/";
  String dae = "dummy.dae";
  String tex = null;
  Model model;
  GPULinearBlendSkinning skinning;
  Solver solver;

  public void settings() {
    size(700, 700, P3D);
  }

  public void setup() {
    randomSeed(14);
    textSize(24);
    //Joint.markers = true;
    //1. Create the scene
    textureMode(NORMAL);
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);

    //2. Load the model
    model = BlenderLoader.loadColladaModel(sketchPath() + path, dae, tex, scene, 3);

    //3. Setup scene
    scene.setBounds(model.mesh().getWidth() * 2);
    scene.eye().rotate(new Quaternion(new Vector(1, 0, 0), PI / 2));
    scene.eye().rotate(new Quaternion(new Vector(0, 0, 1), PI));
    scene.fit();

    //4. Relate mesh and skinning
    skinning = new GPULinearBlendSkinning(model.structure(), model.mesh());

    //5. Adding IK behavior
    //Register an IK Solver
    solver = scene.registerTreeSolver(model.root());
    //Update params
    solver.setMaxError(1f);

    //solver = scene.registerTreeSolver(model.skeleton().get("Bone_004"));
    //Update params
    solver.setMaxError(1f);

    //solver = scene.registerTreeSolver(model.skeleton().get("Bone_009"));

    //Update params
    solver.setMaxError(1f);
    //Identify end effectors (leaf nodes)
    List<Node> endEffectors = new ArrayList<>();
    for (String s : model.skeleton().keySet()) {
      Node node = model.skeleton().get(s);
      if (s.equals("Bone_020") || s.equals("Bone_016") || s.equals("Bone_008") || s.equals("Bone_007")) {
        endEffectors.add(node);
        // Create targets
        Target target = new Target(scene.radius() * 0.02f);
        //target.setReference(root);
        target.setPosition(node.position().get());
        //add IK target to solver
        scene.addIKTarget(node, target);
      }
    }
  }

  public void draw() {
    background(0);
    lights();
    scene.drawAxes();

    //Render mesh
    skinning.render(scene);
    //Render skeleton
    hint(DISABLE_DEPTH_TEST);
    scene.render();
    hint(ENABLE_DEPTH_TEST);

    scene.beginHUD();
    for (String s : model.skeleton().keySet()) {
      Node n = model.skeleton().get(s);
      if (n.cull || !n.tagging) continue;
      Vector sc = scene.screenLocation(new Vector(), n);
      text(s, sc.x(), sc.y());
    }
    scene.endHUD();

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
      scene.scale(scene.mouseDX());
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
    PApplet.main(new String[]{"ik.collada.test.LoadMesh2"});
  }
}
