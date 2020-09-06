package ik.trik.vizSteps;

import ik.basic.Util;
import nub.core.Graph;
import nub.core.Node;
import nub.ik.solver.trik.NodeInformation;
import nub.ik.solver.trik.heuristic.BackAndForth;
import nub.ik.solver.trik.implementations.IKSolver;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.processing.TimingTask;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.util.List;

public class VizSolver extends PApplet {
  //Generate a simple Structure
  Scene scene, vizScene;
  int numJoints = 10;
  float targetRadius;
  float boneLength = 50;
  IKSolver solver;
  boolean solve = false;
  Viz viz;


  public void settings(){
    size(1900, 1000, P3D);
  }

  public void setup(){
    NodeInformation.disableCache = true;
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    targetRadius = numJoints * boneLength * 0.8f * 0.05f;

    scene.setBounds(numJoints * boneLength * 0.6f);
    scene.leftHanded = false;
    scene.fit(0);
    //scene.enableHint(Graph.AXES);
    scene.enableHint(Graph.BACKGROUND, color(255));

    //Generate auxiliar scene
    vizScene = new Scene(createGraphics(width, height, P3D));
    vizScene.setType(Graph.Type.ORTHOGRAPHIC);
    vizScene.setBounds(numJoints * boneLength * 0.6f);
    vizScene.leftHanded = false;
    vizScene.fit(0);

    //vizScene.enableHint(Graph.AXES);
    vizScene.enableHint(Graph.BACKGROUND, color(255));

    vizScene.context().smooth(8);

    //Create viz helper
    viz = new Viz(vizScene);
    viz.path = "C:/Users/olgaa/Desktop/Sebas/Thesis/Results/Viz/COMBINED";
    viz.name = "combined";

    //Create target
    Node target = Util.createTarget(scene, targetRadius);
    List<Node> structure = Util.generateAttachedChain(numJoints, 0.7f * targetRadius, boneLength, new Vector(0, -0.25f * numJoints * boneLength, 0), 0, 100, 150);
    Util.generateConstraints(structure, Util.ConstraintType.HINGE_ALIGNED, 13, scene.is3D());

    solver = new IKSolver(structure, IKSolver.HeuristicMode.COMBINED, true);
    solver.setHeuristic(new TriangulationViz(  solver.context(), viz));
    solver.setMaxError(-10f);
    solver.setMinDistance(-10f);
    solver.setTimesPerFrame(1);
    solver.setMaxIterations(1);
    solver.setTarget(structure.get(numJoints - 1), target);
    solver.context().setTopToBottom(false);
    target.setPosition(structure.get(numJoints - 1).position());
    //Set the original color structure
    for(Node node : structure){
      node._boneColor = color(255,255,0, 100);
    }
    TimingTask task = new TimingTask() {
      @Override
      public void execute() {
        if (solve) solver.solve();
      }
    };
    task.run(40);
  }

  public void draw() {
    ambientLight(102, 102, 102);
    lightSpecular(204, 204, 204);
    directionalLight(102, 102, 102, 0, -0.1f, 0.4f);
    specular(255, 255, 255);
    shininess(10);
    scene.render();

    scene.beginHUD();
    pushStyle();
    text(mouseX + ", " + mouseY, mouseX, mouseY);
    noFill();
    rect(viz.x_min, viz.y_min, viz.x_max - viz.x_min, viz.y_max - viz.y_min);
    popStyle();
    scene.endHUD();
  }


  public void keyPressed() {
    if (key == 'w' || key == 'W') {
      solve = !solve;
    }

    if (key == 's' || key == 'S') {
      vizScene.eye().set(scene.eye());
      viz.resetBounds();
      solver.solve();
    }

    if(key == 'p' || key == 'P'){
      viz.saveFrames();
    }

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
    PApplet.main(new String[]{"ik.trik.vizSteps.VizSolver"});
  }

}
