package ik.trik.vizSteps;

import ik.basic.Util;
import nub.core.Graph;
import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.solver.NodeInformation;
import nub.ik.solver.heuristic.BackAndForth;
import nub.ik.solver.GHIK;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import nub.processing.TimingTask;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class VizSolver extends PApplet {
  //Generate a simple Structure
  Scene scene;
  int numJoints = 5;
  float targetRadius;
  float boneLength = 50;
  Node target;
  HashMap<GHIK, Viz> visualizers = new HashMap<>();
  ArrayList<GHIK> solvers = new ArrayList<>();
  boolean solve = false;


  public void createSolver(String path, String name, String solverType, Util.ConstraintType constraintType, int seed, int color){
    //Generate auxiliar scene
    Scene vizScene = new Scene(createGraphics(width, height, P3D));
    vizScene.setType(Graph.Type.ORTHOGRAPHIC);
    vizScene.setBounds(numJoints * boneLength * 0.6f);
    vizScene.leftHanded = false;
    vizScene.fit(0);
    //vizScene.enableHint(Graph.AXES);
    vizScene.enableHint(Graph.BACKGROUND, color(255));
    vizScene.context().smooth(8);
    //Create viz helper
    Viz viz = new Viz(vizScene);
    viz.path = path;
    viz.name = name;
    List<Node> structure = Util.generateAttachedChain(numJoints, 0.7f * targetRadius, boneLength, new Vector(0, -0.25f * numJoints * boneLength, 0), 0, 100, 150);
    structure.get(structure.size()-1).setConstraint(new Constraint() {
      @Override
      public Vector constrainTranslation(Vector translation, Node node) {
        return new Vector();
      }

      @Override
      public Quaternion constrainRotation(Quaternion rotation, Node node) {
        return new Quaternion();
      }
    });

    Util.generateConstraints(structure, constraintType, seed, scene.is3D());

    GHIK solver = new GHIK(structure, GHIK.HeuristicMode.TRIK, true);
    switch (solverType){
      case "CCD":{
        solver.setHeuristic(new CCDViz(solver.context(), viz));
        break;
      }
      case "TIK":{
        solver.setHeuristic(new TriangulationViz(solver.context(), viz));
        break;
      }
      case "TRIK":{
        solver.setHeuristic(new TRIKViz(solver.context(), viz));
        break;
      }
      case "BFCCD":{
        solver.setHeuristic(new BackAndForthViz(solver.context(), BackAndForth.Mode.CCD,viz));
        break;
      }
      case "BFTIK":{
        solver.setHeuristic(new BackAndForthViz(solver.context(), BackAndForth.Mode.TRIANGULATION,viz));
        break;
      }
      case "BFTRIK":{
        solver.setHeuristic(new BackAndForthViz(solver.context(), BackAndForth.Mode.TRIK,viz));
        break;
      }
      case "ECTIK":{
        solver.setHeuristic(new CombinedViz(solver.context(), viz));
        break;
      }
    }
    solver.setMaxError(-10f);
    solver.setMinDistance(-10f);
    solver.setTimesPerFrame(1);
    solver.setMaxIterations(1);
    solver.setTarget(structure.get(numJoints - 1), target);
    solver.context().setTopToBottom(false);
    target.setPosition(structure.get(numJoints - 1).position());
    //Set the original color structure
    for(Node node : structure){
      node._boneColor = color;
    }
    TimingTask task = new TimingTask() {
      @Override
      public void execute() {
        if (solve) solver.solve();
      }
    };
    task.run(40);
    solvers.add(solver);
    visualizers.put(solver, viz);
  }


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
    String path = "C:/Users/olgaa/Desktop/Sebas/Thesis/Results/Viz";
    //Create target
    target = Util.createTarget(scene, targetRadius);

    Util.ConstraintType type = Util.ConstraintType.NONE;

    createSolver(path  + "/COMBINED/ECTIK", "ECTIK", "ECTIK",
        type, 13, color(255,255,0, 100));
    /*createSolver(path  + "/BACK_AND_FORTH/Last/BFTIK", "BF", "BFTIK",
        type, 13, color(255,255,0, 100));
    createSolver(path  + "/BACK_AND_FORTH/Last/BFTRIK", "BF", "BFTRIK",
        type, 13, color(255,255,0, 100));
    createSolver(path  + "/BACK_AND_FORTH/Last/CCD", "BF", "CCD",
        type, 13, color(255,255,0, 100));
    createSolver(path  + "/BACK_AND_FORTH/Last/TRIK", "BF", "TRIK",
        type, 13, color(255,255,0, 100));
    createSolver(path  + "/BACK_AND_FORTH/Last/TIK", "BF", "TIK",
        type, 13, color(255,255,0, 100));*/

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
    Viz viz = visualizers.get(solvers.get(0));
    rect(viz.x_min, viz.y_min, viz.x_max - viz.x_min, viz.y_max - viz.y_min);
    popStyle();
    scene.endHUD();
  }


  public void keyPressed() {
    if (key == 'w' || key == 'W') {
      solve = !solve;
    }

    if (key == 's' || key == 'S') {
      for(GHIK s : solvers){
        Viz viz = visualizers.get(s);
        viz.scene.eye().set(scene.eye());
        viz.resetBounds();
        s.solve();
      }
    }

    if(key == 'p' || key == 'P'){
      for(GHIK s : solvers) {
        Viz viz = visualizers.get(s);
        viz.saveFrames();
      }
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
