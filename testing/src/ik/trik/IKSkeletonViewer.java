package ik.trik;

import nub.core.Graph;
import nub.core.Node;
import nub.ik.loader.bvh.BVHLoader;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.event.MouseEvent;

public class IKSkeletonViewer  extends PApplet {
  int id = 4;
  String[] paths = new String[]{
      "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__SlowFly.bvh",
      "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Walk.bvh",
      "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Cat/CAT_Walk.bvh",
      "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Tyranno/__Walk.bvh",
      "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Walk.bvh"

  };

  String path = paths[id];
  boolean absolute = true;
  Scene scene;
  BVHLoader loader;

  String pathImg ="C:/Users/olgaa/Desktop/Sebas/Thesis/Results/Viz/Structure/";
  String name = "Bear";
  int idx = 0;

  boolean readNext = false;

  public void settings(){
    size(1900, 1000, P3D);
  }

  public void setup(){
    int[] colors = new int[]{
      color(0,0,255),
      color(204, 255, 179),
      color(224, 89, 36),
      color(199, 46, 199),
      color(199, 196, 46)
    };

    //1. Setup Scene
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.eye().rotate(new Quaternion(0,0, PI));

    //2. Instantiate loader
    loader = new BVHLoader(absolute ? path : (sketchPath() + path), scene, null);
    float height = calculateHeight(loader);
    //Skip first two postures
    loader.nextPosture(true);
    loader.nextPosture(true);
    loader.generateConstraints();
    scene.setBounds(height);
    scene.fit(0);

    loader.skeleton().setColor(colors[id]);
    loader.skeleton().setRadius(scene.radius() * 0.02f);
    //loader.skeleton().setBoneWidth(scene.radius() * 0.01f);
    loader.skeleton().setConstraintFactor(0.4f);
    loader.skeleton().setDepth(true);
    int c_joints = 0;
    int c_effs = 0;
    for(Node node : loader.skeleton().BFS()){
      if(node.children() == null || node.children().isEmpty()) c_effs++;
      c_joints++;
    }

    final int n_joints = c_joints;
    final int n_effs = c_effs;

    scene.setHUD(pg -> {
      pg.pushStyle();
      pg.text("Joints " + n_joints, 50 , 100);
      pg.text("Effs " + n_effs, 50 , 150);
      pg.text("Height " + height, 50 , 200);
      pg.text("FPS " + frameRate, 50 , 250);
      pg.popStyle();
    });

    //4. Set scene
    scene.enableHint(Graph.BACKGROUND, color(255));
    scene.enableHint(Graph.AXES);
  }

  public void draw(){
    ambientLight(102, 102, 102);
    lightSpecular(204, 204, 204);
    directionalLight(102, 102, 102, 0, -0.1f, 0.4f);
    specular(255, 255, 255);
    shininess(10);
    scene.render();
    if(readNext) readNextPosture();
  }

  public void keyPressed(){
    if(key == 'W' || key == 'w'){
      readNext = !readNext;
    }
    if(key == 'S' || key == 's'){
      readNextPosture();
    }
    if(key == 'p' || key == 'P'){
      scene.context().save( pathImg + "/" + name + idx++ + ".jpg");
    }
  }

  public void mouseMoved(){
    scene.mouseTag();
  }

  public void mouseDragged(){
    if(mouseButton == LEFT){
      scene.mouseSpin();
    } else if(mouseButton == RIGHT){
      scene.mouseTranslate();
    } else{
      scene.scale(scene.mouseDX());
    }
  }

  public void mouseWheel(MouseEvent event){
    scene.scale(event.getCount() * 20);
  }

  public void mouseClicked(MouseEvent event){
    if(event.getCount() == 2)
      if(event.getButton() == LEFT)
        scene.focus();
      else
        scene.align();
  }

  void readNextPosture(){
    loader.nextPosture();
  }


  //Some useful functions
  float calculateHeight(BVHLoader parser){ //calculates the height of the skeleton
    Vector min = new Vector(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
    Vector max = Vector.multiply(min, -1);
    for(Node n : parser.skeleton().BFS()){
      Vector pos = parser.skeleton().reference().children().get(0).location(n);
      if(max.x() < pos.x()) max.setX(pos.x());
      if(max.y() < pos.y()) max.setY(pos.y());
      if(max.z() < pos.z()) max.setZ(pos.z());
      if(min.x() > pos.x()) min.setX(pos.x());
      if(min.y() > pos.y()) min.setY(pos.y());
      if(min.z() > pos.z()) min.setZ(pos.z());
    }
    float mX = max.x() - min.x();
    float mY = max.y() - min.y();
    float mZ = max.z() - min.z();
    return Math.max(Math.max(mX, mY), mZ);
  }

  public static void main(String args[]) {
    PApplet.main(new String[]{"ik.trik.IKSkeletonViewer"});
  }
}
