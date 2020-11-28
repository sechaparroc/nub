/*
BVH Reconstruction using GHIK Tree
By Sebastian Chaparro Cuevas

This example shows the performance of the IK Solver when reconstructing the motion of an Articulated Body from a .bvh file.
Several experiments were conducted using:
1. The Tree Bones Zoo Free Pack. visit https://gumroad.com/truebones/p/free-truebones-zoo-over-75-animals-and-animations.
   It will ask for a method of Payment but, you can omit this using the freecode: truebones4freefree.
2. SFU Motion Capture Database -- Available at -- http://mocap.cs.sfu.ca/
3. CMU mocap dataset in bvh format -- Available at -- https://github.com/una-dinosauria/cmu-mocap
*/


import nub.core.*;
import nub.core.constraint.*;
import nub.ik.animation.Skeleton;
import nub.ik.loader.bvh.BVHLoader;
import nub.ik.solver.GHIK;
import nub.primitives.*;
import nub.processing.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

//Place here the absolute path of a BVH File.
boolean absolute_path = false; //Define if the path is absolute or relative to the sketch location
String path = "0017_ParkourRoll001.bvh";

Scene scene;
BVHLoader loader;
Skeleton IKSkeleton;
boolean readNext = false, solve = false;

public void settings(){
    size(800, 600, P3D);
}

public void setup(){
    Scene._retainedBones = false; //Comment this line for a better appeareance (as it renders in immediate mode, the sketch run slower)...
    //1. Setup Scene
    scene = new Scene(this);
    scene.setType(Graph.Type.ORTHOGRAPHIC);
    scene.eye().rotate(new Quaternion(0,0, PI));
    //2. Instantiate loader
    loader = new BVHLoader(absolute_path ? path : (sketchPath() + "/" + path), scene, null);
    float skeletonHeight = calculateHeight(loader);
    //Skip first two postures (idle pose)
    loader.nextPosture(true);
    loader.nextPosture(true);
    loader.generateConstraints();
    loader.skeleton().setRadius(scene.radius() * 0.01f);
    loader.skeleton().setBoneWidth(scene.radius() * 0.01f);
    //Move the loader skeleton to the left
    //3. Create an IK Solver
    IKSkeleton = loader.skeleton().get(); //Generate a copy
    //IKSkeleton2.disableConstraints();
    IKSkeleton.setColor(color(0,255,0));
    IKSkeleton.setDepth(true);
    IKSkeleton.setTargetRadius(scene.radius() * 0.02f);
    IKSkeleton.setRadius(scene.radius() * 0.01f);
    IKSkeleton.setBoneWidth(scene.radius() * 0.01f);
    IKSkeleton.enableIK(GHIK.HeuristicMode.TRIK_ECTIK);
    IKSkeleton.enableDirection(true); //Enable - Disable target direction.
    IKSkeleton.setMaxError(0.001f * skeletonHeight);
    IKSkeleton.addTargets();
    //Relocate the skeletons
    loader.skeleton().reference().translate(0,0,-skeletonHeight * 2f);
    IKSkeleton.reference().translate(0,0,skeletonHeight * 2f);
    //4. Set scene
    scene.setBounds(skeletonHeight * 3);
    scene.fit(0);
    scene.enableHint(Graph.BACKGROUND | Graph.AXES);
    //loader.skeleton().cull(true); //uncomment this line if you dont want to show the original animation
    //IKSkeleton.enableIK(false); //Disable IK Task as it is called explicitly
}

public void draw(){
    ambientLight(102, 102, 102);
    lightSpecular(204, 204, 204);
    directionalLight(102, 102, 102, 0, 0, -1);
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
    //move the root of the skeleton
    for(int i = 0; i < loader.skeleton().reference().children().size(); i++){
        Node skeletonRoot = loader.skeleton().reference().children().get(i);
        Node root = IKSkeleton.joint(loader.skeleton().jointName(skeletonRoot));
        Constraint c = root.constraint();
        root.setConstraint(null);
        root.setTranslation(skeletonRoot.translation().get());
        root.setRotation(skeletonRoot.rotation().get());
        root.setConstraint(c);
    }
    for(Node skNode : loader.skeleton().BFS()){
        Node node = IKSkeleton.joint(loader.skeleton().jointName(skNode));
        Constraint c = node.constraint();
        node.setConstraint(null);
        node.setTranslation(skNode.translation().get());
        node.setConstraint(c);
    }
    //Set the targets
    for(Map.Entry<String, Node> entry : IKSkeleton.targets().entrySet()){
        Node desired =  loader.skeleton().joint(entry.getKey());
        entry.getValue().setTranslation(loader.skeleton().reference().location(desired));
        entry.getValue().setOrientation(loader.skeleton().reference().displacement(new Quaternion(), desired));
    }
    IKSkeleton.IKStatusChanged();
    IKSkeleton.solveIK();
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
