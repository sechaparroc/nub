package ik.trik.mocap;

import nub.core.Graph;
import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.animation.Skeleton;
import nub.ik.loader.bvh.BVHLoader;
import nub.ik.solver.trik.implementations.IKSolver;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class Viewer extends PApplet {
    String[] paths = new String[]{
            "/testing/data/bvh/0007_Cartwheel001.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__SlowFly.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/cmu-mocap-master/data/001/01_02.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Horse/__SlowWalk.bvh"
    };

    String path = paths[1];
    boolean absolute = true;
    Scene scene;
    BVHLoader loader;
    List<Skeleton> skeletons;

    boolean readNext = false, solve = false;

    public void settings(){
        size(800, 600, P3D);
    }

    public void setup(){
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
        //Move the loader skeleton to the left
        //3. Create two skeletons with different solvers
        skeletons = new ArrayList<Skeleton>();
        Skeleton IKSkeleton1 = loader.skeleton().get();
        IKSkeleton1.setColor(color(255,0,0));
        IKSkeleton1.setDepth(true);
        IKSkeleton1.setTargetRadius(scene.radius() * 0.01f);
        IKSkeleton1.enableIK(IKSolver.HeuristicMode.COMBINED_TRIK);
        IKSkeleton1.addTargets();
        skeletons.add(IKSkeleton1);

        Skeleton IKSkeleton2 = loader.skeleton().get();
        IKSkeleton2.setColor(color(0,255,0));
        IKSkeleton2.setDepth(true);
        IKSkeleton2.setTargetRadius(scene.radius() * 0.01f);
        IKSkeleton2.enableIK(IKSolver.HeuristicMode.CCD);
        IKSkeleton2.addTargets();
        skeletons.add(IKSkeleton2);

        //Relocate the skeletons
        loader.skeleton().reference().translate(0,0,-height * 2f);
        IKSkeleton2.reference().translate(0,0,height * 2f);


        //4. Set scene
        scene.setRadius(height * 3);
        scene.fit(0);
        scene.enableHint(Graph.BACKGROUND | Graph.AXES);

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
        for(Skeleton skeleton : skeletons){
            //move the root of the skeleton
            for(int i = 0; i < loader.skeleton().reference().children().size(); i++){
                Node skeletonRoot = loader.skeleton().reference().children().get(i);
                Node root = skeleton.joint(loader.skeleton().jointName(skeletonRoot));
                Constraint c = root.constraint();
                root.setConstraint(null);
                root.setTranslation(skeletonRoot.translation().get());
                root.setRotation(skeletonRoot.rotation().get());
                root.setConstraint(c);
            }
            //Set the targets
            for(Map.Entry<String, Node> entry : skeleton.targets().entrySet()){
                Node desired =  loader.skeleton().joint(entry.getKey());
                entry.getValue().setTranslation(loader.skeleton().reference().location(desired));
                entry.getValue().setOrientation(loader.skeleton().reference().displacement(new Quaternion(), desired));
            }
            skeleton.IKStatusChanged();
        }
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
        PApplet.main(new String[]{"ik.trik.mocap.Viewer"});
    }
}
