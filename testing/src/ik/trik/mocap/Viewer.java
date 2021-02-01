package ik.trik.mocap;

import nub.core.Graph;
import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.animation.Skeleton;
import nub.ik.loader.bvh.BVHLoader;
import nub.ik.solver.GHIK;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;
import processing.core.PFont;
import processing.event.MouseEvent;

import java.util.*;

public class Viewer extends PApplet {
    String[] paths = new String[]{
            "/testing/data/bvh/0017_ParkourRoll001.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Walk.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Tyranno/__TailWhip.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/cmu-mocap-master/data/001/01_02.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Horse/__SlowWalk.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__SlowFly.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Run.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Eagle/__Strike1.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Puppy/Puppy_IdleEnergetic.bvh"
    };

    String path = paths[5];
    boolean absolute = true;
    Scene scene;
    BVHLoader loader;
    List<Skeleton> skeletons;
    List<String> names = new ArrayList<String>();
    List<Float> errorPerFrame = new ArrayList<Float>();
    List<Float> accumulatedError = new ArrayList<Float>();
    Collection<Integer> indices;
    boolean usePositionError = true;
    boolean drawAvgError = true;

    boolean readNext = false, solve = false;
    float sk_height = 0;

    public void settings(){
        size(1900, 1000, P3D);
    }

    public void setup(){
        PFont myFont = createFont("Times New Roman Bold", 50, true);
        textFont(myFont);

        Scene._retainedBones = false;
        //frameRate(24);
        //1. Setup Scene
        scene = new Scene(this);
        scene.setType(Graph.Type.ORTHOGRAPHIC);
        scene.eye().rotate(new Quaternion(0,0, PI));

        //2. Instantiate loader
        loader = new BVHLoader(absolute ? path : (sketchPath() + path), scene, null);
        sk_height = calculateHeight(loader);
        //Skip first two postures
        loader.nextPosture(true);
        loader.nextPosture(true);
        loader.generateConstraints();
        loader.skeleton().setRadius(scene.radius() * 0.01f);
        loader.skeleton().setBoneWidth(scene.radius() * 0.01f);
        loader.skeleton().setColor(color(0,0,255));
        loader.skeleton().setDepth(true);

        //Move the loader skeleton to the left
        //3. Create two skeletons with different solvers
        skeletons = new ArrayList<Skeleton>();
        /*Skeleton IKSkeleton1 = loader.skeleton().get();
        IKSkeleton1.setColor(color(255,0,0));
        IKSkeleton1.setDepth(true);
        IKSkeleton1.setTargetRadius(scene.radius() * 0.02f);
        IKSkeleton1.setRadius(scene.radius() * 0.01f);
        IKSkeleton1.setBoneWidth(scene.radius() * 0.01f);

        IKSkeleton1.enableIK(GHIK.HeuristicMode.CCD);
        IKSkeleton1.setMaxError(0.01f);
        IKSkeleton1.addTargets();
        skeletons.add(IKSkeleton1);
        IKSkeleton1.enableDirection(true, false);*/
        Skeleton IKSkeleton2 = loader.skeleton().get();
        //IKSkeleton2.disableConstraints();
        IKSkeleton2.setColor(color(0,255,0));
        IKSkeleton2.setDepth(true);
        IKSkeleton2.setTargetRadius(scene.radius() * 0.02f);
        IKSkeleton2.setRadius(scene.radius() * 0.01f);
        IKSkeleton2.setBoneWidth(scene.radius() * 0.01f);
        IKSkeleton2.setDepth(true);

        IKSkeleton2.enableIK(GHIK.HeuristicMode.TRIK_ECTIK);
        IKSkeleton2.enableDirection(true, true);
        IKSkeleton2.setMaxError(0.001f * sk_height);
        println("Height : " + sk_height + " Max error " + 0.01f * sk_height);
        IKSkeleton2.addTargets();
        skeletons.add(IKSkeleton2);

        //Relocate the skeletons
        //loader.skeleton().reference().translate(0,0,-height*0.5f);
        //IKSkeleton2.reference().translate(0,0,height);


        scene.setHUD(pg -> {
            pg.pushStyle();
            //pg.text("# Joints" + + IKSkeleton1.BFS().size() + " # End Effectors " + IKSkeleton1.endEffectors().size(), 50 , 50);
            //pg.text("S1 error " + IKSkeleton1.solvers().get(0).error() / IKSkeleton1.endEffectors().size(), 50 , 100);
            //pg.text("S2 error " + IKSkeleton2.solvers().get(0).error() / IKSkeleton2.endEffectors().size(), 50 , 150);
            //pg.text("FPS " + frameRate, 50 , 200);
            pg.popStyle();
            drawTimeArray(errorPerFrame, 0, height - height / 3, width, height / 3, drawAvgError ? 50 : errorPerFrame.size(), usePositionError ? sk_height * 0.2f : PI / 4, sk_height);
            if(!drawAvgError) pickFromPane(indices,0, height - height / 3, width, height / 3, errorPerFrame.size(), usePositionError ? sk_height * 0.2f : PI / 4 );
            else highlightWorst(indices, 0.8f *width, 0.1f * height,  0.2f * width,2f / 3f * height - 0.2f * height, sk_height);
            drawInfo(10, 0.1f * height, 0.2f * width, 2f / 3f * height - 0.2f * height);
        });

        //4. Set scene
        scene.setBounds(sk_height * 3);
        scene.fit(0);
        scene.enableHint(Graph.BACKGROUND, color(255));
        scene.enableHint(Graph.AXES, 10);

        //loader.skeleton().cull(true);

        //IKSkeleton1.enableIK(false);
        //IKSkeleton2.enableIK(false);

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
        if(scene.node() != null ) System.out.println(scene.node().id());
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

            for(Node skNode : loader.skeleton().BFS()){
                Node node = skeleton.joint(loader.skeleton().jointName(skNode));
                Constraint c = node.constraint();
                node.setConstraint(null);
                node.setTranslation(skNode.translation().get());
                node.setConstraint(c);
            }


            //Set the targets
            for(Map.Entry<String, Node> entry : skeleton.targets().entrySet()){
                Node desired =  loader.skeleton().joint(entry.getKey());
                entry.getValue().setTranslation(loader.skeleton().reference().location(desired));
                entry.getValue().setOrientation(loader.skeleton().reference().displacement(new Quaternion(), desired));
                skeleton.joint(entry.getKey()).setConstraint(null);
                //skeleton.joint(entry.getKey()).setRotation(loader.skeleton().joint(entry.getKey()).rotation().get());
                //skeleton.joint(entry.getKey()).setConstraint(new Constraint() {
                    //@Override
                    //public Quaternion constrainRotation(Quaternion rotation, Node node) {
                        //return new Quaternion();
                    //}
                //});
            }
            skeleton.IKStatusChanged();
            skeleton.solveIK();
        }
        if(drawAvgError){
            errorPerFrame.add(usePositionError ? avgPositionDistance() : avgRotationDistance());
            if(usePositionError) positionDistancePerJoint(accumulatedError);
            else rotationDistancePerJoint(accumulatedError);
            indices = sortIndices();
        } else{
            errorPerFrame = usePositionError ? positionDistancePerJoint(accumulatedError) : rotationDistancePerJoint(accumulatedError);
            indices = sortIndices();
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

    //Draw error at each Frame
    public List<Float> positionDistancePerJoint(List<Float> accum){
        List<Float> error = new ArrayList<Float>();
        Skeleton real = loader.skeleton();
        Skeleton computed = skeletons.get(0);
        float dist = 0;
        int i = 0;
        boolean first = accum.isEmpty();
        for (String name : real.names().values()) {
            dist = Vector.distance(real.joint(name).position(), computed.joint(name).position());
            error.add(dist);
            if(first){
                names.add(name);
                accum.add(dist);
            } else{
                accum.set(i, accum.get(i) + dist);
            }
            i++;
        }
        return error;
    }


    public List<Float> rotationDistancePerJoint(List<Float> accum){
        List<Float> error = new ArrayList<Float>();
        Skeleton real = loader.skeleton();
        Skeleton computed = skeletons.get(0);
        float dist = 0;
        int i = 0;
        boolean first = accum.isEmpty();
        for (String name : real.names().values()) {
            dist = quaternionDistance(real.joint(name).rotation(), computed.joint(name).rotation());
            error.add(dist);
            if(first){
                names.add(name);
                accum.add(dist);
            } else{
                accum.set(i, accum.get(i) + dist);
            }
            i++;
        }
        return error;
    }

    public Collection<Integer> sortIndices(){
        Map<Float, Integer> map = new TreeMap<Float, Integer>(Collections.reverseOrder());
        for (int i = 0; i < accumulatedError.size(); ++i) {
            map.put(accumulatedError.get(i), i);
        }
        return map.values();
    }

    public void drawInfo(float x, float y, float w, float h){
        int k = 0;
        int n = 8;
        pushStyle();
        textAlign(LEFT, CENTER);
        fill(0);
        float y_col = y + k++ * (1.f * h / (n + 1));
        textSize(24);
        text("Height: " + String.format("%.2f",sk_height / sk_height * 100), x + 10, y_col);
        y_col = y + k++ * (1.f * h / (n + 1));
        text("# Joints: " + loader.skeleton().joints().size(), x + 10, y_col);
        y_col = y + k++ * (1.f * h / (n + 1));
        text("# End effectors: " + loader.skeleton().endEffectors().size(), x + 10, y_col);
        y_col = y + k++ * (1.f * h / (n + 1));
        float e = skeletons.get(0).solvers().get(0).error() / skeletons.get(0).endEffectors().size();
        text("Average distance error per end effector: " + String.format("%.2f",  e / sk_height * 100), x + 10, y_col);


        popStyle();

    }

    public void highlightWorst(Collection<Integer> indices, float x, float y, float w, float h, float sk_height){
        if(indices == null) return;
        int k = 0;
        int n = 8;
        pushStyle();
        textAlign(LEFT, CENTER);
        fill(0);
        float y_col = y + k++ * (1.f * h / (n + 1));
        textSize(24);
        text("Joint location average error", x + 10, y_col);
        textSize(20);
        for(int idx : indices){
            if(k == n) break;
            //Draw a line from joint to column
            Node joint = skeletons.get(0).joint(names.get(idx));
            Vector v = scene.screenLocation(joint);
            float x_col = x;
            y_col = y + k * (1.f * h / (n + 1));
            stroke(255,0,0);
            strokeWeight(2);
            line(x_col, y_col, v.x(), v.y());
            float e = accumulatedError.get(idx) / errorPerFrame.size();
            e = e / sk_height * 100;
            text(k + " " + names.get(idx) + ": " + String.format("%.2f", e), x_col + 10, y_col);
            k++;
        }
        popStyle();
    }


    public void pickFromPane(Collection<Integer> indices, float x, float y, float w, float h, int cap, float max_v){
        if(indices == null) return;
        int k = 0;
        for(int idx : indices){
            if(k == 5) break;
            //Draw a line from joint to column
            Node joint = skeletons.get(0).joint(names.get(idx));
            Vector v = scene.screenLocation(joint);
            float x_col = x + (idx +0.5f) * w / cap;
            float value = errorPerFrame.get(idx) / max_v;
            float y_col = y + (h - (value) * h);

            stroke(255,0,0);
            strokeWeight(2);
            line(x_col, y_col, v.x(), v.y());
            k++;
        }
    }

    public float avgPositionDistance(){
        Skeleton real = loader.skeleton();
        Skeleton computed = skeletons.get(0);
        float dist = 0;
        for (String name : real.names().values()) {
            dist += Vector.distance(real.joint(name).position(), computed.joint(name).position());
        }
        return dist / real.names().size();
    }


    public float avgRotationDistance(){
        Skeleton real = loader.skeleton();
        Skeleton computed = skeletons.get(0);
        float dist = 0;
        for (String name : real.names().values()) {
            dist += quaternionDistance(real.joint(name).rotation(), computed.joint(name).rotation());
        }
        return dist / real.names().size();
    }

    public static float quaternionDistance(Quaternion a, Quaternion b) {
        float s1 = 1, s2 = 1;
        if (a.w() < 0) s1 = -1;
        if (b.w() < 0) s2 = -1;
        float dot = s1 * a._quaternion[0] * s2 * b._quaternion[0] + s1 * a._quaternion[1] * s2 * b._quaternion[1] + s1 * a._quaternion[2] * s2 * b._quaternion[2] + s1 * a._quaternion[3] * s2 * b._quaternion[3];
        dot = Math.max(Math.min(dot, 1), -1);
        return (float) (1 - Math.pow(dot, 2));
    }


    void drawTimeArray(List<Float> serie, float x, float y, float w, float h, int cap, float max_v, float sk_height){
        int n = serie.size();
        int start = cap < n ? n - cap : 0;
        push();
        fill(74, 77, 79);
        rect(x,y, w, h);
        float w_col = w / cap;
        stroke(150);
        max_v = max_v / sk_height * 100;

        for(int i = start; i < n; i++){
            float value = serie.get(i) / sk_height * 100f;
            value = value / max_v;
            float x_cur  = (i - start) * w_col;
            float y_cur = h - (value) * h;
            fill(18, 111, 204);
            rect(x + x_cur, y + y_cur, w_col, h - y_cur );
        }

        int num_ticks = 7;
        textSize(18);
        noLights();
        float ticks_step = 1f / num_ticks;
        for(int i = 1; i < num_ticks; i++){
            float value = (ticks_step * i);
            float x_cur  = x;
            float y_cur = h - (value) * h;
            fill(0);
            text("" + String.format("%.2f", value * max_v), x_cur + 5, y + y_cur);
            stroke(100);
            line(x_cur + 10, y + y_cur, x + w, y + y_cur);
        }

        noLights();
        fill(0);
        textSize(32);
        text("Joints’ location average mean absolute error", x, y - 10);
        pop();
    }


    public static void main(String args[]) {
        PApplet.main(new String[]{"ik.trik.mocap.Viewer"});
    }
}
