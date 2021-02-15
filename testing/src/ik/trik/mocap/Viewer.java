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
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Puppy/Puppy_IdleEnergetic.bvh",
            //BVH used in Paper
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Cat/CAT_TPOSE.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Cat/CAT_IdlePurr.bvh", //9
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Cat/CAT_LickCleanIdle.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Cat/CAT_StretchYawnIdle.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Cat/CAT_Walk.bvh",


            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/cmu-mocap-master/data/001/01_03.bvh", // 14
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/cmu-mocap-master/data/013/13_17.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/cmu-mocap-master/data/090/90_02.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/cmu-mocap-master/data/090/90_05.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/cmu-mocap-master/data/055/55_02.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/cmu-mocap-master/data/090/90_28.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/cmu-mocap-master/data/088/88_01.bvh",
            "C:/Users/olgaa/Documents/Processing/libraries/nub/examples/BVHReconstruction/0017_ParkourRoll001.bvh",

            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__Idle.bvh",//Dragon 22
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__Idle2.bvh",//Dragon 22
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__Idle3.bvh",//Dragon 22
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__Attack.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__Attack2.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__Attack3.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__Die.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__Fly.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Dragon/__SlowFly.bvh",

            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Attack1.bvh", //Monkey 31
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Attack2.bvh", //Monkey 32
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Attack3.bvh", //Monkey 33
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Attack4.bvh", //Monkey 34
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__B1Die.bvh", //Monkey 35
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Idle1.bvh", // 36
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Idle2.bvh", // 37
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Idle3.bvh", // 38
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Run.bvh", // 39
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Walk.bvh", // 40
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Die.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Run.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Monkey/__Walk.bvh",

            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Attack.bvh", //Spider 44
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Attack3.bvh", //Spider 45
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Attack7.bvh", //Spider 46
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Burrough.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__ComeOut.bvh", //48
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Fangs.bvh", //49
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Run.bvh", //50
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Walk.bvh", //51
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Die.bvh", //52
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Idle.bvh", //53
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__Idle2.bvh", //54
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/SpiderG/__TPOSE.bvh", //55

            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Tyranno/__TPOSE.bvh",// Tyranno 56
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Tyranno/__Attack.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Tyranno/__Attack2.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Tyranno/__Fall.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Tyranno/__HeadButt.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Tyranno/__Idle.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Tyranno/__Idle2.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Tyranno/__Walk.bvh",
            "C:/Users/olgaa/Desktop/Sebas/Thesis/BVH_FILES/truebones/Truebone_Z-OO/Tyranno/__Run.bvh",

    };

    String path = paths[12];
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
    boolean useFABRIK = false;
    int skip =1;
    String modelName = "cat";

    boolean readNext = false, solve = false;
    float sk_height = 0;

    float avg_error = 0;

    boolean useWristAnkles = true;
    boolean showInfo = false, showParams = false;

    public void settings(){
        fullScreen(P3D);
        //size(1900, 1000, P3D);
    }

    public void setup(){
        frameRate(15);
        PFont myFont = createFont("Times New Roman Bold", 50, true);
        textFont(myFont);

        Scene._retainedBones = false; //Set to true to increase speed
        //frameRate(24);
        //1. Setup Scene
        scene = new Scene(this);
        scene.setType(Graph.Type.ORTHOGRAPHIC);
        scene.eye().rotate(new Quaternion(0,0, PI));

        //2. Instantiate loader
        loader = new BVHLoader(absolute ? path : (sketchPath() + path), scene, null);
        humanoidEFFs(loader);
        //dragonEFFs(loader);
        //catEFFs(loader);
        loader.nextPosture(true);
        loader.nextPosture(true);

        sk_height = calculateHeight(loader, false);

        scene.setBounds(sk_height * 3);

        scene.fit(0);

        //Skip first two postures
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

        IKSkeleton1.enableIK(GHIK.HeuristicMode.TRIK_ECTIK);
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

        IKSkeleton2.enableIK(GHIK.HeuristicMode.BFIK);
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
            if(showInfo) {
                drawTimeArray(errorPerFrame, 0, height - height / 4, width, height / 4, drawAvgError ? 50 : errorPerFrame.size(), usePositionError ? sk_height * 0.2f : PI / 4, sk_height);
                if (!drawAvgError)
                    pickFromPane(indices, 0, height - height / 4, width, height / 4, errorPerFrame.size(), usePositionError ? sk_height * 0.2f : PI / 4);
                else
                    highlightWorst(indices, 0.8f * width, 0.1f * height, 0.2f * width, 3f / 4f * height - 0.2f * height, sk_height);
            }
            drawInfo(10, 0.1f * height, 0.2f * width, 3f / 4f * height - 0.2f * height);
        });

        //4. Set scene
        scene.enableHint(Graph.BACKGROUND, color(0));
        scene.enableHint(Graph.AXES, sk_height * 3);

        //loader.skeleton().cull(true);

        //IKSkeleton1.enableIK(false);
        //IKSkeleton2.enableIK(false);
        scene.eye().setConstraint(new Constraint() {
            @Override
            public Quaternion constrainRotation(Quaternion rotation, Node node) {
                return new Quaternion(0,rotation.eulerAngles().y(),0);
            }
        });
    }

    public void draw(){
        ambientLight(102, 102, 102);
        lightSpecular(204, 204, 204);
        directionalLight(102, 102, 102, 0, 0, -1);
        specular(255, 255, 255);
        shininess(10);
        scene.render();

        if(readNext) readNextPosture(skip);
        if(record){
            saveFrame("C:/Users/olgaa/Desktop/Img" + frameCount + ".png");
            record = false;
        }

    }

    boolean record = false;
    public void keyPressed(){
        if(key == 'W' || key == 'w'){
            readNext = !readNext;
        }
        if(key == 'S' || key == 's'){
            readNextPosture(skip);
        }
        if(key == 'R' || key == 'r'){
            record = !record;
        }
        if(key == 'h' || key == 'H'){
            toggleHints();
        }
        if(key == 'j' || key == 'J'){
            loader.skeleton().reference().cull = !loader.skeleton().reference().cull;
        }
        if(key == 'k' || key == 'k'){
            showInfo = !showInfo;
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
        if(scene.node() != null ){
            System.out.println("Node name : " + skeletons.get(0).jointName(scene.node()));
            for(Node child : scene.node().children()){
                System.out.println("---> Node " + skeletons.get(0).jointName(child));

            }

            System.out.println(scene.node().id() + " " + scene.node().position());
        }
        if(event.getCount() == 2)
            if(event.getButton() == LEFT)
                scene.focus();
            else
                scene.align();
    }

    void readNextPosture(int skip){
        for(int i = 0; i < skip; i++)
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
                //EFF own rotation is known
                //skeleton.joint(entry.getKey()).setRotation(desired.rotation().get());
                entry.getValue().setTranslation(loader.skeleton().reference().location(desired));
                entry.getValue().setOrientation(loader.skeleton().reference().displacement(new Quaternion(), desired));
            }
            skeleton.IKStatusChanged();
            skeleton.solveIK();
        }
        if(drawAvgError){
            errorPerFrame.add(usePositionError ? avgPositionDistance() : avgRotationDistance());
            avg_error += avgPositionDistance();
            if(usePositionError) positionDistancePerJoint(accumulatedError);
            else rotationDistancePerJoint(accumulatedError);
            indices = sortIndices();
        } else{
            errorPerFrame = usePositionError ? positionDistancePerJoint(accumulatedError) : rotationDistancePerJoint(accumulatedError);
            indices = sortIndices();
        }
    }


    //Some useful functions
    float calculateHeight(BVHLoader parser, boolean useY){ //calculates the height of the skeleton
        Vector min = new Vector(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
        Vector max = new Vector(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);;
        for(Node n : parser.skeleton().joints().values()){
            //Vector pos = parser.skeleton().reference().children().get(0).location(n);
            Vector pos = n.position().get();
            if(n.children().isEmpty())System.out.println("pos " + parser.skeleton().jointName(n) + " " +  pos);
            if(max.x() < pos.x()) max.setX(pos.x());
            if(max.y() < pos.y()) max.setY(pos.y());
            if(max.z() < pos.z()) max.setZ(pos.z());
            if(min.x() > pos.x()) min.setX(pos.x());
            if(min.y() > pos.y()) min.setY(pos.y());
            if(min.z() > pos.z()) min.setZ(pos.z());
        }
        System.out.println("max" + max + " min " + min);
        float mX = max.x() - min.x();
        float mY = max.y() - min.y();
        float mZ = max.z() - min.z();
        System.out.println("mX" + mX + " mY " + mY + "mZ" + mZ);
        return mY;
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
        fill(255);
        float y_col = y + k++ * (1.f * h / (n + 1));
        textSize(24);
        text("Height: " + String.format("%.2f",sk_height / sk_height * 100), x + 10, y_col);
        y_col = y + k++ * (1.f * h / (n + 1));
        fill(0,0,255);
        ellipse(x + 22, y_col + 2, 20, 20);
        fill(255);
        text("Original", x + 44, y_col);
        y_col = y + k++ * (1.f * h / (n + 1));
        fill(0,255,0);
        ellipse(x + 22, y_col + 2, 20, 20);
        fill(255);
        text("Reconstruction", x + 44, y_col);

        y_col = y + k++ * (1.f * h / (n + 1));
        text("# Joints: " + loader.skeleton().joints().size(), x + 10, y_col);
        y_col = y + k++ * (1.f * h / (n + 1));
        text("# End effectors: " + loader.skeleton().endEffectors().size(), x + 10, y_col);
        y_col = y + k++ * (1.f * h / (n + 1));
        float e = skeletons.get(0).solvers().get(0).error() / skeletons.get(0).endEffectors().size();
        e = Math.max(0, e - 0.001f * sk_height );
        text("Average distance error per end effector: " + String.format("%.2f",  e / sk_height * 100) + "%", x + 10, y_col);




        popStyle();

    }

    public void highlightWorst(Collection<Integer> indices, float x, float y, float w, float h, float sk_height){
        if(indices == null) return;
        int k = 0;
        int n = 8;
        pushStyle();
        textAlign(LEFT, CENTER);
        fill(255);
        float y_col = y + k++ * (1.f * h / (n + 1));
        textSize(24);
        text("Joints with worst position error: ", x + 10, y_col);
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

        int num_ticks = 8;
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
        fill(255);
        textSize(32);
        if(errorPerFrame.size() > 0)
        text("Mean per Joint position error: ( Avg " + String.format("%.2f", (avg_error / sk_height * 100f) / errorPerFrame.size()) + "%)", x, y - 10);
        pop();
    }

    public void removeChildren(Skeleton sk, Node node){
        while(!node.children().isEmpty()){
            Node child = node.children().get(0);
            removeChildren(sk, child);
            sk.joints().remove(sk.jointName(child));
            sk.names().remove(child);
            child.setReference(null);
            Scene.prune(child);
        }
    }


    public void removeChildren(BVHLoader loader, String names[]){
        Skeleton sk = loader.skeleton();
        for(String name : names){
            if(sk.joints().containsKey(name) && useWristAnkles) {
                removeChildren(sk, sk.joint(name));
            }
        }
    }

    public void toggleHints(){
        for(Node n : skeletons.get(0).joints().values()){
            if(showParams){
                n.enableHint(Node.AXES);
                n.enableHint(Node.CONSTRAINT);
            } else{
                n.disableHint(Node.AXES);
                n.disableHint(Node.CONSTRAINT);
            }
        }
        for(Node n : loader.skeleton().joints().values()){
            if(showParams){
                n.enableHint(Node.AXES);
                n.enableHint(Node.CONSTRAINT);
            } else{
                n.disableHint(Node.AXES);
                n.disableHint(Node.CONSTRAINT);
            }
        }
        showParams = !showParams;
    }

    public void obtainEFFS(BVHLoader loader, String names[]){
        Skeleton sk = loader.skeleton();
        for(String name : names){
            if(sk.joints().containsKey(name) && useWristAnkles) {
                Node n = sk.joint(name);
                sk.joints().remove(name);
                sk.names().remove(n);
                n.setReference(null);
                Scene.prune(n);
            }
        }
    }

    public void humanoidEFFs(BVHLoader loader){
        String names[] = new String[]{"RTHUMB", "RIGHTHANDINDEX1", "LTHUMB", "LEFTTOEBASE", "RIGHTTOEBASE", "LEFTHANDINDEX1", "LEFTFINGERBASE", "RIGHTFINGERBASE"};
        obtainEFFS(loader, names);
    }

    public void catEFFs(BVHLoader loader){
        Skeleton sk = loader.skeleton();
        String names[] = new String[]{"BIP01_R_TOE0NUB", "BIP01_L_TOE0NUB", "BIP01_R_FINGER0NUB", "BIP01_L_FINGER0NUB", "BIP01_R_TOE0", "BIP01_L_TOE0", "BIP01_R_FINGER0", "BIP01_L_FINGER0"};
        obtainEFFS(loader, names);
        for(Node node : loader.skeleton().BFS()){
            if(node.children() == null || node.children().isEmpty() )System.out.println("Name : " + loader.skeleton().jointName(node));
        }
    }

    public void dragonEFFs(BVHLoader loader){
        String names[] = new String[]{"BIP01_L_FOOT", "BIP01_R_FOOT", "BIP01_R_HAND", "BIP01_L_HAND"};
        removeChildren(loader, names);
        for(Node node : loader.skeleton().BFS()){
            if(node.children() == null || node.children().isEmpty() )System.out.println("Name : " + loader.skeleton().jointName(node));
        }
    }


    public static void main(String args[]) {
        PApplet.main(new String[]{"ik.trik.mocap.Viewer"});
    }
}
