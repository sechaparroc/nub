package nub.ik.loader.bvh;

import nub.core.Node;
import nub.core.constraint.AxisPlaneConstraint;
import nub.core.constraint.Constraint;
import nub.core.constraint.SphericalPolygon;
import nub.ik.animation.Posture;
import nub.ik.animation.Skeleton;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * This class must be used to load a bvh file and
 * generate an animation.
 * <p>
 * For more info look at http://www.dcs.shef.ac.uk/intranet/research/public/resmes/CS0111.pdf
 * Created by sebchaparr on 23/03/18.
 */
public class BVHLoader {
    protected BufferedReader _buffer;
    //A Joint is a Node with some Properties
    protected int _frames;
    protected int _period;
    protected Skeleton _skeleton;
    protected List<Posture> _postures;
    protected HashMap<Integer, Properties> _joint;
    protected int _currentPosture;
    protected boolean _loop;
    protected float _radius;


    public BVHLoader(String path, Node reference) {
        _radius = 5f;
        _setup(path, reference);
    }

    public BVHLoader(String path, Scene scene, Node reference) {
        _radius = scene.radius() * 0.01f;
        _setup(path, reference);
    }

    public HashMap<Integer, Properties> joint() {
        return _joint;
    }

    public class Properties {
        protected String _name;
        protected int channels;
        protected List<String> _channelType;

        Properties(String name) {
            _name = name;
            channels = 0;
            _channelType = new ArrayList<String>();
        }

        public String name() {
            return _name;
        }

        public boolean addChannelType(String type) {
            return _channelType.add(type);
        }
    }

    public int postures() {
        return _postures.size();
    }

    public int currentPosture() {
        return _currentPosture;
    }

    public Skeleton skeleton(){
        return _skeleton;
    }

    protected void _setup(String path, Node reference) {
        _frames = 0;
        _period = 0;
        _currentPosture = 0;
        _joint = new HashMap<>();
        _loop = true;
        _skeleton = new Skeleton(reference);
        _postures = new ArrayList<Posture>();
        _readHeader(path);
        _saveFrames();
    }


    /**
     * Reads a .bvh file from the given path and builds
     * A Hierarchy of Nodes given by the .bvh header
     */
    protected void _readHeader(String path) {
        try {
            _buffer = new BufferedReader(new FileReader(path));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return;
        }
        Node current = null;
        Node currentRoot = _skeleton.reference();
        Properties currentProperties = null;
        boolean boneBraceOpened = false;
        boolean rootRead = false;
        //READ ALL THE HEADER
        String line = "";
        while (!line.contains("FRAME_TIME") &&
                !line.contains("FRAME TIME")) {
            try {
                line = _buffer.readLine();
                if (line == null) return;
                line = line.toUpperCase();
            } catch (IOException e) {
                e.printStackTrace();
                return;
            }
            //clean line
            line.replace("\t", " ");
            line.replace("\n", "");
            line.replace("\r", "");
            line.replace(":", "");
            line = line.trim();
            line = line.toUpperCase();
            //split
            String[] expression = line.split(" ");
            //Check Type
            if (expression[0].equals("ROOT")) {
                if (rootRead) {
                    //TODO : Allow multiple ROOTS
                    while (!line.equals("MOTION")) {
                        try {
                            line = _buffer.readLine();
                        } catch (IOException e) {
                            e.printStackTrace();
                            return;
                        }
                        //clean line
                        line.replace("\t", " ");
                        line.replace("\n", "");
                        line.replace("\r", "");
                        line.replace(":", " ");
                    }
                    return;
                }
                //Create a Joint and add to the skeleton
                current = _skeleton.addJoint(expression[1], -1,_radius);
                currentRoot = current;
                currentProperties = new Properties(expression[1]);
                _joint.put(current.id(), currentProperties);
                boneBraceOpened = true;
                rootRead = true;
            } else if (expression[0].equals("OFFSET")) {
                if (!boneBraceOpened) continue;
                float x = Float.valueOf(expression[1]);
                float y = Float.valueOf(expression[2]);
                float z = Float.valueOf(expression[3]);
                current.setTranslation(x, y, z);
            } else if (expression[0].equals("CHANNELS")) {
                currentProperties.channels = Integer.valueOf(expression[1]);
                for (int i = 0; i < currentProperties.channels; i++)
                    currentProperties.addChannelType(expression[i + 2]);
            } else if (expression[0].equals("JOINT")) {
                //Create a node
                current = _skeleton.addJoint(expression[1], _skeleton.jointName(currentRoot), -1, _radius);
                currentRoot = current;
                currentProperties = new Properties(expression[1]);
                _joint.put(current.id(), currentProperties);
                boneBraceOpened = true;
            } else if (expression[0].equals("END_SITE")) {
                boneBraceOpened = false;
            } else if (expression[0].equals("}")) {
                if (boneBraceOpened) {
                    currentRoot = currentRoot.reference();
                } else {
                    boneBraceOpened = true;
                }
            } else if (expression[0].equals("FRAMES")) {
                _frames = Integer.valueOf(expression[1]);
            } else if (expression[0].equals("FRAME_TIME")) {
                _period = Integer.valueOf(expression[1]);
            } else if (expression.length >= 2) {
                if ((expression[0] + " " + expression[1]).equals("END SITE")) {
                    boneBraceOpened = false;
                }
            } else if (expression.length >= 3) {
                if ((expression[0] + " " + expression[1]).equals("FRAME TIME")) {
                    _period = Integer.valueOf(expression[2]);
                }
            }
        }
    }

    public void setLoop(boolean loop){
      _loop = loop;
    }

    /*Saves Frame info to be read in a later stage*/

    protected void _saveFrames() {
        boolean next = true;
        while (next)
            next = _readNextFrame();
    }

    protected boolean _readNextFrame() {
        //READ JUST ONE LINE
        String line = "";
        try {
            line = _buffer.readLine();
            if (line == null) return false;
            line = line.toUpperCase();
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
        Posture posture = new Posture(_skeleton);

        //clean line
        line.replace("\t", " ");
        line.replace("\n", "");
        line.replace("\r", "");
        line = line.trim();
        //split
        String[] expression = line.split(" ");
        //traverse each line
        int i = 0;
        for (Node current : Scene.branch(_skeleton.reference())) {
            if(current == _skeleton.reference()) continue;
            Properties properties = _joint.get(current.id());
            boolean translationInfo = false;
            boolean rotationInfo = false;
            Vector translation = new Vector();
            Quaternion rotation = new Quaternion();
            Vector euler_params = new Vector();

            for (String channel : properties._channelType) {
                float value = Float.valueOf(expression[i]);
                switch (channel) {
                    case "XPOSITION": {
                        translationInfo = true;
                        translation.setX(value);
                        break;
                    }
                    case "YPOSITION": {
                        translation.setY(value);
                        break;
                    }
                    case "ZPOSITION": {
                        translation.setZ(value);
                        break;
                    }
                    case "ZROTATION": {
                        rotationInfo = true;
                        rotation.compose(new Quaternion(new Vector(0, 0, 1), PApplet.radians(value)));
                        euler_params.setZ(PApplet.radians(value));
                        break;
                    }
                    case "YROTATION": {
                        rotation.compose(new Quaternion(new Vector(0, 1, 0), PApplet.radians(value)));
                        euler_params.setY(PApplet.radians(value));
                        break;
                    }
                    case "XROTATION": {
                        rotation.compose(new Quaternion(new Vector(1, 0, 0), PApplet.radians(value)));
                        euler_params.setX(PApplet.radians(value));
                        break;
                    }
                }
                i++;
            }
            String name = _skeleton.jointName(current);
            // Use the shortest rotation between two quaternions
            // See: https://stackoverflow.com/questions/2886606/flipping-issue-when-interpolating-rotations-using-quaternions
            Node prev = _postures.size() > 0 ? _postures.get(_postures.size() - 1).jointState(name) : current;
            if (Quaternion.dot(rotation, prev.rotation()) < 0) {
                // change sign
                rotation.negate();
            }
            if(rotationInfo) posture.jointState(name).setRotation(rotation);
            if(translationInfo) posture.jointState(name).setTranslation(translation);
        }
        _postures.add(posture);
        return true;
    }

    public void nextPosture(boolean remove) {
        if (_currentPosture >= _postures.size()) {
            if (_loop) _currentPosture = 0;
            else return;
        }
        _postures.get(_currentPosture).loadValues(_skeleton);
        if (remove) _postures.remove(_currentPosture);
        if (!remove) _currentPosture++;
    }

    public void nextPosture() {
        nextPosture(false);
    }

    public void postureAt(int idx) {
        if (idx >= _postures.size()) {
            return;
        }
        _postures.get(idx).loadValues(_skeleton);
        _currentPosture = idx;
    }

    protected Vector findRestVector(Node node) {
        Vector init = new Vector(0, 1, 0); //use any vector
        Vector rest = init.get();
        if (node.children().size() == 1) {
            rest = node.children().get(0).translation().get();
            rest.normalize();
            if (rest.magnitude() > 0.1)
                return rest;
        }
        if (node.children().size() > 1) {
            Vector centroid = new Vector();
            for (Node child : node.children()) {
                centroid.add(child.translation().get());
            }
            centroid.divide(1f / node.children().size());
            if (centroid.magnitude() > 0.1) {
                centroid.normalize();
                return centroid;
            }
        }
        String name = _skeleton.jointName(node);
        Quaternion restRotation = node.rotation().get();
        for (int i = 0; i < _postures.size(); i++) {
            Node keyNode = _postures.get(i).jointState(name);
            Quaternion delta = Quaternion.compose(restRotation.inverse(), keyNode.rotation());
            delta.normalize();
            rest.add(delta.rotate(init));
        }

        if (rest.magnitude() < 0.001f) //pick any vector
            rest = init;


        rest.multiply(1f / (_postures.size() + 1));
        rest.normalize();
        return rest.get();
    }

    public void generateConstraints() {
        for (Node node : _skeleton.BFS()) {
            if (node.reference() == _skeleton.reference()) continue;
            generateConstraint(node);
        }
    }



    protected void generateConstraint(Node node) {
        Vector rest = findRestVector(node);
        Quaternion restRotation = node.rotation().get();
        Vector up = rest.orthogonalVector();
        Vector right = Vector.cross(rest, up, null);

        /*if (node.children() == null || node.children().isEmpty()) {
            return;
        }*/

        float minTwist = 0, maxTwist = 0;
        float upAngle = 0, downAngle = 0, leftAngle = 0, rightAngle = 0;

        String name = _skeleton.jointName(node);
        for (int i = 0; i < _postures.size(); i++) {
            Node keyNode = _postures.get(i).jointState(name);
            Quaternion delta = Quaternion.compose(restRotation.inverse(), keyNode.rotation());
            delta.normalize();
            if (Quaternion.dot(delta, restRotation) < 0) {
                delta.negate();
            }

            Quaternion deltaRest = decomposeQuaternion(delta, rest);

            float restAng = calcAngle(rest , deltaRest);

            if(restAng > 0)maxTwist = Math.max(restAng, maxTwist);
            else minTwist = Math.max(-restAng, minTwist);

            Quaternion deltaUp = decomposeQuaternion(delta, up);
            float upAng = calcAngle(up , deltaUp);

            if(upAng > 0)upAngle = Math.max(upAng, upAngle);
            else downAngle = Math.max(-upAng, downAngle);

            Quaternion deltaRight = decomposeQuaternion(delta, right);
            float rightAng = calcAngle(right , deltaRight);

            if(rightAng > 0)rightAngle = Math.max(rightAng, rightAngle);
            else leftAngle = Math.max(-rightAng, leftAngle);
        }
        if(upAngle + downAngle + leftAngle + rightAngle + minTwist + maxTwist < 0.00001f){
            node.setConstraint(new Constraint() {
                @Override
                public Quaternion constrainRotation(Quaternion rotation, Node node) {
                    return new Quaternion();
                }
            });
        } else {
            //Clamp angles between 5 and 178 degrees
            upAngle = Math.min(Math.max(upAngle + (float) Math.toRadians(3), (float) Math.toRadians(5)), (float) Math.toRadians(178));
            downAngle = Math.min(Math.max(downAngle + (float) Math.toRadians(3), (float) Math.toRadians(5)), (float) Math.toRadians(178));
            leftAngle = Math.min(Math.max(leftAngle + (float) Math.toRadians(3), (float) Math.toRadians(5)), (float) Math.toRadians(178));
            rightAngle = Math.min(Math.max(rightAngle + (float) Math.toRadians(3), (float) Math.toRadians(5)), (float) Math.toRadians(178));
            minTwist = Math.min(Math.max(minTwist + (float) Math.toRadians(3), (float) Math.toRadians(5)), (float) Math.toRadians(178));
            maxTwist = Math.min(Math.max(maxTwist + (float) Math.toRadians(3), (float) Math.toRadians(5)), (float) Math.toRadians(178));

            SphericalPolygon constraint = new SphericalPolygon(downAngle, upAngle, leftAngle, rightAngle);
            constraint.setTranslationConstraintType(AxisPlaneConstraint.Type.FREE);
            constraint.setRestRotation(restRotation.get(), right.get(), rest.get());
            constraint.setTwistLimits(minTwist, maxTwist);
            node.setConstraint(constraint);
        }
    }

    protected float calcAngle(Vector axis, Quaternion q){
        axis = q.inverseRotate(axis);
        Vector rotAxis = q.axis();
        float angle = q.angle();
        if (rotAxis.dot(axis) < 0) angle = -angle;
        if(Math.abs(angle) > Math.PI) //express as shortest quaternion
            angle = (float) (angle - Math.signum(angle) * 2 * Math.PI);
        return angle;
    }


    protected Quaternion decomposeQuaternion(Quaternion quaternion, Vector axis) {
        //pass axis to local space
        axis = quaternion.inverseRotate(axis);
        Vector rotationAxis = new Vector(quaternion._quaternion[0], quaternion._quaternion[1], quaternion._quaternion[2]);
        rotationAxis = Vector.projectVectorOnAxis(rotationAxis, axis); // w.r.t idle
        //Get rotation component on Axis direction
        return new Quaternion(rotationAxis.x(), rotationAxis.y(), rotationAxis.z(), quaternion.w()); //w.r.t rest
    }
}

