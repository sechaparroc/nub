package nub.ik.loader.bvh;

import nub.core.Node;
import nub.core.constraint.AxisPlaneConstraint;
import nub.core.constraint.Constraint;
import nub.core.constraint.SphericalPolygon;
import nub.ik.animation.Joint;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PApplet;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
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
    //TODO : Update
    protected BufferedReader _buffer;
    //A Joint is a Node with some Properties
    //TODO: Consider _id() public?
    protected HashMap<Integer, Properties> _joint;
    protected int _frames;
    protected int _period;
    protected Class<? extends Node> _class;
    protected Node _root;
    protected List<Node> _branch;
    protected HashMap<Integer, ArrayList<Node>> _poses;
    protected int _currentPose;
    protected boolean _loop;
    protected float _radius;


    public BVHLoader(String path, Node reference) {
        _class = Joint.class;
        _radius = 5f;
        _setup(path, reference);
    }


    public BVHLoader(String path, Scene scene, Node reference) {
        _class = Joint.class;
        _radius = scene.radius() * 0.01f;
        _setup(path, reference);
    }

    public BVHLoader(Class<? extends Node> nodeClass, String path, Node reference) {
        _class = nodeClass;
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

    public List<Node> branch() {
        return _branch;
    }

    public int poses() {
        return _poses.get(_root.id()).size();
    }

    public Node root() {
        return _root;
    }

    public int currentPose() {
        return _currentPose;
    }

    protected void _setup(String path, Node reference) {
        _frames = 0;
        _period = 0;
        _currentPose = 0;
        _joint = new HashMap<>();
        _poses = new HashMap<>();
        _loop = true;
        _readHeader(path, reference);
        _saveFrames();
    }


    /**
     * Reads a .bvh file from the given path and builds
     * A Hierarchy of Nodes given by the .bvh header
     */
    protected Node _readHeader(String path, Node reference) {
        Node root = null;
        try {
            _buffer = new BufferedReader(new FileReader(path));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return null;
        }
        Node current = root;
        Node currentRoot = reference;
        Properties currentProperties = null;
        boolean boneBraceOpened = false;
        //READ ALL THE HEADER
        String line = "";
        while (!line.contains("FRAME_TIME") &&
                !line.contains("FRAME TIME")) {
            try {
                line = _buffer.readLine();
                if (line == null) return root;
                line = line.toUpperCase();
            } catch (IOException e) {
                e.printStackTrace();
                return null;
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
                if (root != null) {
                    //TODO : Allow multiple ROOTS
                    while (!line.equals("MOTION")) {
                        try {
                            line = _buffer.readLine();
                        } catch (IOException e) {
                            e.printStackTrace();
                            return null;
                        }
                        //clean line
                        line.replace("\t", " ");
                        line.replace("\n", "");
                        line.replace("\r", "");
                        line.replace(":", " ");
                    }
                    return root;
                }
                //Create a Frame
                try {
                    root = _class.getConstructor().newInstance();

                } catch (InstantiationException e) {
                    e.printStackTrace();
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                } catch (NoSuchMethodException e) {
                    e.printStackTrace();
                } catch (InvocationTargetException e) {
                    e.printStackTrace();
                }
                root.setReference(reference);
                current = root;
                currentRoot = root;
                currentProperties = new Properties(expression[1]);
                if (root instanceof Joint) {
                    ((Joint) current).setName(expression[1]);
                    ((Joint) current).setRadius(_radius);
                    ((Joint) current).setColor(255, 255, 255);
                }
                _joint.put(current.id(), currentProperties);
                _poses.put(current.id(), new ArrayList<>());
                boneBraceOpened = true;
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
                try {
                    current = _class.getConstructor().newInstance();
                    if (current instanceof Joint) {
                        ((Joint) current).setRadius(_radius);
                        ((Joint) current).setColor(255, 255, 255);
                    }
                } catch (InstantiationException e) {
                    e.printStackTrace();
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                } catch (NoSuchMethodException e) {
                    e.printStackTrace();
                } catch (InvocationTargetException e) {
                    e.printStackTrace();
                }
                current.setReference(currentRoot);
                currentRoot = current;
                currentProperties = new Properties(expression[1]);
                _joint.put(current.id(), currentProperties);
                _poses.put(current.id(), new ArrayList<>());
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
        _root = root;
        if (root instanceof Joint) ((Joint) root).setRoot(true);

        _branch = Scene.branch(_root);
        return root;
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

        //clean line
        line.replace("\t", " ");
        line.replace("\n", "");
        line.replace("\r", "");
        line = line.trim();
        //split
        String[] expression = line.split(" ");
        //traverse each line
        int i = 0;
        for (Node current : _branch) {
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
            // Use the shortest rotation between two quaternions
            // See: https://stackoverflow.com/questions/2886606/flipping-issue-when-interpolating-rotations-using-quaternions
            Node prev = _poses.get(current.id()).size() > 0 ? _poses.get(current.id()).get(_poses.get(current.id()).size() - 1) : current;
            if (Quaternion.dot(rotation, prev.rotation()) < 0) {
                // change sign
                rotation.negate();
            }
            Node next = Node.detach(current.translation().get(), current.rotation().get(), 1);
            if (rotationInfo) {
                next.setRotation(rotation);
            }
            if (translationInfo) {
                next.setTranslation(translation);
            }
            _poses.get(current.id()).add(next);
        }
        return true;
    }

    public void nextPose(boolean remove) {
        if (_currentPose >= _poses.get(_root.id()).size()) {
            if (_loop) _currentPose = 0;
            else return;
        }
        for (Node node : _branch) {
            Constraint c = node.constraint();
            //node.setConstraint(null);
            node.setRotation(_poses.get(node.id()).get(_currentPose).rotation().get());
            node.setTranslation(_poses.get(node.id()).get(_currentPose).translation().get());
            node.setConstraint(c);
            if (remove) _poses.get(node.id()).remove(_currentPose);
        }
        if (!remove) _currentPose++;
    }

    public void nextPose() {
        nextPose(false);
    }

    public void poseAt(int idx) {
        if (idx >= _poses.get(_root.id()).size()) {
            return;
        }
        for (Node node : _branch) {
            Constraint c = node.constraint();
            node.setConstraint(null);
            node.setRotation(_poses.get(node.id()).get(idx).rotation().get());
            node.setTranslation(_poses.get(node.id()).get(idx).translation().get());
            node.setConstraint(c);
        }
        _currentPose = idx;
    }

    protected Vector findRestVector(Node node) {
        List<Node> poses = _poses.get(node.id());
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
        Quaternion restRotation = node.rotation().get();
        for (Node keyNode : poses) {
            Quaternion delta = Quaternion.compose(restRotation.inverse(), keyNode.rotation());
            delta.normalize();
            rest.add(delta.rotate(init));
        }

        if (rest.magnitude() < 0.001f) //pick any vector
            rest = init;


        rest.multiply(1f / (poses.size() + 1));
        rest.normalize();
        return rest.get();
    }

    public void generateConstraints() {
        for (Node node : _branch) {
            if (node == _root) continue;
            generateConstraint(node);
        }
    }



    protected void generateConstraint(Node node) {
        Vector rest = findRestVector(node);
        List<Node> poses = _poses.get(node.id());
        Quaternion restRotation = node.rotation().get();
        Vector up = rest.orthogonalVector();
        Vector right = Vector.cross(rest, up, null);

        if (node.children() == null || node.children().isEmpty()) {
            return;
        }
        if(node.translation().magnitude() < 10) System.out.println("Low trans : " + node.translation().magnitude());


        float minTwist = 0, maxTwist = 0;
        float upAngle = 0, downAngle = 0, leftAngle = 0, rightAngle = 0;

        for (Node keyNode : poses) {
            Quaternion delta = Quaternion.compose(restRotation.inverse(), keyNode.rotation());
            delta.normalize();
            if (Quaternion.dot(delta, restRotation) < 0) {
                delta.negate();
            }

            Quaternion deltaRest = decomposeQuaternion(delta, rest);

            float restAng = calcAngle(rest , deltaRest);

            if(restAng > 0)maxTwist = Math.max(restAng, maxTwist);
            else minTwist = Math.max(-restAng, minTwist);

            Vector restUp = Vector.projectVectorOnPlane(delta.rotate(rest), up);
            Quaternion q = new Quaternion(rest, restUp);
            Quaternion deltaUp = decomposeQuaternion(delta, up);
            float upAng = calcAngle(up , deltaUp);

            if(upAng > 0)upAngle = Math.max(upAng, upAngle);
            else downAngle = Math.max(-upAng, downAngle);

            Vector restright = Vector.projectVectorOnPlane(delta.rotate(rest), right);
            q = new Quaternion(rest, restright);
            Quaternion deltaRight = decomposeQuaternion(delta, right);
            float rightAng = calcAngle(right , deltaRight);

            if(rightAng > 0)rightAngle = Math.max(rightAng, rightAngle);
            else leftAngle = Math.max(-rightAng, leftAngle);
        }

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

