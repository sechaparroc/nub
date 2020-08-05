package nub.ik.loader.collada;

import nub.core.Node;
import nub.ik.loader.collada.data.Model;
import nub.ik.loader.collada.xml.XmlNode;
import nub.primitives.Matrix;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;

import java.util.List;

public class SkeletonLoader {
  private XmlNode armatureData;
  private List<String> boneOrder;
  private List<Matrix> bindMatrices;
  private int jointCount = 0;
  private float max = -1;

  public SkeletonLoader(XmlNode visualSceneNode, List<String> boneOrder) {
    this(visualSceneNode, boneOrder, null);
  }

  public SkeletonLoader(XmlNode visualSceneNode, List<String> boneOrder, List<Matrix> bindMatrices) {
    XmlNode node = visualSceneNode.getChild("visual_scene").getChildWithAttribute("node", "id", "Armature", true);
    if (node != null) {
      this.armatureData = visualSceneNode.getChild("visual_scene").getChildWithAttribute("node", "id", "Armature", true);
    } else {
      this.armatureData = visualSceneNode.getChild("visual_scene").getChild("node");
    }
    this.boneOrder = boneOrder;
    this.bindMatrices = bindMatrices;
  }


  public void extractBoneData(Model model, boolean blender) {
    XmlNode headNode = armatureData.getChild("node");
    Node root = loadJointData(headNode, model, null, blender);
    model.set_root(root);
    model.setJointCount(jointCount);
    model.setScaling(100 / max);
    //scale skeleton from children to root
    for (Node j : model.skeleton().values()) {
      j.setTranslation(Vector.multiply(j.translation(), model.scaling()));
    }
  }

  private Node loadJointData(XmlNode jointNode, Model model, Node parent, boolean blender) {
    Node joint = blender ? extractMainJointData(jointNode, model, parent) : extractMainJointTransformationData(jointNode, model, parent);
    float mag = joint.position().magnitude();
    max = max < mag ? mag : max;
    for (XmlNode childNode : jointNode.getChildren("node")) {
      loadJointData(childNode, model, joint, blender);
    }
    return joint;
  }

  private Node extractMainJointData(XmlNode jointNode, Model model, Node parent) {
    String nameId = jointNode.getAttribute("sid");
    int index = boneOrder.indexOf(nameId);

    Node joint = new Node();
    joint.enableHint(Node.CONSTRAINT);
    if(parent != null) {
      joint.enableHint(Node.BONE, Scene.pApplet.color((float)Math.random() * 255, (float)Math.random() * 255, (float)Math.random() * 255), model.getScene().radius() * 0.01f);
      joint.setReference(parent);
    }
    else{
      joint.setShape(pg ->{
        pg.pushStyle();
        pg.noStroke();
        pg.fill(-1);
        if(pg.is3D()) pg.sphere(model.getScene().radius() * 0.01f);
        else pg.ellipse(0,0, 2 * model.getScene().radius() * 0.01f, 2 * model.getScene().radius() * 0.01f);
        pg.popStyle();
      });
    }

    //use bind matrix info

    //if is possible apply bind transformation
    if (index != -1) {
      Matrix mat = bindMatrices.get(index);
      joint.fromWorldMatrix(mat);
    }
    //if not apply usual transformation
    else {
      String[] matrixRawData = jointNode.getChild("matrix").getData().split(" ");
      float[] matrixData = convertData(matrixRawData);
      Matrix jmat = new Matrix(matrixData);
      joint.fromWorldMatrix(jmat);
    }
    //joint.setPosition(matrixData[3], matrixData[7], matrixData[11]);
    //joint.setRotation(new Quaternion(mat));
    //joint.fromMatrix(mat);
    jointCount++;
    model.skeleton().put(nameId, joint);
    model.getIdxs().put(joint.id(), index);
    return joint;
  }

  private Node extractMainJointTransformationData(XmlNode jointNode, Model model, Node parent) {
    Node joint = new Node();
    joint.enableHint(Node.CONSTRAINT);
    joint.enableHint(Node.BONE, Scene.pApplet.color((float)Math.random() * 255, (float)Math.random() * 255, (float)Math.random() * 255),  model.getScene().radius() * 0.01f);

    if (parent != null) joint.setReference(parent);

    String nameId = jointNode.getAttribute("sid");
    for (XmlNode transformations : jointNode.getChildren()) {
      if (transformations.getName().equals("translate")) {
        float[] translation = convertData(transformations.getData().split(" "));
        joint.translate(translation[0], translation[1], translation[2]);
      } else if (transformations.getName().equals("rotate")) {
        float[] rotation = convertData(transformations.getData().split(" "));
        joint.rotate(new Quaternion(new Vector(rotation[0], rotation[1], rotation[2]), (float) Math.toRadians(rotation[3])));
      }
    }

    //Related geom
    List<XmlNode> geom = jointNode.getChildren("instance_geometry");
    for (XmlNode g : geom) {
      model.meshMap().put(g.getAttribute("url").substring(1), joint);
    }
    jointCount++;
    model.skeleton().put(nameId, joint);
    return joint;
  }

  private float[] convertData(String[] rawData) {
    float[] matrixData = new float[Math.min(rawData.length, 16)];
    for (int i = 0; i < matrixData.length; i++) {
      matrixData[i] = Float.parseFloat(rawData[i]);
    }
    return matrixData;
  }

}