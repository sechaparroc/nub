package nub.ik.loader.collada;

import nub.core.Node;
import nub.ik.loader.collada.data.Mesh;
import nub.ik.loader.collada.data.Model;
import nub.ik.loader.collada.xml.XmlNode;
import nub.ik.loader.collada.xml.XmlParser;
import nub.processing.Scene;
import processing.core.PShape;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class URDFLoader {
  public static Model loadColladaModel(String colladaFile, String dae, Scene scene) {
    XmlNode node = XmlParser.loadXmlFile(colladaFile + dae);
    Model model = new Model(scene);

    SkeletonLoader jointsLoader = new SkeletonLoader(node.getChild("library_visual_scenes"), null);
    jointsLoader.extractBoneData(model, false);

    GeometryLoader g = new GeometryLoader(node.getChild("library_geometries"), null);
    List<Mesh> meshes = g.extractURDFModelData(model.scaling());
    int i = 0;
    List<XmlNode> xmlNodes = node.getChild("library_geometries").getChildren("geometry");
    float max = -1;

    HashMap<Node, List<PShape>> shapes = new HashMap<Node, List<PShape>>();
    for (Mesh mesh : meshes) {
      PShape pshape = mesh.generatePShape(scene.context(), null);
      String id = xmlNodes.get(i++).getAttribute("id");
      model.addModel(id, pshape);
      max = max < pshape.getWidth() ? pshape.getWidth() : max;
      Node joint = model.meshMap().get(id);
      pshape.setFill(scene.context().color(joint._boneColor));
      if(shapes.containsKey(joint)){
        shapes.get(joint).add(pshape);
      } else{
        ArrayList<PShape> list = new ArrayList<PShape>();
        list.add(pshape);
        shapes.put(joint, list);
      }
    }

    for(Map.Entry<Node, List<PShape>> entry : shapes.entrySet()){
      entry.getKey().setShape(pg ->{
        for(PShape s : entry.getValue()){
          pg.shape(s);
        }
      });
    }

    scene.setBounds(max * 5f);
    for (Node joint : model.skeleton().values()) {
      joint._boneRadius = scene.radius() * 0.03f;
      joint.setBullsEyeSize(-0.03f);
    }

    return model;
  }
}