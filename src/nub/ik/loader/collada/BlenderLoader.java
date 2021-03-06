package nub.ik.loader.collada;

import nub.core.Node;
import nub.ik.loader.collada.data.Mesh;
import nub.ik.loader.collada.data.Model;
import nub.ik.loader.collada.data.SkinningData;
import nub.ik.loader.collada.xml.XmlNode;
import nub.ik.loader.collada.xml.XmlParser;
import nub.primitives.Matrix;
import nub.processing.Scene;
import processing.core.PImage;

public class BlenderLoader {
  /**
   * Load a Collada model from Blender
   * This parser is usable only for quite simple structures
   */

  public static Model loadColladaModel(String colladaFile, String dae, String tex, Scene scene, int maxWeights) {
    XmlNode node = XmlParser.loadXmlFile(colladaFile + dae);

    Model model = new Model(scene);

    SkinLoader skinLoader = new SkinLoader(node.getChild("library_controllers"), maxWeights);
    SkinningData skinningData = skinLoader.extractSkinData();
    SkeletonLoader jointsLoader = new SkeletonLoader(node.getChild("library_visual_scenes"), skinningData.jointOrder, skinningData.bindMatrices);
    jointsLoader.extractBoneData(model, true);

    XmlNode bind = node.getChild("library_controllers").getChild("controller").getChild("skin").getChild("bind_shape_matrix");
    String[] bind_data = bind.getData().split(" ");
    float[] mat = new float[bind_data.length];

    for (int i = 0; i < bind_data.length; i++) {
      mat[i] = Float.parseFloat(bind_data[i]);
    }
    Matrix m = new Matrix(mat, false);

    GeometryLoader g = new GeometryLoader(node.getChild("library_geometries"), skinningData.verticesSkinData, m);
    Mesh meshData = g.extractBlenderModelData(model.scaling());
    model.addModel(null, meshData.generatePShape(scene.context(), tex == null ? null : colladaFile + tex));


    if (tex == null) {
      //set a dummy texture
      PImage img = scene.context().parent.createImage(1, 1, scene.context().parent.ARGB);
      img.loadPixels();
      for (int i = 0; i < img.pixels.length; i++) {
        img.pixels[i] = scene.context().color(222, 184, 135);
      }
      model.mesh().setTexture(img);
    }

    scene.setBounds(100);
    for (Node joint : scene.nodes()) {
      joint._boneRadius = scene.radius() * 0.03f;
      joint.setBullsEyeSize(-0.03f);
    }

    return model;
  }
}