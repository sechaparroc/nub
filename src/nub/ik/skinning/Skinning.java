package nub.ik.skinning;

import nub.core.Node;
import nub.processing.Scene;
import processing.core.PGraphics;

import java.util.List;

public interface Skinning {
  void updateParams();

  void initParams();

  void render(PGraphics pg);

  void render(Scene scene);

  void render(Scene scene, Node node);

  List<Node> skeleton();
}
