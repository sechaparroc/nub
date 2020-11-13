package ik.trik.vizSteps;

import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.solver.trik.heuristic.Heuristic;
import nub.primitives.Quaternion;
import nub.primitives.Vector;
import nub.processing.Scene;
import processing.core.PConstants;
import processing.core.PGraphics;
import processing.core.PImage;

import java.util.*;

public class Viz {
  static class Image{
    String title;
    PImage pimage;
    public Image(String title, PImage img){
      this.title = title;
      this.pimage = img;
    }

  }

  static class HighlightNode{
    String name;
    int color;
    Node node;

    public HighlightNode(String name, Node node, int color){
      this.name = name;
      this.node = node;
      this.color = color;
    }

  }

  static class Shape{
    Vector[] vertices;
    int color;
    int textCol;
    String name;

    public Shape(String name, Vector[] vertices, int color, int textCol){
      this.name = name;
      this.vertices = vertices;
      this.color = color;
      this.textCol = textCol;
    }

    public Shape(String name, Vector[] vertices, int color){
      this(name, vertices, color, Scene.pApplet.color(0));
    }
  }

  boolean showAnnotations, first = true; //fist is used due to an annoying bug :/
  String path, name;
  String title;
  public Scene scene;
  public List<HighlightNode> highlightingNodes = new ArrayList<HighlightNode>();
  public List<Shape> arrows = new ArrayList<Shape>();
  public List<Shape> shapes = new ArrayList<Shape>();
  public List<Shape> arcs = new ArrayList<Shape>();
  public List<Vector> drawingNames = new ArrayList<Vector>();

  int x_min = Integer.MAX_VALUE, x_max = -Integer.MAX_VALUE, y_min = Integer.MAX_VALUE, y_max = -Integer.MAX_VALUE;

  public List<Image> sequence = new ArrayList<Image>();

  //Store some structures copies to facilitate visualization
  int[] fromIdx = new int[5];
  public List<List<Node>> structureCopies = new ArrayList<List<Node>>(5);

  Heuristic heuristic;

  public void setHeuristic(Heuristic h){
    heuristic = h;
    //Generate structure copies
    for(int i = 0; i < 10; i++){
      List<Node> copy = h.context()._attachedCopy(h.context().chain(), null);

      for(Node node : copy){
        node.setConstraint(null); //remove constraints
        node.disableHint(Node.AXES);//disable axes
      }

      structureCopies.add(copy);
      copy.get(0).cull = true;
      setColor(copy, Scene.pApplet.color(0,255,255,100));
    }
  }

  public void setColorCopy(int i, int color){
    setColor(structureCopies.get(i), color);
  }

  public void setColor(List<Node> chain, int color){
    for(Node node : chain){
      node._boneColor = color;
    }
  }

  public void showStructureCopy(int i, int from){
    fromIdx[i] = from;
  }

  public void hideStructureCopy(int i){
    fromIdx[i] = -1;
  }

  public void hideStructureCopies(){
    for(int i = 0; i < fromIdx.length; i++)
      fromIdx[i] = -1;
  }

  public void addHighlightNode(String name, Node node, int color){
    highlightingNodes.add(new HighlightNode(name, node, color));
  }

  public void unhighlightNode(Node node){
    HighlightNode h = null;
    for(HighlightNode n : highlightingNodes) {
      if(n.node == node){
        h = n;
        break;
      }
    }
    if(h != null) highlightingNodes.remove(h);
  }


  public void clearHighlightingNodes(){
    highlightingNodes.clear();
  }

  public void updateStructureCopy(int idx){
    List<Node> origin = heuristic.context().usableChain();
    List<Node> dest = structureCopies.get(idx);
    //Copy the content of the origin chain into dest
    Node refDest = dest.get(0).reference();
    if (refDest != null) {
      refDest.set(origin.get(0).reference());
    }

    for (int i = 0; i < origin.size(); i++) {
      Node node = origin.get(i);
      Constraint constraint = dest.get(i).constraint();
      Quaternion rotation = node.rotation().get();
      Vector translation = node.translation().get();

      dest.get(i).setConstraint(null);
      dest.get(i).setRotation(rotation);
      dest.get(i).setTranslation(translation);
      dest.get(i).setScaling(node.scaling());
      dest.get(i).setConstraint(constraint);
    }
  }

  public Viz(Scene scene){
    this.scene = scene;

    //setup shapes
    this.scene.setShape(pGraphics -> {
      pGraphics.hint(PConstants.DISABLE_DEPTH_TEST);
      pGraphics.pushStyle();
      pGraphics.noStroke();
      for(HighlightNode highlightNode : highlightingNodes){
        pGraphics.fill(highlightNode.color);
        pGraphics.pushMatrix();
        Vector v = highlightNode.node.position();
        pGraphics.translate(v.x(), v.y(),v.z());
        pGraphics.sphere(highlightNode.node._boneRadius * 1.2f);
        pGraphics.popMatrix();
      }
      pGraphics.popStyle();
      pGraphics.hint(PConstants.ENABLE_DEPTH_TEST);
    });

    this.scene.setHUD(pGraphics -> {
      pGraphics.noLights();
      //draw shapes
      for(Shape shape : shapes){
        drawShape(pGraphics, shape);
      }
      //draw arrows text
      arrows.sort((o1, o2) -> {
        Vector v1 = Vector.subtract(o1.vertices[1], o1.vertices[0]);
        Vector v2 = Vector.subtract(o2.vertices[1], o2.vertices[0]);
        return (int)(-v1.magnitude() + v2.magnitude());
      });

      for(Shape arrow : arrows){
        drawArrow(pGraphics, arrow);
      }
      //draw arcs
      for(Shape arc : arcs){
        drawArc(pGraphics, arc);
      }

      //Draw names of highlighting nodes
      for(HighlightNode highlightNode : highlightingNodes){
        drawName(pGraphics,  highlightNode);
      }

      //draw title
      pGraphics.pushStyle();
      pGraphics.stroke(0);
      pGraphics.fill(0);
      pGraphics.textSize(14);
      pGraphics.textAlign(PConstants.LEFT, PConstants.TOP);
      pGraphics.text(title, 10, 10);
      pGraphics.popStyle();
    });
     }

  public void clearFigures(){
    arrows.clear();
    arcs.clear();
    shapes.clear();
    highlightingNodes.clear();
    hideStructureCopies();
  }

  public void clearArcs(){
    arcs.clear();
  }

  public void clearArrows(){
    arrows.clear();
  }

  public void addShape(String name, int color, Vector... vertices){
    shapes.add(new Shape(name,vertices, color));
  }

  public void addArrow(String name, Vector init, Vector end, int color, int textCol){
    arrows.add(new Shape(name, new Vector[]{init, end}, color, textCol));
  }

  public void addArc(String name, Vector anchor, Vector init, Vector end, int color){
    arcs.add(new Shape(name, new Vector[]{anchor, init, end}, color));
  }


  public PImage drawFrame(String name){
    drawingNames.clear();
    //1. render the scene at that time
    title = name;
    scene.openContext();
    scene.context().ambientLight(102, 102, 102);
    scene.context().lightSpecular(204, 204, 204);
    scene.context().directionalLight(102, 102, 102, 0, -0.1f, 0.4f);
    scene.context().specular(255, 255, 255);
    scene.context().shininess(10);
    //Render only usable chain
    heuristic.context().chain().get(0).cull = true;
    //Render target
    scene.render(heuristic.context().target());
    scene.render(heuristic.context().usableChain().get(0));
    //Render auxiliary structure
    for(int i = 0; i < fromIdx.length; i++) {
      if (fromIdx[i] != -1)
        scene.render(structureCopies.get(i).get(fromIdx[i]));
    }
    heuristic.context().chain().get(0).cull = false;
    scene.closeContext();

    setVizBounds();
    return scene.context().get();
  }

  public void resetBounds(){
    x_min = Integer.MAX_VALUE;
    x_max = -Integer.MAX_VALUE;
    y_min = Integer.MAX_VALUE;
    y_max = -Integer.MAX_VALUE;
  }

  public void setVizBounds(){
    float x_min = Float.MAX_VALUE, x_max = -Float.MAX_VALUE;
    float y_min = Float.MAX_VALUE, y_max = -Float.MAX_VALUE;
    for(Node node : heuristic.context().usableChain()){
      Vector v = scene.screenLocation(node);
      x_max = Math.max(v.x(), x_max);
      x_min = Math.min(v.x(), x_min);
      y_max = Math.max(v.y(), y_max);
      y_min = Math.min(v.y(), y_min);
    }
    for(List<Node> structure : structureCopies) {
      for (Node node : structure) {
        Vector v = scene.screenLocation(node);
        x_max = Math.max(v.x(), x_max);
        x_min = Math.min(v.x(), x_min);
        y_max = Math.max(v.y(), y_max);
        y_min = Math.min(v.y(), y_min);
      }
    }

    Vector v = scene.screenLocation(heuristic.context().target());
    x_max = Math.max(v.x(), x_max);
    x_min = Math.min(v.x(), x_min);
    y_max = Math.max(v.y(), y_max);
    y_min = Math.min(v.y(), y_min);

    //set viz bounds
    this.x_min = Math.min((int) (x_min - (x_max - x_min) * 0.3f), this.x_min);
    this.x_max = Math.max((int) (x_max + (x_max - x_min) * 0.3f), this.x_max);
    this.y_min = Math.min((int) (y_min - (y_max - y_min) * 0.1f), this.y_min);
    this.y_max = Math.max((int) (y_max + (y_max - y_min) * 0.1f), this.y_max);
  }

  public void addFrame(String name){
    showAnnotations = true;
    sequence.add(new Image(name, drawFrame(name)));
    showAnnotations = false;
    sequence.add(new Image("un" + name, drawFrame(name)));
  }

  public void saveFrames(){
    x_min = x_min - 20;
    y_min = y_min - 20;

    PGraphics pg = Scene.pApplet.createGraphics(x_max - x_min + 10, y_max - y_min + 20, PConstants.P2D);
    int idx = 0;
    pg.beginDraw();
    pg.clear();
    pg.endDraw();

    for(Image image : sequence){
      pg.beginDraw();
      pg.clear();
      pg.image(image.pimage.get(x_min, y_min,x_max - x_min,y_max - y_min),0,20);
      pg.stroke(0);
      pg.fill(0);
      pg.textAlign(PConstants.LEFT, PConstants.TOP);
      pg.text(image.title, 0,0);
      pg.endDraw();
      //crop image
      //image.pimage.get(x_min,y_min,x_max - x_min,y_max - y_min).save(path + "/" + name + idx++ + ".png");
      pg.save(path + "/" + idx++ / 2 + image.title + ".png");
      //p.save(path + "/" + name + idx++ + ".png");
    }
  }

  //Drawing methods
  void drawShape(PGraphics pg, Shape shape){
    pg.pushStyle();
    pg.stroke(0);
    pg.fill(shape.color);
    pg.beginShape();
    for(Vector v : shape.vertices){
      Vector s = scene.screenLocation(v);
      pg.vertex(s.x(), s.y());
    }
    pg.endShape(PConstants.CLOSE);
    pg.popStyle();
  }

  void drawArrow(PGraphics pg, Shape shape){
    pg.pushStyle();
    //Get position in terms of screen pix
    Vector v1 = scene.screenLocation(shape.vertices[0]);
    Vector v2 = scene.screenLocation(shape.vertices[1]);
    Vector dir = Vector.subtract(v2, v1).normalize(null);
    Vector v3 = Vector.subtract(v2, Vector.multiply(dir, 20));
    //Orthogonal dir
    Vector ortho = new Vector(-dir.y(), dir.x());
    ortho.multiply(2);
    //Vertices of quad
    Vector p1 = Vector.add(v1, ortho);
    Vector p2 = Vector.add(v3, ortho);
    Vector p3 = Vector.subtract(v3, ortho);
    Vector p4 = Vector.subtract(v1, ortho);
    //Vectices of Triangle
    ortho.multiply(4);
    Vector p5 = Vector.add(v3, ortho);
    Vector p6 = Vector.subtract(v3, ortho);
    Vector p7 = v2;

    pg.stroke(0);
    pg.fill(shape.color);
    pg.beginShape();
    pg.vertex(p1.x(), p1.y());
    pg.vertex(p2.x(), p2.y());
    pg.vertex(p3.x(), p3.y());
    pg.vertex(p4.x(), p4.y());
    pg.endShape(PConstants.CLOSE);

    pg.beginShape();
    pg.vertex(p5.x(), p5.y());
    pg.vertex(p6.x(), p6.y());
    pg.vertex(p7.x(), p7.y());
    pg.endShape(PConstants.CLOSE);

    pg.stroke(shape.color);
    pg.line(p2.x(), p2.y(), p3.x(), p3.y());

    if(!shape.name.equals("") && showAnnotations) {
      Vector l1 = Vector.add(v1, v2);
      l1.multiply(0.5f);
      //l1 = v2;
      Vector aux = Vector.cross(dir, new Vector(0,0,1), null);
      aux.multiply(20);
      Vector l2 = Vector.add(l1, aux);
      if(intersects(l2, 36)){
        aux.multiply(-1);
        l2 = Vector.add(l1, aux);
      }

      pg.stroke(0);
      pg.strokeWeight(3);
      pg.line(l1.x(), l1.y(), l2.x(), l2.y() + 20 * 0.25f);

      pg.stroke(255);
      pg.strokeWeight(1);
      pg.line(l1.x(), l1.y(), l2.x(), l2.y() + 20 * 0.25f);

      pg.stroke(0);
      pg.fill(255);
      pg.ellipse(l2.x(), l2.y() + 20 * 0.25f, 26, 26);
      pg.stroke(shape.textCol);
      pg.fill(shape.textCol);
      pg.textSize(18);
      pg.textAlign(PConstants.CENTER, PConstants.CENTER);
      pg.text(shape.name, l2.x(), l2.y());
      drawingNames.add(l2);
    }
    pg.popStyle();
  }

  void drawName(PGraphics pg, HighlightNode highlightNode) {
    if (!highlightNode.name.equals("") && showAnnotations) {
      Vector v1 = scene.screenLocation(highlightNode.node.reference());
      Vector v2 = scene.screenLocation(highlightNode.node);
      float dist = highlightNode.node._boneRadius * 6 * scene.sceneToPixelRatio(new Vector());
      Vector dir = Vector.subtract(v2, v1).normalize(null);
      Vector l1 = Vector.add(v1, v2);
      l1.multiply(0.5f);
      l1 = v2;
      Vector aux = Vector.cross(dir, new Vector(0, 0, 1), null);
      aux.multiply(dist);
      Vector l2 = Vector.add(l1, aux);
      if(intersects(l2, 36)){
        aux.multiply(-1);
        l2 = Vector.add(l1, aux);
      }

      pg.stroke(0);
      pg.strokeWeight(3);
      pg.line(l1.x(), l1.y(), l2.x(), l2.y() + 20 * 0.25f);

      pg.stroke(255);
      pg.strokeWeight(1);
      pg.line(l1.x(), l1.y(), l2.x(), l2.y() + 20 * 0.25f);

      pg.stroke(0);
      pg.fill(255);
      pg.ellipse(l2.x(), l2.y() + 20 * 0.25f, 26, 26);
      pg.stroke(0);
      pg.fill(0);
      pg.textSize(18);
      pg.textAlign(PConstants.CENTER, PConstants.CENTER);
      pg.text(highlightNode.name, l2.x(), l2.y());
      drawingNames.add(l2);
    }
  }


  void drawArc(PGraphics pg, Shape shape){
    Vector x = new Vector(1,0);
    Vector v0 = scene.screenLocation(shape.vertices[0]);
    Vector v1 = scene.screenLocation(shape.vertices[1]);
    Vector v2 = scene.screenLocation(shape.vertices[2]);
    Vector v01 = Vector.subtract(v1, v0);
    Vector v02 = Vector.subtract(v2, v0);

    float radius = Math.min(v01.magnitude(), v02.magnitude()) * 0.5f;
    //set starting point
    float start = Vector.angleBetween(v01, x);
    start = v01.y() > 0 ? start : -start;
    float stop =  Vector.angleBetween(v02, x);
    stop = v02.y() > 0 ? stop : -stop;
    if(Math.abs(Math.abs(start) - Math.abs(stop)) < Math.toRadians(3)) return;
    circArrow(pg, v0.x(),v0.y(), radius, start, stop, 11, Scene.pApplet.color(0), 3);
    circArrow(pg, v0.x(),v0.y(), radius, start, stop, 10, shape.color, 1);
  }

  // from https://forum.processing.org/one/topic/drawing-a-circular-arrow.html
  public void circArrow(PGraphics pg, float x, float y, float radius, float start, float stop, float arrowSize, int color, int weight){
    pg.pushStyle();
    pg.ellipseMode(PConstants.CENTER);
    pg.strokeWeight(weight);
    pg.stroke(color);
    pg.noFill();
    float flip = stop < start ? -1 : 1;
    if(flip < 0){
      pg.arc(x, y, radius * 2, radius * 2, stop, start);
    } else{
      pg.arc(x, y, radius * 2, radius * 2, start, stop);
    }
    float arrowX = x + Scene.pApplet.cos(stop) * radius;
    float arrowY = y + Scene.pApplet.sin(stop) * radius;

    float point1X = x + (Scene.pApplet.cos(stop) * radius) + flip * (Scene.pApplet.cos(stop - Scene.pApplet.radians(arrowSize * 5)) * (arrowSize));
    float point1Y = y + (Scene.pApplet.sin(stop) * radius) + flip * (Scene.pApplet.sin(stop - Scene.pApplet.radians(arrowSize * 5)) * (arrowSize));

    float point2X = x + (Scene.pApplet.cos(stop) * radius) + flip * (Scene.pApplet.cos(stop - Scene.pApplet.radians(-arrowSize * 5)) * (-arrowSize));
    float point2Y = y + (Scene.pApplet.sin(stop) * radius) + flip * (Scene.pApplet.sin(stop - Scene.pApplet.radians(-arrowSize * 5)) * (-arrowSize));

    pg.line(arrowX, arrowY, point1X, point1Y);
    pg.line(arrowX, arrowY, point2X, point2Y);
    pg.popStyle();
  }

  boolean intersects(Vector v, float r){
    for(Vector u : drawingNames){
      if(Vector.distance(u,v) < r) return  true;
    }
    return false;
  }
}
