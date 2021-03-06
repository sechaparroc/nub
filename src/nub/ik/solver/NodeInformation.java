package nub.ik.solver;

import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

import java.util.ArrayList;
import java.util.List;

public class NodeInformation {

  //THIS CLASS IS USED IN ORDER TO AVOID REDUNDANT WORK
  //TODO: Move this class and refine
  public static boolean disableCache = false; //DISABLE CACHE ONLY FOR HIGH PRECISION BENCHMARK
  protected NodeInformation _reference;
  protected Node _node;
  protected Quaternion _orientationCache;
  protected Vector _positionCache;

  public NodeInformation(NodeInformation ref, Node node) {
    this._reference = ref;
    this._node = node;
  }

  public void setPositionCache(Vector position) {
    _positionCache = position.get();
  }

  public void setOrientationCache(Quaternion orientation) {
    _orientationCache = orientation.get();
  }

  public void setCache(Vector position, Quaternion orientation) {
    setPositionCache(position);
    setOrientationCache(orientation);
  }

  public Node node() {
    return _node;
  }

  public NodeInformation reference() {
    return _reference;
  }

  public Quaternion orientationCache() {
    if (disableCache) return node().orientation();
    return _orientationCache;
  }

  public Vector positionCache() {
    if (disableCache) return node().position();
    return _positionCache;
  }

  public void setOrientationWithCache(Quaternion orientation) {
    if (_node.constraint() == null) {
      Quaternion delta = Quaternion.compose(_orientationCache.inverse(), orientation);
      _orientationCache = orientation;
      _node.rotate(delta);
    } else {
      Quaternion delta = _node.constraint().constrainRotation(Quaternion.compose(_orientationCache.inverse(), orientation), _node);
      _orientationCache.compose(delta);
      Constraint constraint = _node.constraint();
      _node.setConstraint(null);
      _node.rotate(delta);
      _node.setConstraint(constraint);
    }
  }

  public void setPositionWithCache(Vector position) {
    Vector translation = Vector.subtract(position, _positionCache);
    //diff w.r.t ref
    if (_reference != null) {
      translation = _reference._orientationCache.inverseRotate(translation);
    }
    if (_node.constraint() == null) {
      _positionCache = position;
      _node.translate(translation);
    } else {
      translation = _node.constraint().constrainTranslation(translation, _node);
      _positionCache.add(translation);
      Constraint constraint = _node.constraint();
      _node.setConstraint(null);
      _node.translate(translation);
      _node.setConstraint(constraint);
    }
  }

  public Vector locationWithCache(Vector worldVector) {
    if (disableCache) return node().location(worldVector);
    Vector translation = Vector.subtract(worldVector, positionCache());
    return orientationCache().inverseRotate(translation);
  }

  public Vector locationWithCache(NodeInformation node) {
    return locationWithCache(node.positionCache());
  }

  /*
   * NOTE : This update the cache taking into account only the current action, if parents were modified, the cache
   * must be updated explicitly.
   * */

  //Translate the node by delta and update the orientation/position of the remaining nodes
  public void translateAndUpdateCache(Vector delta, boolean useConstraint, NodeInformation... nodeInfoList) {
    Constraint constraint = _node.constraint();
    if (useConstraint && constraint != null) delta = constraint.constrainTranslation(delta, _node);
    Quaternion ref = Quaternion.compose(orientationCache(), _node.rotation().inverse());
    ref.normalize();
    Vector t = ref.rotate(delta); //w.r.t world
    _node.setConstraint(null);
    _node.translate(delta);
    _node.setConstraint(constraint);
    _positionCache.add(t);
    if (nodeInfoList.length > 0) {
      for (NodeInformation other : nodeInfoList) {
        other.positionCache().add(t);
      }
    }
  }


  //Rotate the node by delta and update the orientation/position of the remaining nodes
  public void rotateAndUpdateCache(Quaternion delta, boolean useConstraint, NodeInformation... nodeInfoList) {
    Constraint constraint = _node.constraint();
    if (useConstraint && constraint != null){
      delta = constraint.constrainRotation(delta, _node);
      delta.normalize();
    }

    _node.setConstraint(null);
    Quaternion orientation = Quaternion.compose(orientationCache(), delta);
    orientation.normalize(); // Prevents numerical drift
    if (nodeInfoList.length > 0) {
      Quaternion q = Quaternion.compose(orientation, orientationCache().inverse());
      q.normalize();
      for (NodeInformation other : nodeInfoList) {
        Quaternion o = Quaternion.compose(q, other.orientationCache());
        o.normalize();
        other.setOrientationCache(o);
        Vector p = Vector.subtract(other.positionCache(), positionCache());
        other.setPositionCache(Vector.add(positionCache(), q.rotate(p)));
      }
    }
    _node.rotate(delta);
    _node.setConstraint(constraint);
    _orientationCache = orientation;
  }

  //Updates cache assuming that reference contains an updated cache
  public void updateCacheUsingReference() {
    _orientationCache = Quaternion.compose(reference().orientationCache(), node().rotation());
    _orientationCache.normalize();
    _positionCache = Vector.add(reference().positionCache(), reference().orientationCache().rotate(node().translation()));
  }

  //Updates cache using child
  public void updateCacheUsingChild(NodeInformation child) {
    _orientationCache = Quaternion.compose(child.orientationCache(), child.node().rotation().inverse());
    _orientationCache.normalize();
    _positionCache = Vector.subtract(child.positionCache(), orientationCache().rotate(child.node().translation()));
  }


  public static List<NodeInformation> _createInformationList(List<? extends Node> nodeList, boolean updateCache) {
    List<NodeInformation> infoList = new ArrayList<NodeInformation>();
    NodeInformation ref = null;
    Quaternion orientation = nodeList.get(0).reference() != null && updateCache ? nodeList.get(0).reference().orientation() : new Quaternion();
    Vector position = nodeList.get(0).reference() != null && updateCache ? nodeList.get(0).reference().position() : new Vector();
    for (Node node : nodeList) {
      NodeInformation nodeInfo = new NodeInformation(ref, node);
      infoList.add(nodeInfo);
      ref = nodeInfo;
      //update cache
      if (updateCache) {
        position.add(orientation.rotate(node.translation()));
        orientation.compose(node.rotation());
        orientation.normalize();
        nodeInfo.setCache(position.get(), orientation.get());
      }
    }
    return infoList;
  }

  public static void _updateCache(List<NodeInformation> nodeInfoList) {
    Quaternion orientation = nodeInfoList.get(0).node().reference() != null ? nodeInfoList.get(0).node().reference().orientation().get() : new Quaternion();
    orientation.normalize();
    Vector position = nodeInfoList.get(0).node().reference() != null ? nodeInfoList.get(0).node().reference().position() : new Vector();
    for (NodeInformation nodeInfo : nodeInfoList) {
      Node node = nodeInfo.node();
      position.add(orientation.rotate(node.translation()));
      orientation.compose(node.rotation());
      orientation.normalize();
      nodeInfo.setCache(position.get(), orientation.get());
    }
  }

  public static void _copyCache(List<NodeInformation> origin, List<NodeInformation> dest) {
    for (int i = 0; i < origin.size(); i++) {
      dest.get(i).setPositionCache(origin.get(i).positionCache().get());
      dest.get(i).setOrientationCache(origin.get(i).orientationCache().get());
    }
  }
}
