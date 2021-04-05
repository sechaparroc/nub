package nub.ik.solver.fabrik;

import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.solver.*;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FABRIKTree extends Solver{
  protected static class TreeNode {
    protected TreeNode _parent;
    protected List<TreeNode> _children;
    protected List<TreeNode> _reachableLeafNodes; //Reachable leaf nodes from current node
    protected FABRIKChain _solver;
    protected float _weight = 1.f;
    protected boolean _outerTarget = false;

    public TreeNode() {
      _children = new ArrayList<TreeNode>();
      _reachableLeafNodes = new ArrayList<TreeNode>();
    }

    public TreeNode(FABRIKChain solver) {
      this._solver = solver;
      _solver.setTimesPerFrame(1);
      _children = new ArrayList<TreeNode>();
      _reachableLeafNodes = new ArrayList<TreeNode>();
    }

    protected TreeNode(TreeNode parent, FABRIKChain solver) {
      this._parent = parent;
      this._solver = solver;
      if (parent != null) {
        parent._addChild(this);
      }
      _children = new ArrayList<TreeNode>();
      _reachableLeafNodes = new ArrayList<TreeNode>();
    }

    protected boolean _addChild(TreeNode n) {
      return _children.add(n);
    }

    protected List<TreeNode> _children() {
      return _children;
    }

    protected float _weight() {
      return _weight;
    }

    protected FABRIKChain _solver() {
      return _solver;
    }
  }

  protected TreeNode _root;
  protected List<List<NodeState>> _initial, _bestSubchains;
  protected float _bestVal = 99999f, _bestDist = 999999f;
  protected boolean _innerChains = false;

  protected HashMap<Node, Node> _endEffectorMap = new HashMap<>();
  protected float _distanceFactor = 2;

  public FABRIKTree(Node root) {
    super();
    TreeNode dummy = new TreeNode(); //Dummy TreeNode to Keep Reference
    _setup(dummy, root, new ArrayList<Node>());
    //dummy must have only a child,
    this._root = dummy._children().get(0);
    this._root._parent = null;
    this.setMaxIterations(10);
    this.setTimesPerFrame(10);
    this.setChainTimesPerFrame(1);
    this.setChainMaxIterations(5);
    _reset();
  }


  protected void _setup(TreeNode parent, Node node, List<Node> list) {
    if (node == null) return;
    if (node.children().isEmpty()) { //Is a leaf node, hence we've found a chain of the structure
      list.add(node);
      FABRIKChain solver = new FABRIKChain(list);
      new TreeNode(parent, solver);
    } else if (node.children().size() > 1) {
      list.add(node);
      FABRIKChain solver = new FABRIKChain(list);
      TreeNode treeNode = new TreeNode(parent, solver);
      for (Node child : node.children()) {
        List<Node> newList = new ArrayList<>();
        _setup(treeNode, child, newList);
      }
    } else {
      list.add(node);
      _setup(parent, node.children().get(0), list);
    }
  }

  protected void _findReachableLeafNodes(TreeNode treeNode){
    treeNode._reachableLeafNodes.clear();
    if(treeNode._outerTarget == true){ // a leaf node has been reached
      return;
    }
    for(TreeNode child : treeNode._children){
      _findReachableLeafNodes(child);
      //Add the reachable leaf nodes of the child to the current node
      if(child._outerTarget == true) treeNode._reachableLeafNodes.add(child);
      treeNode._reachableLeafNodes.addAll(child._reachableLeafNodes);
    }
  }

  protected void _findLeafNodesTargets(TreeNode treeNode, List<Vector> effs, List<Vector> targets, Vector effs_centroid, Vector targets_centroid, boolean usableChain){
    effs.clear();
    targets.clear();
    effs_centroid.set(0,0,0);
    targets_centroid.set(0,0,0);
    Node node = usableChain ? treeNode._solver.context().usableChain().get(treeNode._solver.context().chain().size() -1) : treeNode._solver.context().chain().get(treeNode._solver.context().chain().size() -1);
    int i = 0;
    for(TreeNode leaf : treeNode._reachableLeafNodes){
      Vector eff = node.location(leaf._solver.context().chain().get(leaf._solver.context().chain().size() - 1));
      if(eff.magnitude() < 0.1) continue; //Too near from current node
      Vector target = node.location(leaf._solver.target());
      effs.add(eff);
      targets.add(target);
      effs_centroid.add(eff);
      targets_centroid.add(target);
      i++;
    }
    effs_centroid.divide(i);
    targets_centroid.divide(i);
  }

  protected float _applyBestRotation(TreeNode treeNode, Node target, List<Vector> effs, List<Vector> targets){
    FABRIKChain solver = treeNode._solver;
    Quaternion rotation = FA3R.FA3R(targets, effs, new Vector(), new Vector());
    if(solver.context().chain().get(solver.context().chain().size() - 1).constraint() != null)
      rotation = solver.context().chain().get(solver.context().chain().size() - 1).constraint().constrainRotation(rotation, solver.context().chain().get(solver.context().chain().size() - 1));
    //Avoid the rotation if the angle is relative small
    if(Math.abs(rotation.angle()) < Math.toRadians(2))
      rotation = new Quaternion();
    //constraint the rotation
    float prev_error = 0;
    float next_error = 0;
    for(int k = 0; k < effs.size(); k++){
      prev_error += Math.pow(Vector.subtract(effs.get(k), targets.get(k)).magnitude(), _distanceFactor);
      next_error += Math.pow(Vector.subtract(rotation.rotate(effs.get(k)), targets.get(k)).magnitude(), _distanceFactor);
    }
    prev_error /= effs.size();
    next_error /= effs.size();

    Quaternion q = solver.context().chain().get(solver.context().chain().size() - 1).orientation().get();
    target.setOrientation(q.get());

    if(next_error < prev_error) {
      target.rotate(rotation);
      solver.context().chain().get(solver.context().chain().size() - 1).rotate(rotation);
      return next_error;
    }
    return prev_error;
  }

  protected void moveParent(TreeNode node){
    if(node._parent == null) return;
    Node subbase = node._parent._solver.context().chain().get(node._parent._solver.context().chain().size() - 1);
    FABRIKChain solver = node._solver;
    if(solver.target() == null) return;
    //target w.r.t subbase
    Vector v1 = subbase.location(solver.context().chain().get(solver.context().chain().size() - 1).position());
    Vector v2 = subbase.location(solver.target().position());
    Quaternion q = new Quaternion(v1, v2);
    q = new Quaternion(q.axis(), q.angle() * 0.2f);
    //Rotate parent
    subbase.rotate(q);
  }

  protected boolean _solve(TreeNode treeNode) {
    if (treeNode._children == null || treeNode._children.isEmpty()) {
      FABRIKChain solver = treeNode._solver;
      if (solver.target() == null) return false;
      //solve ik for current chain
      solver._reset();
      for(int i = 0; i < solver.maxIterations(); i++) {
        solver.solve(); //Perform a given number of iterations
      }
      moveParent(treeNode);
      return true;
    }
    FABRIKChain solver = treeNode._solver;
    for (TreeNode child : treeNode._children()) {
      _solve(child);
    }
    solver._reset();
    List<Vector> effs = new ArrayList<Vector>();
    List<Vector> targets = new ArrayList<Vector>();
    Vector effs_centroid = new Vector();
    Vector targets_centroid = new Vector();
    Quaternion expected = new Quaternion();
    if(!treeNode._reachableLeafNodes.isEmpty()) {
      //Get the information of the leaf nodes
      _findLeafNodesTargets(treeNode, effs, targets, effs_centroid, targets_centroid, false);
      //Define the target position
      Vector targetTranslation = Vector.subtract(targets_centroid, effs_centroid);
      targetTranslation.multiply(_trust < 0.6f ? 0.8f : 1);
      Node target = solver.target() == null ? Node.detach(new Vector(), new Quaternion(), 1f) : solver.target();
      target.setPosition(solver.context().chain().get(solver.context().chain().size() - 1).worldLocation(targetTranslation).get());
      solver.setTarget(target);
      //Apply best rotation
      _applyBestRotation(treeNode, target, effs, targets);
      _findLeafNodesTargets(treeNode, effs, targets, effs_centroid, targets_centroid,false);
      targetTranslation = Vector.subtract(targets_centroid, effs_centroid);
      target.setPosition(solver.context().chain().get(solver.context().chain().size() - 1).worldLocation(targetTranslation));
      if (solver.context().chain().size() >= 2) {//If the solver has only a node we require to update manually
        if (!(solver.context().chain().size() == 2 && solver.context().chain().get(1).translation().magnitude() < 0.1)) {
          solver._reset();
          //Apply best rotation & keep best state
          _findLeafNodesTargets(treeNode, effs, targets, effs_centroid, targets_centroid, true);
          targetTranslation = Vector.subtract(targets_centroid, effs_centroid);
          target.setPosition(solver.context().usableChain().get(solver.context().chain().size() - 1).worldLocation(targetTranslation));
          solver.solve(); //Perform a given number of iterations
          //Fix eff rotation
          _findLeafNodesTargets(treeNode, effs, targets, effs_centroid, targets_centroid, false);
          _applyBestRotation(treeNode, target, effs, targets);
          //if(solver.direction()) _applyFIXO(treeNode, expected, target, 0.1f);
          moveParent(treeNode);
        }
      }
    }
    return true;
  }

  @Override
  protected boolean _iterate() {
    _solve(_root);
    //Keep best
    _saveBest();
    return false;
  }

  @Override
  protected void _update() {
    _updateToBest();
  }

  protected boolean _changed(TreeNode treeNode) {
    if (treeNode == null) return false;
    if (treeNode._solver()._changed() && treeNode._children().isEmpty()) return true;
    for (TreeNode child : treeNode._children()) {
      if (_changed(child)) return true;
    }
    return false;
  }

  @Override
  protected boolean _changed() {
    return _changed(_root);
  }

  protected void _reset(TreeNode treeNode) {
    if (treeNode == null) return;
    //Update Previous Target
    if (treeNode._solver()._changed()) treeNode._solver()._reset();
    for (TreeNode child : treeNode._children()) {
      _reset(child);
    }
  }

  @Override
  protected void _reset() {
    _iterations = 0;
    _bestVal = 10e10f;
    _bestDist = 10e10f;
    _saveInitial();
    _saveBest();
    _reset(_root);
  }

  @Override
  public float error() {
    return error(false);
  }

  public float error(boolean direction) {
    float e = 0;
    for (Map.Entry<Node, Node> entry : _endEffectorMap.entrySet()) {
      e += Vector.distance(entry.getKey().position(), entry.getValue().position());
      if(direction) e += Context.orientationError(entry.getKey().orientation(), entry.getValue().orientation(), false);
    }
    return e;
  }

  public float orientationError() {
    float e = 0;
    for (Map.Entry<Node, Node> entry : _endEffectorMap.entrySet()) {
      e += Math.toRadians(Context.orientationError(entry.getKey().orientation(), entry.getValue().orientation(), true));
    }
    return e;
  }



  @Override
  public void setTarget(Node endEffector, Node target) {
    addTarget(endEffector, target);
  }

  public Node head() {
    return (Node) _root._solver().context().chain().get(0);
  }


  protected boolean _addTarget(TreeNode treeNode, Node endEffector, Node target) {
    if (treeNode == null) return false;
    boolean nodeFound = false;
    Node prevEndEffector = null;
    for (Node node : treeNode._solver().context().chain()) {
      if (node == endEffector) {
        treeNode._solver().setTarget(endEffector, target);
        nodeFound = true;
      } else{
        if(_endEffectorMap.containsKey(node)) prevEndEffector = node;
      }
    }
    if(nodeFound){
      treeNode._outerTarget = true;
      _endEffectorMap.put(endEffector, target);
      if(prevEndEffector != null){
        _endEffectorMap.remove(prevEndEffector);
      }
      return true;
    }

    for (TreeNode child : treeNode._children()) {
      _addTarget(child, endEffector, target);
    }
    return false;
  }

  public boolean addTarget(Node endEffector, Node target) {
    boolean added = _addTarget(_root, endEffector, target);
    _findReachableLeafNodes(this._root); //update reachable leaf nodes
    return added;
  }


  public void setMaxError(float maxError) {
    super.setMaxError(maxError);
    _setMaxError(maxError, _root);
  }

  protected void _setMaxError(float maxError, TreeNode node) {
    node._solver.setMaxError(maxError);
    for (TreeNode child : node._children()) {
      _setMaxError(maxError, child);
    }
  }

  public void setChainTimesPerFrame(int timesPerFrame) {
    _setChainTimesPerFrame(timesPerFrame, _root);
  }

  protected void _setChainTimesPerFrame(float timesPerFrame, TreeNode node) {
    if (node == null) return;
    node._solver().setTimesPerFrame(1);
    node._solver().setMaxIterations((int) timesPerFrame);
    for (TreeNode child : node._children()) {
      _setChainTimesPerFrame(timesPerFrame, child);
    }
  }

  public void setChainMaxIterations(int maxIterations) {
    _setChainMaxIterations(maxIterations, _root);
  }

  protected void _setChainMaxIterations(int maxIterations, TreeNode node) {
    if (node == null) return;
    node._solver().setTimesPerFrame(1);
    node._solver().setMaxIterations(maxIterations);
    for (TreeNode child : node._children()) {
      _setChainMaxIterations(maxIterations, child);
    }
  }

  protected List<List<NodeState>> _obtainSubchains(){
    List<List<NodeState>> subchains = new ArrayList<>();
    List<TreeNode> frontier = new ArrayList<>();
    frontier.add(_root);
    while(!frontier.isEmpty()){
      TreeNode current = frontier.remove(0);
      if(current._children != null) frontier.addAll(current._children);
      subchains.add(saveState(current._solver.context().chain()));
    }
    return subchains;
  }

  protected void _saveInitial(){
    _initial = _obtainSubchains();
  }

  protected float distance(List<List<NodeState>> stateA, List<List<NodeState>> stateB){
    float dist = 0;
    for(int i = 0; i < stateA.size(); i++){
      List<NodeState> sa = stateA.get(i);
      List<NodeState> sb = stateB.get(i);
      float d = 0;
      for(int j = 0; j < sa.size(); j++){
        NodeState na = sa.get(j);
        NodeState nb = sb.get(j);
        d += Context.quaternionDistance(na._orientation, nb._orientation);
      }
      dist += d;
    }
    return dist / stateA.size();
  }


  protected void _updateToBest(){
    for(List<NodeState> state : _bestSubchains){
      restoreState(state);
    }
  }

  protected float _trust = 1f;
  protected void _saveBest(){
    float curError = error();
    if(Math.abs(curError - _bestVal) < _maxError*2){
      List<List<NodeState>> sc = _obtainSubchains();
      float dist = distance(sc, _initial);
      if(dist < _bestDist){
        _bestVal = curError;
        _bestSubchains = sc;
        _bestDist = dist;
        _trust = 1;
      } else{
        _trust *= 0.8f;
      }
    } else if(curError < _bestVal){
      List<List<NodeState>> sc = _obtainSubchains();
      float dist = distance(sc, _initial);
      _bestVal = curError;
      _bestSubchains = sc;
      _bestDist = dist;
      _trust = 1;
    } else{
      _trust *= 0.8f;
    }
    if(_trust < 0.6){
      _updateToBest();
    }
  }

  //Save Chain states
  protected static class NodeState{
    protected Node _node;
    protected Quaternion _rotation, _orientation;
    protected Vector _translation, _position;
    protected NodeState(Node node) {
         _node = node;
        _rotation = node.rotation().get();
        _translation = node.translation().get();
        _orientation = node.orientation().get();
        _position = node.position().get();
      }
  }

  public static List<NodeState> saveState(List<? extends Node> chain){
    List<NodeState> state = new ArrayList<NodeState>();
    for(Node  node : chain){
      state.add(new NodeState(node));
    }
    return state;
  }

  public static void restoreState(List<NodeState> state){
    for(NodeState nodeState : state){
      Node node = nodeState._node;
      Constraint constraint = node.constraint();
      node.setConstraint(null);
      node.setRotation(nodeState._rotation.get());
      node.setTranslation(nodeState._translation.get());
      node.setConstraint(constraint);
    }
  }



}
