package nub.ik.solver.trik;

import nub.core.Node;
import nub.ik.solver.Solver;
import nub.ik.solver.trik.implementations.IKSolver;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Tree extends Solver {
  protected static class TreeNode {
    protected TreeNode _parent;
    protected List<TreeNode> _children;
    protected List<TreeNode> _reachableLeafNodes; //Reachable leaf nodes from current node
    protected IKSolver _solver;
    protected float _weight = 1.f;

    protected boolean _outerTarget = false;

    public TreeNode() {
      _children = new ArrayList<TreeNode>();
      _reachableLeafNodes = new ArrayList<TreeNode>();
    }

    public TreeNode(IKSolver solver) {
      this._solver = solver;
      _solver.setTimesPerFrame(1);
      _children = new ArrayList<TreeNode>();
      _reachableLeafNodes = new ArrayList<TreeNode>();
    }

    protected TreeNode(TreeNode parent, IKSolver solver) {
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

    protected IKSolver _solver() {
      return _solver;
    }
  }

  protected TreeNode _root;
  protected IKSolver.HeuristicMode _mode;
  protected float _current = 10e10f, _best = 10e10f;
  protected HashMap<Node, Node> _endEffectorMap = new HashMap<>();
  protected float _distanceFactor = 5;


  public Tree(Node root) {
    this(root, IKSolver.HeuristicMode.COMBINED_TRIK);
  }

  public Tree(Node root, IKSolver.HeuristicMode mode) {
    super();
    TreeNode dummy = new TreeNode(); //Dummy TreeNode to Keep Reference
    _mode = mode;
    _setup(dummy, root, new ArrayList<Node>());
    //dummy must have only a child,
    this._root = dummy._children().get(0);
    this._root._parent = null;
  }


  protected void _setup(TreeNode parent, Node node, List<Node> list) {
    if (node == null) return;
    if (node.children().isEmpty()) { //Is a leaf node, hence we've found a chain of the structure
      list.add(node);
      IKSolver solver = new IKSolver(list, _mode);
      new TreeNode(parent, solver);
    } else if (node.children().size() > 1) {
      list.add(node);
      IKSolver solver = new IKSolver(list, _mode);
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

  protected void _findLeafNodesTargets(TreeNode treeNode, List<Vector> effs, List<Vector> targets, Vector effs_centroid, Vector targets_centroid){
    effs.clear();
    targets.clear();
    Node node = treeNode._solver.context().chain().get(treeNode._solver.context().endEffectorId());
    for(TreeNode leaf : treeNode._reachableLeafNodes){
      Vector eff = node.location(leaf._solver.context().chain().get(leaf._solver.context().endEffectorId()));
      Vector target = node.location(leaf._solver.target());
      if(eff.magnitude() < 0.1) continue; //Too near from current node
      effs.add(eff);
      targets.add(target);
      effs_centroid.add(eff);
      targets_centroid.add(target);
    }
    effs_centroid.divide(effs.size());
    targets_centroid.divide(effs.size());
  }

  protected float _applyBestRotation(TreeNode treeNode, List<Vector> effs, List<Vector> targets){
      IKSolver solver = treeNode._solver;
      Quaternion rotation = FA3R.FA3R(targets, effs, new Vector(), new Vector());
      if(solver.context().chain().get(solver.context().endEffectorId()).constraint() != null)
          rotation = solver.context().chain().get(solver.context().endEffectorId()).constraint().constrainRotation(rotation, solver.context().chain().get(solver.context().endEffectorId()));
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

      if(next_error < prev_error) {
          Quaternion expected = Quaternion.compose(solver.context().chain().get(solver.context().endEffectorId()).rotation(), rotation);
          expected.normalize();
          solver.context().chain().get(solver.context().endEffectorId()).rotate(rotation);
          return next_error;
      }
      return prev_error;
  }

  protected boolean _solve(TreeNode treeNode) {
    if (treeNode._children == null || treeNode._children.isEmpty()) {
      IKSolver solver = treeNode._solver;
      if (solver.target() == null) return false;
      //solve ik for current chain
      solver.reset();
      for(int i = 0; i < solver.maxIterations(); i++) {
        solver.solve(); //Perform a given number of iterations
      }
      return true;
    }

    IKSolver solver = treeNode._solver;
    for (TreeNode child : treeNode._children()) {
      _solve(child);
    }

    List<Vector> effs = new ArrayList<Vector>();
    List<Vector> targets = new ArrayList<Vector>();
    Vector effs_centroid = new Vector();
    Vector targets_centroid = new Vector();

    if(!treeNode._reachableLeafNodes.isEmpty()) {
      //Get the information of the leaf nodes
      _findLeafNodesTargets(treeNode, effs, targets, effs_centroid, targets_centroid);

      //Define the target position
      Vector targetTranslation = Vector.subtract(targets_centroid, effs_centroid);
      Node target = solver.target() == null ? Node.detach(new Vector(), new Quaternion(), 1f) : solver.target();
      target.setPosition(solver.context().chain().get(solver.context().endEffectorId()).worldLocation(targetTranslation));
      solver.setTarget(target);

      solver.context().setDirection(false);
      //Apply best rotation
      float minError = _applyBestRotation(treeNode, effs, targets);
      //Apply IK
      if (solver.context().chain().size() >= 2) {//If the solver has only a node we require to update manually
        if (!(solver.context().chain().size() == 2 && solver.context().chain().get(1).translation().magnitude() < 0.1)) {
          solver.reset();
          //Apply best rotation & keep best state
          List<NodeState> bestState = Context.saveState(solver.context().chainInformation());
          for (int i = 0; i < solver.maxIterations(); i++) {
            solver.solve(); //Perform a given number of iterations
            //Fix eff rotation
            _findLeafNodesTargets(treeNode, effs, targets, effs_centroid, targets_centroid);
            float err = _applyBestRotation(treeNode, effs, targets);
            //Keep best configuration
            if (err < minError) {
              bestState = Context.saveState(solver.context().chainInformation());
              minError = err;
            } else {
              Context.restoreState(bestState);
              //reduce the distance to the target
              targetTranslation.multiply(0.5f);
              solver.reset();
              solver.target().setPosition(solver.context().chain().get(solver.context().endEffectorId()).worldLocation(targetTranslation));
            }
          }
          //update to best state
          Context.restoreState(bestState);
        }
      }
    }

    return true;
  }

  @Override
  protected boolean _iterate() {
    _solve(_root);
    return false;
  }

  @Override
  protected void _update() {

  }

  protected boolean _changed(TreeNode treeNode) {
    if (treeNode == null) return false;
    if (treeNode._solver().changed() && treeNode._children().isEmpty()) return true;
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
    if (treeNode._solver().changed()) treeNode._solver().reset();
    for (TreeNode child : treeNode._children()) {
      _reset(child);
    }
  }

  public void set2D(boolean is2D) {
    _set2D(is2D, _root);
  }

  protected void _set2D(boolean is2D, TreeNode treeNode) {
    if (treeNode == null) return;
    if (treeNode._children().isEmpty()) treeNode._solver.context().set2D(is2D);
    for (TreeNode child : treeNode._children()) {
      _setDirection(is2D, child);
    }
  }


  public void setDirection(boolean direction) {
    _setDirection(direction, _root);
  }

  protected void _setDirection(boolean direction, TreeNode treeNode) {
    if (treeNode == null) return;
    if (treeNode._children().isEmpty()) treeNode._solver.context().setDirection(direction);
    for (TreeNode child : treeNode._children()) {
      _setDirection(direction, child);
    }
  }

  @Override
  protected void _reset() {
    _iterations = 0;
    _best = 0;
    _current = 10e10f;
    _reset(_root);
  }

  @Override
  public float error() {
    float e = 0;
    for (Map.Entry<Node, Node> entry : _endEffectorMap.entrySet()) {
      e += Vector.distance(entry.getKey().position(), entry.getValue().position());
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
    node._solver().setTimesPerFrame(timesPerFrame);
    for (TreeNode child : node._children()) {
      _setChainTimesPerFrame(timesPerFrame, child);
    }
  }

  public void setChainMaxIterations(int maxIterations) {
    _setChainMaxIterations(maxIterations, _root);
  }

  protected void _setChainMaxIterations(int maxIterations, TreeNode node) {
    if (node == null) return;
    node._solver().setMaxIterations(maxIterations);
    for (TreeNode child : node._children()) {
      _setChainMaxIterations(maxIterations, child);
    }
  }

  public void setSearchingAreaRadius(float radius, boolean relativeToBoneAvg) {
    _setSearchingAreaRadius(radius, relativeToBoneAvg, _root);
  }

  protected void _setSearchingAreaRadius(float radius, boolean relativeToBoneAvg, TreeNode node) {
    if (node == null) return;
    node._solver.context().setSearchingAreaRadius(radius, relativeToBoneAvg);
    for (TreeNode child : node._children()) {
      _setSearchingAreaRadius(radius, relativeToBoneAvg, child);
    }

  }


  public void setOrientationWeight(float orientationWeight) {
    setOrientationWeight(orientationWeight, _root);
  }

  protected void setOrientationWeight(float orientationWeight, TreeNode node) {
    if (node == null) return;
    node._solver.context().setOrientationWeight(orientationWeight);
    for (TreeNode child : node._children()) {
      setOrientationWeight(orientationWeight, child);
    }
  }
}
