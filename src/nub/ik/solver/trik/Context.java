package nub.ik.solver.trik;

import nub.core.Graph;
import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.solver.Solver;
import nub.ik.animation.Joint;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

import java.util.ArrayList;
import java.util.List;

public class Context {
  /**
   * A TRIK solver is a set of Heuristics that works on the same Context in order to solve the IK
   * problem on a kinematic chain.
   *
   * A Context contains the sufficient information that a heuristic must use in order to find
   * a new chain configuration.
   * */

  /**
   * Chain is the kinematic chain to modify. Since we expect to return at each iteration a better
   * solution that the previously found, this solver must be steady state. Hence, we perform any action
   * on a usableChain (a copy of the original chain) and only when the new configuration results in a
   * better solution we modify the kinematic chain.
   */
  protected List<? extends Node> _chain; //the kinematic chain to modify
  protected List<Node> _usableChain; //a copy of the kinematic chain.


  //Expressive Parameters tries to reduce the movement done by each joint, such that the distance from initial one is reduced
  protected boolean _enableDelegation = false;
  protected float _clamping;
  protected float[] _maxAngleAtJoint;
  protected float _delegationIterationsRatio = 1; //How many of the max number of iterations must consider the expressive parameters
  protected Quaternion[] _initialRotations;


  protected int _deadlockCounter = 0, _lockTimesCriteria = 5;

  //This structures allows to find the world position /orientation of a Node using the sufficient operations
  protected List<NodeInformation> _chainInformation, _usableChainInformation; //Keep position / orientation information
  protected Node _target, _worldTarget, _previousTarget; //Target to reach

  protected boolean _direction = false;
  //Important parameters for orientation solution
  protected float _searchingAreaRadius = 1;

  protected boolean _radiusRelativeToBoneAverage = true;
  protected float _orientationWeight = 0.5f;

  protected boolean _topToBottom = false;

  //Error attributes
  protected float _maxLength = 0, _avgLength = 0;
  protected Solver _solver;

  protected boolean _is2D = false;
  protected boolean _debug = false;
  protected int _last = -1;
  protected int _endEffectorId;

  protected boolean _singleStep = false;


  public float searchingAreaRadius() {
    if (_radiusRelativeToBoneAverage) return _searchingAreaRadius * _avgLength;
    return _searchingAreaRadius;
  }

  public void setSearchingAreaRadius(float searchingAreaRadius, boolean relativeToBoneAverage) {
    _radiusRelativeToBoneAverage = relativeToBoneAverage;
    _searchingAreaRadius = searchingAreaRadius;
  }

  public void setSearchingAreaRadius(float searchingAreaRadius) {
    setSearchingAreaRadius(searchingAreaRadius, false);
  }

  public void setRadiusRelativeToBoneAverage(boolean relativeToBoneAverage) {
    _radiusRelativeToBoneAverage = relativeToBoneAverage;
  }

  public void setOrientationWeight(float orientationWeight) {
    _orientationWeight = orientationWeight;
  }

  public float orientationWeight() {
    return _orientationWeight;
  }

  public void setDirection(boolean direction) {
    _direction = direction;
  }


  public int deadlockCounter() {
    return _deadlockCounter;
  }

  public int lockTimesCriteria(){
    return _lockTimesCriteria;
  }

  public void setLockTimesCriteria(int times){
    _lockTimesCriteria = times;
  }

  public void incrementDeadlockCounter() {
    _deadlockCounter++;
  }

  public void resetDeadlockCounter() {
    _deadlockCounter = 0;
  }

  public Context(List<? extends Node> chain, Node target) {
    this(chain, target, false);
  }

  public Context(List<? extends Node> chain, Node target, boolean debug) {
    this._chain = chain;
    this._debug = debug;
    if (_debug && Graph.isReachable(_chain.get(0))) {
      this._usableChain = _attachedCopy(chain, null);
    } else {
      this._usableChain = _detachedCopy(chain);
    }

    //create info list
    _chainInformation = NodeInformation._createInformationList(_chain, true);
    _usableChainInformation = NodeInformation._createInformationList(_usableChain, true);
    this._last = _chain.size() - 1;

    this._target = target;
    this._previousTarget =
        target == null ? null : Node.detach(target.position().get(), target.orientation().get(), 1);

    this._worldTarget = target == null ? Node.detach(new Vector(), new Quaternion(), 1) : Node.detach(_target.position(), _target.orientation(), 1);
    this._last = _chain.size() - 1;
    _endEffectorId = _last;
    _maxAngleAtJoint = new float[chain.size()];
    for(int i = 0; i < chain.size(); i++){
      _maxAngleAtJoint[i] = Float.MAX_VALUE; //By default no clamping is performed
    }
    _initialRotations = new Quaternion[chain().size()];
    update();
  }

  public void setSolver(Solver solver) {
    _solver = solver;
  }

  public Solver solver() {
    return _solver;
  }

  //Getters and setters
  public List<? extends Node> chain() {
    return _chain;
  }

  public List<Node> usableChain() {
    return _usableChain;
  }

  public List<NodeInformation> chainInformation() {
    return _chainInformation;
  }

  public List<NodeInformation> usableChainInformation() {
    return _usableChainInformation;
  }

  public int last() {
    return _last;
  }

  public int endEffectorId(){
    return _endEffectorId;
  }


  public boolean direction() {
    return _direction;
  }

  public Node target() {
    return _target;
  }

  public Node worldTarget() {
    return _worldTarget;
  }


  public boolean debug() {
    return _debug;
  }

  public void setDebug(boolean debug) {
    _debug = debug;
  }

  public boolean singleStep() { //TODO : REMOVE!
    return _singleStep;
  }

  public void setSingleStep(boolean singleStep) {
    _singleStep = singleStep;
  }

  public NodeInformation endEffectorInformation() {
    return _usableChainInformation.get(_endEffectorId);
  }

  public Node previousTarget() {
    return _previousTarget;
  }

  public void setPreviousTarget(Node previousTarget) {
    _previousTarget = previousTarget;
  }

  public float avgLength() {
    return _avgLength;
  }

  public float maxLength() {
    return _maxLength;
  }

  public void setMaxLength(float maxLength) {
    _maxLength = maxLength;
  }

  public void setAvgLength(float avgLength) {
    _avgLength = avgLength;
  }

  public void setTarget(Node endEffector, Node target){
    _endEffectorId = chain().indexOf(endEffector);
    _target = target;
  }

  public void setEndEffector(int id){
    _endEffectorId = id;
  }

  public void setEndEffector(Node endEffector){
    _endEffectorId = chain().indexOf(endEffector);
  }

  public void setTarget(Node target) {
    _target = target;
  }

  public boolean is2D(){
    return _is2D;
  }

  public void set2D(boolean is2D){
    _is2D = is2D;
  }

  public void copyChainState(List<NodeInformation> origin, List<NodeInformation> dest) {
    //Copy the content of the origin chain into dest
    Node refDest = dest.get(0).node().reference();
    if (refDest != null) {
      Constraint constraint = refDest.constraint();
      refDest.setConstraint(null);
      refDest.set(origin.get(0).node().reference());
      refDest.setConstraint(constraint);
    }

    for (int i = 0; i < origin.size(); i++) {
      Node node = origin.get(i).node();
      Constraint constraint = dest.get(i).node().constraint();
      Quaternion rotation = node.rotation().get();
      Vector translation = node.translation().get();

      dest.get(i).node().setConstraint(null);
      dest.get(i).node().setRotation(rotation);
      dest.get(i).node().setTranslation(translation);
      dest.get(i).node().setScaling(node.scaling());
      dest.get(i).node().setConstraint(constraint);
    }
  }

  public int currentIteration() {
    return !_singleStep ? _solver.iteration() :  _solver.iteration() / chain().size();
  }

  public boolean applyDelegation(){
    return _enableDelegation && currentIteration() < _delegationIterationsRatio * solver().maxIterations();
  }

  public float clamping(int i){
    float k = (1f - _clamping) * (1f * currentIteration() + 1f) / delegationIterations();
    //Clamping based on distance
    return Math.min(_clamping + k, 1);
  }

  public int delegationIterations(){
    return (int) (_delegationIterationsRatio * solver().maxIterations());
  }

  public void setClamping(float clamping) {
    _clamping = clamping;
  }

  public float maxAngleAtJoint(int i) {
    return _maxAngleAtJoint[i];
  }

  public void setMaxAngleAtJoint(int i, float value) {
    _maxAngleAtJoint[i] = value;
  }

  public void setMaxAngle(float max){
    for (int i = 0; i < _maxAngleAtJoint.length; i++){
      _maxAngleAtJoint[i] = max;
    }
  }



  public void enableDelegation(boolean enableDelegation) {
    _enableDelegation = enableDelegation;
  }

  public boolean enableDelegation() {
    return _enableDelegation;
  }

  public void setTopToBottom(boolean topToBottom) {
    if (_topToBottom != topToBottom) {
      _topToBottom = topToBottom;
    }
  }


  public boolean topToBottom() {
    return _topToBottom;
  }


  /*Error measures*/
  public static float positionError(Vector eff, Vector target) {
    return Vector.distance(eff, target);
  }

  public static float positionError(NodeInformation eff, Node target) {
    return positionError(eff.positionCache(), target.position());
  }

  public static float orientationError(Quaternion eff, Quaternion target, boolean angles) {
    float s1 = 1, s2 = 1;
    if (eff.w() < 0) s1 = -1;
    if (target.w() < 0) s2 = -1;
    float dot = s1 * eff._quaternion[0] * s2 * target._quaternion[0] +
        s1 * eff._quaternion[1] * s2 * target._quaternion[1] +
        s1 * eff._quaternion[2] * s2 * target._quaternion[2] +
        s1 * eff._quaternion[3] * s2 * target._quaternion[3];

    //clamp dot product
    dot = Math.min(Math.max(dot, -1), 1);
    if (angles) return (float) Math.toDegrees(Math.acos(Math.min(Math.max(2 * dot * dot - 1, -1), 1)));
    return (float) (1 - dot * dot);
  }


  public static float quaternionDistance(Quaternion a, Quaternion b) {
    float dot = Quaternion.dot(a,b);
    return (float) (1 - Math.pow(dot, 2));
  }

  public float error(NodeInformation eff, Node target, float w1, float w2) {
    return error(eff.positionCache(), target.position(), eff.orientationCache(), target.orientation(), w1, w2);
  }

  public float error(NodeInformation eff, Node target) {
    return error(eff.positionCache(), target.position(), eff.orientationCache(), target.orientation(), 1, 1);
  }

  public float error(Vector effPosition, Vector targetPosition, Quaternion effRotation, Quaternion targetRotation, float w1, float w2) {
    float error = positionError(effPosition, targetPosition);
    float radius = _radiusRelativeToBoneAverage ? _searchingAreaRadius * _avgLength : _searchingAreaRadius;
    if (_direction) {
      float orientationError = orientationError(effRotation, targetRotation, false);
      float weighted_error = error / radius;
      error = weighted_error * (1 - _orientationWeight) + _orientationWeight * orientationError;
    }
    return error;
  }

  public void update() {
    //find maxLength
    float maxLength = 0;
    for (int i = 0; i < chain().size() - 1; i++) {
      maxLength += Vector.distance(chainInformation().get(i).positionCache(), chainInformation().get(i + 1).positionCache());
    }
    _maxLength = maxLength;
    _avgLength = maxLength / chain().size();

    //Set values of worldTarget and worldEndEffector
    if (_target != null) {
      worldTarget().setRotation(target().orientation().get());
      worldTarget().setPosition(target().position().get());
    }
  }

  public static List<Node> _detachedCopy(List<? extends Node> chain, Node reference) {
    return _detachedCopy(chain, reference, true);
  }

  public static List<Node> _detachedCopy(List<? extends Node> chain, Node reference, boolean copy_constraints) {
    List<Node> copy = new ArrayList<Node>();
    for (Node joint : chain) {
      Node newJoint = Node.detach(new Vector(), new Quaternion(), 1);
      if(reference != null) newJoint.setReference(reference);
      newJoint.setPosition(joint.position().get());
      newJoint.setOrientation(joint.orientation().get());
      if (copy_constraints) newJoint.setConstraint(joint.constraint());
      copy.add(newJoint);
      reference = newJoint;
    }
    return copy;
  }

  public static List<Node> _detachedCopy(List<? extends Node> chain) {
    return _detachedCopy(chain, true);
  }

  public static List<Node> _detachedCopy(List<? extends Node> chain, boolean copy_constraints) {
    Node reference = chain.get(0).reference();
    if (reference != null) {
      reference = Node.detach(reference.position().get(), reference.orientation().get(), 1);
    }
    return _detachedCopy(chain, reference, copy_constraints);
  }

  public static List<Node> _attachedCopy(List<? extends Node> chain, Node reference) {
    return _attachedCopy(chain, reference, true);
  }

  public static List<Node> _attachedCopy(List<? extends Node> chain, Node reference, boolean copy_constraints) {
    Node ref = reference;
    List<Node> copy = new ArrayList<Node>();
    if (ref == null) {
      reference = chain.get(0).reference();
      if (reference != null) {
        ref = new Node();
        ref.setPosition(reference.position().get());
        ref.setOrientation(reference.orientation().get());
        ref.tagging = false;
      }
    }

    int r = (int) (Math.random() * 255);
    int g = (int) (Math.random() * 255);
    int b = (int) (Math.random() * 255);
    for (Node joint : chain) {
      Joint newJoint = new Joint(r, g, b, 3);
      if (copy.isEmpty()) {
        newJoint.setRoot(true);
      }
      newJoint.setReference(ref);
      newJoint.setPosition(joint.position().get());
      newJoint.setOrientation(joint.orientation().get());
      if (copy_constraints) newJoint.setConstraint(joint.constraint());
      copy.add(newJoint);
      ref = newJoint;
    }
    return copy;
  }

  public static List<NodeState> saveState(List<? extends NodeInformation> chain){
    List<NodeState> state = new ArrayList<NodeState>();
    for(NodeInformation  nodeInformation : chain){
      state.add(new NodeState(nodeInformation));
    }
    return state;
  }

  public static void restoreState(List<NodeState> state){
    for(NodeState nodeState : state){
      Node node = nodeState._nodeInformation.node();
      Constraint constraint = node.constraint();
      node.setConstraint(null);
      node.setRotation(nodeState._rotation.get());
      node.setTranslation(nodeState._translation.get());
      node.setConstraint(constraint);
      nodeState._nodeInformation.setCache(nodeState._position.get(), nodeState._orientation.get());
    }
  }
}
