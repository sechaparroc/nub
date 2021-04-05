package nub.ik.solver.fabrik;

import nub.core.Node;
import nub.core.constraint.Constraint;
import nub.ik.solver.Context;
import nub.ik.solver.NodeInformation;
import nub.ik.solver.Solver;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

import java.util.ArrayList;
import java.util.List;

public class FABRIKChain extends Solver {
  protected Context _context;
  protected  ArrayList<Vector> _positions = new ArrayList<>();
  protected ArrayList<Float> _distances = new ArrayList<>();

  //Steady state algorithm
  protected float _current = 10e10f, _best = 10e10f, _previousBest = 10e10f;
  protected int _stepCounter;
  protected int _totalDeadlock = 0;
  protected boolean _enableDeadLockResolution = false;
  protected boolean _workInOrientationSpace = true; //Required for constraints

  public void enableDeadLockResolution(boolean enable) {
    _enableDeadLockResolution = enable;
  }

  public int totalDeadlock(){
    return _totalDeadlock;
  }

  public FABRIKChain(List<? extends Node> chain, Node target) {
    super();
    this._context = new Context(chain, target, false);
    _context.setSolver(this);
    _context.setSingleStep(false);
  }

  public FABRIKChain(List<? extends Node> chain) {
    this(chain, null);
  }

  public ArrayList<Vector> _positions() {
    return _positions;
  }

  public Context context() {
    return _context;
  }

  public void workInOrientationSpace(boolean enable){
    _workInOrientationSpace = enable;
  }

  public boolean workInOrientationSpace(){
    return _workInOrientationSpace;
  }

  protected Vector _move(Vector u, Vector v, float distance){
    float r = Vector.distance(u, v);
    float lambda_i = distance / r;
    Vector new_u = Vector.multiply(u, 1.f - lambda_i);
    new_u.add(Vector.multiply(v, lambda_i));
    return new_u;
  }

  protected float _forwardReaching(){
    float change = 0;
    List<NodeInformation> chain = _context.usableChainInformation();
    int n = _context.usableChainInformation().size();
    for(int i = n - 2; i >= 0; i--){
      Vector pos_i = _positions.get(i);
      Vector pos_i1 = _positions.get(i + 1);
      float dist_i = _distances.get(i + 1);
      if(dist_i <= 10e-4){
        _positions.set(i, pos_i1.get());
        continue;
      }
      if(chain.get(i+1).node().constraint() != null && i < n - 2){
        pos_i = applyForwardConstraint(i);
      }
      _positions.set(i, _move(pos_i1, pos_i, dist_i));
      change += Vector.distance(pos_i, _positions.get(i));
    }
    return change;
  }

  protected void _backwardReachingUnconstrained(){ //Assume that all axis are aligned
    List<NodeInformation> chain = _context.usableChainInformation();
    for(int i = 0; i < chain.size() - 1; i++){
      Vector pos_i = _positions.get(i);
      Vector pos_i1 = _positions.get(i + 1);
      float dist_i = _distances.get(i + 1);
      if(dist_i <= 10e-4){
        _positions.set(i+1, _positions.get(i));
        continue;
      }
      Vector new_pos_i1 = _move(pos_i, pos_i1, dist_i);
      _positions.set(i+1, new_pos_i1);
      //update node
      //chain.get(i+1).updateCacheUsingReference();
      chain.get(i+1).node().setTranslation(Vector.subtract(new_pos_i1, pos_i));
      chain.get(i+1).setPositionCache(new_pos_i1);
      //chain.get(i+1).setPositionWithCache(_positions.get(i+1));
    }
  }

  protected void _backwardReachingConstrained(){
    List<NodeInformation> chain = _context.usableChainInformation();
    for(int i = 0; i < chain.size() - 1; i++){
      Vector pos_i1 = _positions.get(i + 1);
      float dist_i = _distances.get(i + 1);
      if(dist_i <= 10e-4){
        _positions.set(i+1, _positions.get(i));
        //update node orientation
        chain.get(i+1).updateCacheUsingReference();
        continue;
      }
      //Do operations in Local coordinates
      NodeInformation j_i = chain.get(i);
      NodeInformation j_i1 = chain.get(i+1);
      Vector desired = j_i.locationWithCache(pos_i1);
      Vector current = j_i1.node().translation();
      //Find Rotation
      Quaternion delta = new Quaternion(current, desired);
      chain.get(i).rotateAndUpdateCache(delta, true);
      //Update cache of next Node
      chain.get(i+1).updateCacheUsingReference();
      _positions.set(i+1, chain.get(i+1).positionCache());
    }
  }


  @Override
  protected boolean _iterate() {
    if (_context.target() == null) return true;
    List<NodeInformation> chain = context().usableChainInformation();
    _current = 10e10f; //Keep the current error
    //NodeInformation._updateCache(_context.usableChainInformation());

    //Core algorithm
    Node end = context().endEffectorInformation().node();
    Vector initial = chain.get(0).positionCache();
    //Assume that end effector reaches target
    _positions.set(chain.size() - 1, context().target().position().get());
    _forwardReaching();
    //Restore root position
    _positions.set(0, initial);
    if(!_workInOrientationSpace){
      _backwardReachingUnconstrained();
    }
    else{
      _backwardReachingConstrained();
    }

    //Obtain current error
    if (_context.debug()) System.out.println("Current error: ");
    //measure the error depending on position error
    _current = context().error(_context.usableChainInformation().get(_context.endEffectorId()), _context.worldTarget());
    if (_context.debug()) System.out.println("Current :" + _current + "Best error: " + _best);

    //Define dead lock if eff does not move to a better position after a given number of iterations
    if (_enableDeadLockResolution) {
      if (context().deadlockCounter() == context().lockTimesCriteria()) { //apply random perturbation
        _totalDeadlock += 1;
        for (int i = 0; i < _context.endEffectorId(); i++) {
          NodeInformation j_i = _context.usableChainInformation().get(i);
          Quaternion q = Quaternion.random();
          if (j_i.node().constraint() != null) j_i.node().constraint().constrainRotation(q, j_i.node());
          j_i.node().rotate(q);
        }
        NodeInformation._updateCache(_context.usableChainInformation());
        context().resetDeadlockCounter();
        _current = 10e10f;
      }
    }
    _update(); //update if required
    if (_enableDeadLockResolution) {
      if (Math.abs(_best - _previousBest) < _maxError * 0.1f) { //DeadLock
        context().incrementDeadlockCounter();
      } else {
        context().resetDeadlockCounter();
      }
      _previousBest = _best;
    }

    if (error() <= _maxError) {
      return true;
    }
    return false;

  }

  protected Vector applyForwardConstraint(int i){
    NodeInformation j_i = context().usableChainInformation().get(i);
    NodeInformation j_i1 = context().usableChainInformation().get(i+1);
    NodeInformation j_i2 = context().usableChainInformation().get(i+2);
    Vector o = _positions.get(i);
    Vector p = _positions.get(i+1);
    Vector q = _positions.get(i+2);
    //Convenient transformations to use local constraints in forward stage
    Constraint c_i1 = j_i1.node().constraint();
    Vector x = j_i1.locationWithCache(j_i);
    Vector y = j_i1.locationWithCache(j_i2);
    Vector z = j_i1.locationWithCache(Vector.add(Vector.subtract(q,p), j_i1.positionCache()));
    Vector w = j_i1.locationWithCache(Vector.add(Vector.subtract(o,p), j_i1.positionCache()));

    Quaternion delta = new Quaternion(z,y);
    w = delta.rotate(w);
    Quaternion desired = new Quaternion(w, x);
    Quaternion constrained = c_i1.constrainRotation(desired, j_i1.node());

    Vector target = x.get();
    target.normalize();
    target.multiply(w.magnitude());
    target = constrained.inverseRotate(target);
    target = delta.inverseRotate(target);
    target = j_i1.orientationCache().rotate(target);
    target.add(p);

    return target;
  }


  @Override
  protected void _update() {
    if (_current < _best) {
      for (int i = 0; i < _context.endEffectorId() + 1; i++) {
        if(!_workInOrientationSpace) {
          _context.chain().get(i).setTranslation(_context.usableChain().get(i).translation().get());
        } else {
          _context.chain().get(i).rotation()._quaternion[0] = _context.usableChain().get(i).rotation()._quaternion[0];
          _context.chain().get(i).rotation()._quaternion[1] = _context.usableChain().get(i).rotation()._quaternion[1];
          _context.chain().get(i).rotation()._quaternion[2] = _context.usableChain().get(i).rotation()._quaternion[2];
          _context.chain().get(i).rotation()._quaternion[3] = _context.usableChain().get(i).rotation()._quaternion[3];
        }

        if(i > 0)_context.chainInformation().get(i).updateCacheUsingReference();
        else _context.chainInformation().get(i).setCache(_context.chainInformation().get(i).node().position().get(),
            _context.chainInformation().get(i).node().orientation().get());
      }
      _best = _current;
    }
  }

  @Override
  protected boolean _changed() {
    if (_context.target() == null) {
      _context.setPreviousTarget(null);
      return false;
    } else if (_context.previousTarget() == null) {
      return true;
    }
    return !(_context.previousTarget().position().matches(_context.target().position()) && _context.previousTarget().orientation().matches(_context.target().orientation()));
  }

  @Override
  protected void _reset() {
    _context.setPreviousTarget(_context.target() == null ? null : Node.detach(_context.target().position().get(), _context.target().orientation().get(), 1));
    //Copy original state into chain
    _context.copyChainState(_context.chainInformation(), _context.usableChainInformation());
    //Update cache
    NodeInformation._updateCache(_context.chainInformation());
    NodeInformation._copyCache(_context.chainInformation(), _context.usableChainInformation());
    _iterations = 0;
    _totalDeadlock = 0;
    _context.update();
    if (_context.target() != null) {
      _best = context().error(_context.chainInformation().get(_context.endEffectorId()), _context.target());
    } else {
      _best = 10e10f;
    }
    _previousBest = 10e10f;
    context().resetDeadlockCounter();
    _init();
  }

  @Override
  public float error() {
    return context().error(_context.chain().get(_context.endEffectorId()).position(), _context.worldTarget().position(),
        _context.chain().get(_context.endEffectorId()).orientation(), _context.worldTarget().orientation(), 1, _context.orientationWeight());
  }

  @Override
  public void setTarget(Node endEffector, Node target) {
    _context.setTarget(endEffector, target);
  }

  public void setTarget(Node target) {
    _context.setTarget(target);
  }

  public Node target() {
    return _context.target();
  }

  public boolean changed() {
    return _changed();
  }

  public void reset() {
    _reset();
  }

  public float positionError() {
    return _context.positionError(_context.chain().get(_context.endEffectorId()).position(), _context.worldTarget().position());
  }

  public float orientationError() {
    return _context.orientationError(_context.chain().get(_context.endEffectorId()).orientation(), _context.worldTarget().orientation(), true);
  }

  public float orientationError(boolean angles) {
    return _context.orientationError(_context.chain().get(_context.endEffectorId()).orientation(), _context.worldTarget().orientation(), angles);
  }

  public float bestError() {
    return _best;
  }

  //Initialize info about positions and distances
  protected void _init(){
    _positions.clear();
    _distances.clear();
    Vector prev = context().chain().get(0).reference() == null ? new Vector() : context().chain().get(0).reference().position();
    for(NodeInformation nodeInformation : context().chainInformation()){
      Vector curr = nodeInformation.positionCache();
      _positions.add(curr.get());
      _distances.add(Vector.distance(prev, curr));
      prev = curr;
    }
  }

}
