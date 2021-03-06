package nub.ik.solver;

import nub.core.Node;
import nub.ik.solver.heuristic.*;
import nub.primitives.Quaternion;
import nub.primitives.Vector;

import java.util.List;

public class GHIK extends Solver {
    public static boolean log = false;
    public enum HeuristicMode{
        CCD, BFIK_CCD,
        TIK, BFIK_TIK,
        TRIK, BFIK_TRIK,
        ECTIK, BFIK,
        FABRIK //DELETE THIS SOON!
    }
    protected boolean _swapOrder = false; //swap the order of traversal at each iteration
    protected boolean _enableDeadLockResolution = false;
    protected Context _context;
    protected HeuristicMode _heuristicMode;
    protected Heuristic _heuristic, _twistHeuristic;

    //Steady state algorithm
    protected float _current = 10e10f, _best = 10e10f, _previousBest = 10e10f;
    protected int _stepCounter;
    protected int _totalDeadlock = 0;
    protected boolean _enableTwist; //Apply a twisting movement after each step

    public void enableDeadLockResolution(boolean enable) {
        _enableDeadLockResolution = enable;
    }

    public int totalDeadlock(){
        return _totalDeadlock;
    }


    public GHIK(List<? extends Node> chain, Node target, HeuristicMode mode, boolean debug) {
        super();
        this._context = new Context(chain, target, debug);
        _context.setSolver(this);
        _setHeuristicMode(mode);
        _twistHeuristic = new Twist(_context);
        _enableTwist = false;
        _context.setSingleStep(false);
    }

    public GHIK(List<? extends Node> chain, HeuristicMode mode) {
        this(chain, null, mode, false);
    }

    public GHIK(List<? extends Node> chain, HeuristicMode mode, boolean debug) {
        this(chain, null, mode, debug);
    }

    public GHIK(List<? extends Node> chain, Node target, HeuristicMode mode){
        this(chain, target, mode, false);
    }



    protected void _setHeuristicMode(HeuristicMode mode) {
        switch (mode) {
            case CCD: {
                _heuristic = new CCD(_context);
                break;
            }
            case BFIK_CCD: {
                _heuristic = new BackAndForth(_context, BackAndForth.Mode.CCD);
                break;
            }
            case TIK: {
                _heuristic = new TIK(_context);
                break;
            }
            case BFIK_TIK: {
                _heuristic = new BackAndForth(_context, BackAndForth.Mode.TRIANGULATION);
                break;
            }
            case TRIK: {
                _heuristic = new TRIK(_context);
                break;
            }
            case BFIK_TRIK: {
                _heuristic = new BackAndForth(_context, BackAndForth.Mode.TRIK);
                break;
            }
            case ECTIK: {
                _heuristic = new ECTIK(_context);
                break;
            }
            case BFIK: {
                _heuristic = new BFIK(_context);
                break;
            }
        }
        _heuristicMode = mode;
    }

    public Context context() {
        return _context;
    }

    public boolean enableTwist() {
        return _enableTwist;
    }

    public void enableTwist(boolean enable) {
        _enableTwist = enable;
    }

    public boolean direction() {
        return _context.direction();
    }

    public void setSwapOrder(boolean swap){
        _swapOrder = swap;
    }

    public void setHeuristic(Heuristic heuristic){
        _heuristic = heuristic;
    }

    public HeuristicMode mode() {
        return _heuristicMode;
    }

    protected boolean _iterateStepByStep() {
        System.out.println("On step " + _stepCounter);
        if (_stepCounter == 0) {
            if (_context.target() == null) return true; //As no target is specified there is no need to solve IK
            _current = 10e10f; //Keep the current error
            _heuristic.prepare();
        } else if (_stepCounter < _context.endEffectorId() + 1) {
            int i = context().topToBottom() ? _stepCounter - 1 : _context.endEffectorId() - _stepCounter;
            _heuristic.applyActions(i);
            if (_enableTwist) _twistHeuristic.applyActions(i);
            if (context().topToBottom()) _context.usableChainInformation().get(i + 1).updateCacheUsingReference();

        } else {
            _current = context().error(_context.usableChainInformation().get(_context.endEffectorId()), _context.worldTarget());
            _update();
            _stepCounter = -1;
            if (_swapOrder) {
                context().setTopToBottom(!context().topToBottom());
            }
        }
        _stepCounter++;
        return false;

    }


    @Override
    protected boolean _iterate() {
        if(GHIK.log) showInfo("Begin iterate " + "iteration " + _iterations, _context);
        if (_context.target() == null) return true;
        if (_context.singleStep()) return _iterateStepByStep();
        _current = 10e10f; //Keep the current error
        _heuristic.prepare();
        if (context().topToBottom()) {
            for (int i = 0; i < _context.endEffectorId(); i++) {
                _heuristic.applyActions(i);
                if (_enableTwist) _twistHeuristic.applyActions(i);
                //update next joint cache based on current one
                _context.usableChainInformation().get(i + 1).updateCacheUsingReference();
            }
        } else {
            for (int i = _context.endEffectorId() - 1; i >= 0; i--) {
                //if(IKSolver.debugERROR) showInfo("Begin iterate " + "iteration " + _iterations + " joint " + i, _context);
                _heuristic.applyActions(i);
            }
        }

        if (_swapOrder) {
            context().setTopToBottom(!context().topToBottom());
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
                    if (j_i.node().constraint() != null){
                        q = j_i.node().constraint().constrainRotation(q, j_i.node());
                    }
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

    public float bestError() {
        return _best;
    }

    @Override
    protected void _update() {
        //if(IKSolver.debugERROR) showInfo("Begin Update " + "iteration " + _iterations, _context);
        if (_context.singleStep()) System.out.println("Current : " + _current + " best " + _best);
        if (_current < _best) {
            for (int i = 0; i < _context.endEffectorId() + 1; i++) {
                _context.chain().get(i).rotation()._quaternion[0] = _context.usableChain().get(i).rotation()._quaternion[0];
                _context.chain().get(i).rotation()._quaternion[1] = _context.usableChain().get(i).rotation()._quaternion[1];
                _context.chain().get(i).rotation()._quaternion[2] = _context.usableChain().get(i).rotation()._quaternion[2];
                _context.chain().get(i).rotation()._quaternion[3] = _context.usableChain().get(i).rotation()._quaternion[3];
                if(i > 0)_context.chainInformation().get(i).updateCacheUsingReference();
                else _context.chainInformation().get(i).setCache(_context.chainInformation().get(i).node().position().get(),
                        _context.chainInformation().get(i).node().orientation().get());
            }
            //if(IKSolver.debugERROR) showInfo("End Update " + "iteration " + _iterations, _context);
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

        if (_context.singleStep()) _stepCounter = 0;
        if (_swapOrder && context().topToBottom() == true) {
            context().setTopToBottom(false);
        }
        context().resetDeadlockCounter();
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


    public Node target() {
        return _context.target();
    }

    public boolean changed() {
        return _changed();
    }

    public void reset() {
        _reset();
    }

    @Override
    public float error() {
        return context().error(_context.chain().get(_context.endEffectorId()).position(), _context.worldTarget().position(),
                _context.chain().get(_context.endEffectorId()).orientation(), _context.worldTarget().orientation(), 1, _context.orientationWeight());
    }

    public float error(float wo) {
        return context().error(_context.chain().get(_context.endEffectorId()).position(), _context.worldTarget().position(),
            _context.chain().get(_context.endEffectorId()).orientation(), _context.worldTarget().orientation(), 1, wo);
    }


    @Override
    public void setTarget(Node endEffector, Node target) {
        _context.setTarget(endEffector, target);
    }

    public void setTarget(Node target) {
        _context.setTarget(target);
    }

    public void setEndEffector(int id){
        _context.setEndEffector(id);
    }

    public void setEndEffector(Node node){
        _context.setEndEffector(node);
    }

    public Heuristic heuristic() {
        return _heuristic;
    }

    public void set2D(boolean is2D){
        _context.set2D(is2D);
    }

    public void setDirection(boolean direction){
        _context.setDirection(direction);
    }

    public static void showInfo(String name, Context context){
        System.out.println("--------------NAME " + name + "----------------------------------");
        System.out.println("-------------------------------------------------------------------");
        System.out.println("-------------------------------------------------------------------");
        System.out.println("main");
        for(NodeInformation ni : context.chainInformation()){
            Vector cache = ni.positionCache();
            Vector real = ni.node().position();
            System.out.println("Rot : " + ni.node().rotation() + "Cache : " + cache + " Real : " + real + " Diff" + Vector.distance(cache, real));
        }

        System.out.println("aux");
        for(NodeInformation ni : context.usableChainInformation()){
            Vector cache = ni.positionCache();
            Vector real = ni.node().position();
            System.out.println("Rot : " + ni.node().rotation() + "Cache : " + cache + " Real : " + real + " Diff" + Vector.distance(cache, real));
        }

        System.out.println("Target " + context.worldTarget().position());
        System.out.println("Error " + context.error(context.endEffectorInformation(), context.target()));

        System.out.println("<<<<<------------------------------------------------------------------->>>>>");
        System.out.println("<<<<<------------------------------------------------------------------->>>>>");
        System.out.println("<<<<<--------------------------   END   -------------------------------->>>>>");

    }

}
