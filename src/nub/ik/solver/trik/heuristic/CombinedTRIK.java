package nub.ik.solver.trik.heuristic;

import nub.ik.solver.trik.Context;
import nub.ik.solver.trik.NodeInformation;

public class CombinedTRIK extends Heuristic{
    protected float _trikFraction; //Define how much of the iterations must use TRIK Heuristic (parameter between 0 and 1)
    protected int _trikIterations;
    protected int _times = 2; //How many passes of CCD must the combined heuristic perform

    public CombinedTRIK(Context context, float trikFraction, int times){
        super(context);
        _trikFraction = trikFraction;
        _times = times;
    }

    public CombinedTRIK(Context context, float trikFraction){
        this(context, trikFraction, 2);
    }

    public CombinedTRIK(Context context){
        this(context, 0.3f, 2);
    }

    @Override
    public void prepare() {
        //Update cache of usable chain
        NodeInformation._updateCache(_context.usableChainInformation());
        _trikIterations = (int) (_context.solver().maxIterations() * _trikFraction);
    }

    @Override
    public void applyActions(int i) {
        if(_context.currentIteration() < _trikIterations){
            TRIK.applyTRIK(this, i);
            NodeInformation j_i1 = _context.usableChainInformation().get(i + 1);
            //for (int t = 0; t < _times; t++) {
                j_i1.updateCacheUsingReference();
                CCD.applyCCD(this, i + 1, 0.3f);
                CCD.applyCCD(this, i, 0.3f);
            //}
            j_i1.updateCacheUsingReference();
            CCD.applyOrientationalCCD(this, i + 1);
        } else {
            if(_context.currentIteration() == _trikIterations) {
                if( (_context.topToBottom() && i == 0) || (!_context.topToBottom() && i == _context.endEffectorId() - 1)){
                    //Reset usable chain
                    _context.copyChainState(_context.chainInformation(), _context.usableChainInformation());
                    //Update cache
                    NodeInformation._updateCache(_context.chainInformation());
                    NodeInformation._copyCache(_context.chainInformation(), _context.usableChainInformation());
                }
            }
            Combined.applyCombined(this, i, _times);
        }
    }

    @Override
    public NodeInformation[] nodesToModify(int i) {
        return new NodeInformation[]{_context.usableChainInformation().get(i - 1), _context.usableChainInformation().get(i)};
    }
}
