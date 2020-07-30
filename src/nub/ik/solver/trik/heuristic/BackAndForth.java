package nub.ik.solver.trik.heuristic;

import nub.ik.solver.trik.NodeInformation;
import nub.ik.solver.trik.Context;

public class BackAndForth extends Heuristic {
    /**
     * The idea of this solver is to apply a local action over a couple of joints followed by a CCD correction step.
     */
    public enum Mode{ CCD, TRIK, TRIANGULATION }

    protected int _times = 2;
    protected Mode _mode;

    public BackAndForth(Context context, Mode mode) {
        super(context);
        _mode = mode;
    }

    public BackAndForth(Context context) {
        this(context, Mode.TRIK);
    }


    @Override
    public void prepare() {
        //Update cache of usable chain
        NodeInformation._updateCache(_context.usableChainInformation());
    }

    @Override
    public void applyActions(int i) {
        switch (_mode){
            case TRIANGULATION:{
                Triangulation.applyTriangulation(this, i, true, _context.applyDelegation());
                break;
            }
            case TRIK:{
                TRIK.applyTRIK(this, i);
                break;
            }
            case CCD:{
                CCD.applyCCD(this, i, _context.applyDelegation());
            }
        }
        if(i >= _context.endEffectorId() - 1){
            CCD.applyOrientationalCCD(this, i);
            return;
        }
        //Apply CCD Back and Forth k times
        NodeInformation j_i1 = _context.usableChainInformation().get(i + 1);
        for (int t = 0; t < _times; t++) {
            j_i1.updateCacheUsingReference();
            CCD.applyCCD(this, i + 1, _context.applyDelegation());
            CCD.applyCCD(this, i, _context.applyDelegation());
        }
        j_i1.updateCacheUsingReference();
        CCD.applyOrientationalCCD(this, i + 1);
    }

    @Override
    public NodeInformation[] nodesToModify(int i) {
        return new NodeInformation[]{_context.usableChainInformation().get(i - 1), _context.usableChainInformation().get(i)};
    }
}
