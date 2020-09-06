package ik.trik.vizSteps;

import nub.ik.solver.trik.Context;
import nub.ik.solver.trik.NodeInformation;
import nub.ik.solver.trik.heuristic.*;
import nub.processing.Scene;

public class BackAndForthViz extends Heuristic {
  /**
   * The idea of this solver is to apply a local action over a couple of joints followed by a CCD correction step.
   */
  public enum Mode{ CCD, TRIK, TRIANGULATION }

  public Viz viz;
  protected int _times = 2;
  protected BackAndForth.Mode _mode;

  public BackAndForthViz(Context context, BackAndForth.Mode mode, Viz viz) {
    super(context);
    this.viz = viz;
    this.viz.setHeuristic(this);
    _mode = mode;
  }

  public BackAndForthViz(Context context, Viz viz) {
    this(context, BackAndForth.Mode.CCD, viz);
  }


  @Override
  public void prepare() {
    //Update cache of usable chain
    NodeInformation._updateCache(_context.usableChainInformation());
    if (viz != null){
      viz.clearFigures();
      viz.addFrame("dummy"); //Dummy frame with no information :|
      viz.sequence.clear();
      viz.addHighlightNode("P\u2080", context().usableChain().get(0), Scene.pApplet.color(0,255,0));
      viz.addHighlightNode("Pe", context().endEffectorInformation().node(), Scene.pApplet.color(0,255,0));
      viz.addHighlightNode("Pt", context().target(), Scene.pApplet.color(0,255,0));
      viz.addFrame("Initial configuration"); //Just add the initial image
    }
  }

  @Override
  public void applyActions(int i) {
    switch (_mode){
      case TRIANGULATION:{
        TriangulationViz.applyTriangulation(viz, this, i, true, _context.applyDelegation());
        break;
      }
      case TRIK:{
        TRIKViz.applyTRIK(viz, this, i);
        break;
      }
      case CCD:{
        CCDViz.applyCCD(viz,this, i, _context.applyDelegation());
      }
    }
    if(i >= _context.endEffectorId() - 1){
      CCD.applyOrientationalCCD(this, i);
      return;
    }

    //Show the final result after the main Heuristic is applied
    if(viz != null){
      viz.clearFigures();
      //highlight current nodes
      viz.addHighlightNode("", context().usableChain().get(i), Scene.pApplet.color(252, 186,3));
      viz.addHighlightNode("", context().usableChain().get(i + 1), Scene.pApplet.color(0, 255, 255));
      viz.addFrame("New Configuration after main heuristic");
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
    //Show the final result after the ccd  pass is performed
    if(viz != null){
      viz.clearFigures();
      //highlight current nodes
      viz.addHighlightNode("", context().usableChain().get(i), Scene.pApplet.color(252, 186,3));
      viz.addHighlightNode("", context().usableChain().get(i + 1), Scene.pApplet.color(0, 255, 255));
      viz.addFrame("New Configuration after CCD Pass");
    }
  }

  @Override
  public NodeInformation[] nodesToModify(int i) {
    return new NodeInformation[]{_context.usableChainInformation().get(i - 1), _context.usableChainInformation().get(i)};
  }
}
