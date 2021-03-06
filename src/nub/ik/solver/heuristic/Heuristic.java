package nub.ik.solver.heuristic;

import nub.ik.solver.NodeInformation;
import nub.ik.solver.Context;

public abstract class Heuristic {
  /**
   * An heuristic defines how to modify a joint or a set of joints in order that a kinematic
   * chain reach its target position/orientation.
   * Each heuristic manages information provided by a Context. If required, an heuristic could also manage its own information.
   * e.g cache secondary goals, ...
   * <p>
   * As we want to explore actions of different heuristics we make the following assumptions:
   * -   An heuristic only requires to update its internal information at the beginning of an iteration.
   * -   The context information is updated so that action to be made at in-between step i is correct.
   * -   An heuristic could modify more than a node in a single step.
   * -   A Solver must determines the order of the actions, and the heuristics to use.
   */

  protected Context _context; //Contains the basic information a heuristic requires

  public Heuristic(Context context) {
    _context = context;
  }

  public abstract void prepare(); //This method is executed at the beginning of each iteration

  public abstract void applyActions(int i);

  public Context context(){
    return _context;
  }

  //This methods keeps at least the state of the chain.
  //Override it to save additional information
  public abstract NodeInformation[] nodesToModify(int i); //return the nodes that the heuristic will modify in a single step

}
