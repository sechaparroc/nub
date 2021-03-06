/****************************************************************************************
 * nub
 * Copyright (c) 2019 National University of Colombia, https://visualcomputing.github.io/
 * @author Sebastian Chaparro, https://github.com/sechaparroc
 * @author Jean Pierre Charalambos, https://github.com/VisualComputing
 *
 * All rights reserved. A 2D or 3D scene graph library providing eye, input and timing
 * handling to a third party (real or non-real time) renderer. Released under the terms
 * of the GPL v3.0 which is available at http://www.gnu.org/licenses/gpl.html
 ****************************************************************************************/

package nub.ik.solver;

import nub.core.Node;

import java.util.Iterator;

/**
 * A Solver is a convenient class to solve IK problem,
 * Given a Chain or a Tree Structure of Nodes, this class will
 * solve the configuration that the Nodes must have to reach
 * a desired position.
 */

public abstract class Solver {
  //TODO : Add visual hints to show how the solver's algorithm works.
  //TODO paper idea: optimize values per _solver / timer / local config
  protected float _maxError = 0.01f;
  protected int _maxIterations = 50;
  protected float _minDistance = 0.01f;
  protected float _timesPerFrame = 5.f;
  protected float _frameCounter = 0;
  protected int _iterations = 0;
  protected int _last_iteration = 0; //TODO : Clean this!
  protected boolean _change_temp = false; //TODO : Clean this!
  protected boolean _accumulate = false;
  protected float _accumulatedError = 0; //TODO : Remove this
  protected int _accumulatedTimes = 0; //TODO : Remove this

  /*Getters and setters*/
  public int lastIteration() {
    return _last_iteration;
  }

  public int iteration() {
    return _iterations;
  }

  public void setMaxError(float maxError) {
    _maxError = maxError;
  }

  public int maxIterations(){
    return _maxIterations;
  }

  public void setMaxIterations(int maxIterations) {
    _maxIterations = maxIterations;
  }

  public void setMinDistance(float minDistance) {
    _minDistance = minDistance;
  }

  public void setTimesPerFrame(float timesPerFrame) {
    _timesPerFrame = timesPerFrame;
  }

  public void hasChanged(boolean change) {
    _change_temp = change;
  }

  public float averageError() {
    return _accumulatedError /_accumulatedTimes;
  }

  public int accumulatedTimes(){
    return _accumulatedTimes;
  }

  /*Performs an Iteration of Solver Algorithm */
  protected abstract boolean _iterate();

  protected abstract void _update();

  protected abstract boolean _changed();

  protected abstract void _reset();

  public abstract float error();

  public void change(boolean change) {
    _change_temp = change;
  }

  public boolean solve() {
    //Reset counter
    if (_changed() || _change_temp) {
      _reset();
      _last_iteration = 0;
      _accumulate = true;
      _change_temp = false;
    }

    if (_iterations >= _maxIterations) {
      return true;
    }

    _frameCounter += _timesPerFrame;

    while (Math.floor(_frameCounter) > 0) {
      //Returns a boolean that indicates if a termination condition has been accomplished
      if (_iterate()) {
        _last_iteration = _iterations + 1;
        _iterations = _maxIterations;
        _frameCounter = 0;
      } else {
        _iterations += 1;
        _last_iteration = _iterations;
        _frameCounter -= 1;
      }
    }

    if (_iterations >= _maxIterations) {
      if (_accumulate) {
        _accumulatedError += error();
        _accumulatedTimes += 1;
        _accumulate = false;
      }
    }

    //update positions
    _update();
    return false;
  }

  public abstract void setTarget(Node endEffector, Node target);
}
