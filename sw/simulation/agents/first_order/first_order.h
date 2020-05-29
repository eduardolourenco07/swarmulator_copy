#ifndef FIRST_ORDER_H
#define FIRST_ORDER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"

using namespace std;

/**
 * This child class of agent implements the dynamics of a first order system using a kinematic model (BEBOP)
 */
class first_order: public Agent
{
public:
  /**
   * Constructor
   */
  first_order(int i, vector<float> state, float tstep);

  /**
   * State update implementation
   */
  vector<float> state_update(vector<float> state);

  /**
   * Animation openGL implementation for visualization
   */
  void animation();
};

#endif /*FIRST_ORDER_H*/
