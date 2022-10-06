#pragma once

#include "filter.hpp"

/*
 * state: (x, y, theta, left, right)
 * output: (theta, left, right)
 */

double angleSum(double angle1, double angle2);
StateVector meanStates(Matrixd<STATES, WEIGHTS> states, WeightVector weights);
OutputVector meanOutputs(Matrixd<OUTPUTS, WEIGHTS> outputs,
                         WeightVector weights);

StateVector addStates(StateVector a, StateVector b);
OutputVector subtractOutputs(OutputVector a, OutputVector b);

StateVector modelFunction(StateVector state, InputVector u, double trackWidth,
                          double dt);
OutputVector measurementFunction(StateVector state);
