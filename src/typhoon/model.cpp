#include "model.hpp"

#include "filter.hpp"

/*
 * state: (x, y, theta, left, right)
 * input: (leftSpeed, rightSpeed)
 * output: (theta, left, right)
 */

template <int X>
double angleMean(Vectord<X> angles, Vectord<X> weights) {
	Eigen::Array<double, X, 1> anglesX = weights.array() * angles.array().cos();
	Eigen::Array<double, X, 1> anglesY = weights.array() * angles.array().sin();

	return atan2(anglesY.sum(), anglesX.sum());
}

double angleSum(double angle1, double angle2) {
	double sum = fmod(angle1 + angle2, 2 * M_PI);
	if (sum < -M_PI) {
		sum += 2 * M_PI;
	}
	if (sum > M_PI) {
		sum -= 2 * M_PI;
	}
	return sum;
}

StateVector meanStates(Matrixd<STATES, WEIGHTS> states, WeightVector weights) {
	StateVector mean = StateVector::Zero();
	for (int i = 0; i < states.cols(); i++) {
		mean += weights(i) * states.col(i);
	}
	mean(2) = angleMean(states.row(2).transpose().eval(), weights);
	return mean;
}

OutputVector meanOutputs(Matrixd<OUTPUTS, WEIGHTS> outputs,
                         WeightVector weights) {
	OutputVector mean = OutputVector::Zero();
	for (int i = 0; i < outputs.cols(); i++) {
		mean += weights(i) * outputs.col(i);
	}
	mean(0) = angleMean(outputs.row(0).transpose().eval(), weights);
	return mean;
}

StateVector addStates(StateVector a, StateVector b) {
	StateVector sum = a + b;
	sum(2) = angleSum(a(2), b(2));
	return sum;
}

StateVector subtractStates(StateVector a, StateVector b) {
	return addStates(a, -b);
}

OutputVector subtractOutputs(OutputVector a, OutputVector b) {
	OutputVector residual = a - b;
	residual(0) = angleSum(a(0), -b(0));
	return residual;
}

StateVector modelFunction(StateVector state, InputVector u, double trackWidth,
                          double dt) {
	double forwardSpeed = (u(0) + u(1)) / 2;
	double turnSpeed = (u(1) - u(0)) / trackWidth;
	double dTheta = turnSpeed * dt;

	double dx, dy;
	if (fabs(turnSpeed) < 0.001) {
		dx = forwardSpeed * cos(state(2));
		dy = forwardSpeed * sin(state(2));
	} else {
		dx = forwardSpeed * (sin(state(2) + dTheta) - sin(state(2))) / turnSpeed;
		dy = forwardSpeed * (cos(state(2)) - cos(state(2) + dTheta)) / turnSpeed;
	}

	return addStates(state, {dx, dy, dTheta, u(0) * dt, u(1) * dt});
}

OutputVector measurementFunction(StateVector state) {
	return state.tail(3);
}
