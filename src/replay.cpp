#include "replay.hpp"

#include <fstream>
#include <iostream>
#include <iterator>

/*
auto system: replay a set of inputs from a driver with sensor correction.to
do this, construct a mapping from change in sensor reading to inputs, and use
it to calculate an input to add to the previously recorded one. this mapping
can be constructed once and stored, saving processing time. through this, the
robot "learns" what targets look like and can gain benefits of global state
tracking without the complexity or tuning.

y: readings
u: inputs
*/

using namespace okapi;

int Recording::length() {
	return inputs_.size();
}

void Recording::clear() {
	inputs_.clear();
	outputs_.clear();
}

void Recording::addFrame(Inputs input, OutputVector output) {
	inputs_.push_back(input);
	outputs_.push_back(output);
}

void Recording::buildModel() {
 	std::vector<double> inputVector{};
 	inputVector.reserve(length());

 	for (int i = 0; i < length(); i++) {
 		inputVector[i] = inputs_[i][0];
 	}
 	model_.row(0) = regress(inputVector, outputs_);
 	for (int i = 0; i < length(); i++) {
 		inputVector[i] = inputs_[i][1];
 	}
 	model_.row(1) = regress(inputVector, outputs_);
}

Matrixd<1, OUTPUTS> Recording::regress(std::vector<double> input,
                                       std::vector<OutputVector> outputs) {
 	Eigen::Matrix3Xd U = Eigen::MatrixXd(3, outputs.size());
 	for (int i = 0; i < length(); i++) {
 		U.col(i) = outputs[i];
 	}
 	Eigen::VectorXd y = Eigen::VectorXd(outputs.size());
 	for (int i = 0; i < length(); i++) {
 		y(i) = input[i];
 	}
 	return U.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);
}

Inputs Recording::getFrame(int i, OutputVector y) {
	Inputs base = inputs_[i];
	// OutputVector residual = outputs_[i] - y;
	// InputVector correction = model_ * residual;

	// std::cout << base << " , " << correction << std::endl;
	return base;
}

void Recording::write(std::string path) {
	std::ofstream file(path);
	file << length();
	std::ostream_iterator<Inputs> iter1(file);
	std::copy(inputs_.begin(), inputs_.end(), iter1);
	std::ostream_iterator<OutputVector> iter2(file);
	std::copy(outputs_.begin(), outputs_.end(), iter2);
}
void Recording::read(std::string path) {
	std::ifstream file(path, std::iostream::in);
	int32_t length;
	file >> length;
	inputs_.clear();
	inputs_.reserve(length);
	outputs_.clear();
	outputs_.reserve(length);
	std::istream_iterator<Inputs> iter1(file);
	for (int i = 0; i < length; i++) {
		// inputs_.push_back(iter1.next());
	}
	inputs_ = std::vector<Inputs>(std::istream_iterator<Inputs>(file),
	                              std::istream_iterator<Inputs>());
}
