#include "typhoon/filter.hpp"

using namespace filter;

std::tuple<Matrixd<STATES, WEIGHTS>, WeightVector, WeightVector> generateSigmas(
    StateVector mean, Matrixd<STATES, STATES> variance, double alpha = 1e-3,
    double kappa = 1, double beta = 2) {
	double defaultWeight = 1 / (2 * alpha * alpha * kappa);

	Matrixd<STATES, WEIGHTS> sigmas = Matrixd<STATES, WEIGHTS>::Zero();
	WeightVector meanWeights = WeightVector::Constant(defaultWeight);
	WeightVector varianceWeights = WeightVector::Constant(defaultWeight);

	sigmas.colwise() = mean;
	meanWeights(0) = (alpha * alpha * kappa - STATES) / (alpha * alpha * kappa);
	varianceWeights(0) = meanWeights(0) + 1 - alpha * alpha + beta;

	Matrixd<STATES, STATES> a = variance.llt().matrixL();

	sigmas.block(0, 1, STATES, STATES) += alpha * sqrt(kappa) * a;
	sigmas.block(0, STATES + 1, STATES, STATES) -= alpha * sqrt(kappa) * a;

	return std::make_tuple(sigmas, meanWeights, varianceWeights);
}

PositionFilter::PositionFilter(StateVector mean0,
                               Matrixd<STATES, STATES> variance0,
                               Matrixd<OUTPUTS, OUTPUTS> measurementNoise)
    : xMean_(mean0),
      xVariance_(variance0),
      measurementNoise_(measurementNoise) {
	std::tie(xSigmas_, meanWeights_, varianceWeights_) =
	    generateSigmas(xMean_, xVariance_);
}

void PositionFilter::step(InputVector input, OutputVector output) {
	update(output);
	predict(input);
}

void PositionFilter::predict(InputVector input) {
	std::tie(xSigmas_, meanWeights_, varianceWeights_) =
	    generateSigmas(xMean_, xVariance_);
	for (auto col : xSigmas_.colwise()) {
		col = modelFunction_(col, input);
	}
	xMean_ = xMeanFunction_(xSigmas_, meanWeights_);
	xVariance_.setZero();
	for (int i = 0; i < xSigmas_.cols(); i++) {
		auto residual = xResidualFunction_(xSigmas_.col(i), xMean_);
		xVariance_ += varianceWeights_(i) * residual * residual.transpose();
	}
}

void PositionFilter::update(OutputVector output) {
	Matrixd<OUTPUTS, WEIGHTS> zSigmas;
	for (int i = 0; i < zSigmas.cols(); i++) {
		zSigmas.col(i) = measurementFunction_(xSigmas_.col(i));
	}

	OutputVector zMean = zMeanFunction_(zSigmas, meanWeights_);
	Matrixd<OUTPUTS, OUTPUTS> zVariance = measurementNoise_;
	for (int i = 0; i < zSigmas.cols(); i++) {
		auto residual = zResidualFunction_(zSigmas.col(i), zMean);
		zVariance += varianceWeights_(i) * residual * residual.transpose();
	}

	Matrixd<STATES, OUTPUTS> crossCovariance = Matrixd<STATES, OUTPUTS>::Zero();
	for (int i = 0; i < zSigmas.cols(); i++) {
		auto xResidual = xResidualFunction_(xSigmas_.col(i), xMean_);
		auto zResidual = zResidualFunction_(zSigmas.col(i), zMean);
		crossCovariance += varianceWeights_(i) * xResidual * zResidual.transpose();
	}

	Matrixd<STATES, OUTPUTS> gain = crossCovariance * zVariance.inverse();

	xMean_ += gain * (output - zMean);
	xVariance_ -= gain * zVariance * gain.transpose();
}
