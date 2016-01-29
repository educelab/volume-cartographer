#pragma once

#ifndef _VOLCART_SEGMENTATION_SPLINE_H_
#define _VOLCART_SEGMENTATION_SPLINE_H_

#include <unsupported/Eigen/Splines>
#include <vector>
#include <cassert>

namespace volcart {

namespace segmentation {

template <typename Scalar=double, uint32_t Degree=3>
class Spline {
public:
	using ScalarVector = std::vector<Scalar>;
	using SplineType = Eigen::Spline<Scalar, 2>;

	Spline() = default;

	Spline(const ScalarVector& xs, const ScalarVector& ys) :
		curveLength_(curveLength(xs, ys))
	{
		assert(xs.size() == ys.size() && "xs and ys must be same length\n");
		npoints_ = xs.size();
		points_ = makeWideMatrix(xs, ys);
		spline_ = Eigen::SplineFitting<SplineType>::Interpolate(points_, Degree);
		knots_ = spline_.knots();
	}

	Eigen::VectorXd knots() const { return knots_; }

	Eigen::MatrixXd ctrls() const { return spline_.ctrls(); }

	// Plain evaluation at t-space value t \in [0, 1]
	Pixel eval(Scalar t) const
	{
		assert(t >= 0 && t <= 1 && "t must be in range [0, 1]\n");
		Eigen::Vector2d val = spline_(t);
		return Pixel(val(0), val(1));
	}

private:
	Scalar curveLength_;
	std::vector<Scalar> resampledPoints_;
	Eigen::VectorXd knots_;
	size_t npoints_;
	Eigen::MatrixXd points_;
	SplineType spline_;

	Scalar curveLength(const ScalarVector& xs, const ScalarVector& ys) const
	{
		Scalar length = 0;
		for (size_t i = 0; i < xs.size() - 1; ++i) {
			Scalar xdiff = xs[i+1] - xs[i];
			Scalar ydiff = ys[i+1] - ys[i];
			length += std::sqrt(xdiff * xdiff + ydiff + ydiff);
		}
		return length;
	}

	// Calculate the knots for the curve (the points at which the curve will interpolate
	// exactly) on the curve
	Eigen::VectorXd calculateKnots(const ScalarVector& xs, const ScalarVector& ys) const
	{
		Eigen::VectorXd knots(xs.size());
		knots(0) = 0;
		Scalar length = 0;
		for (size_t i = 1; i < xs.size(); ++i) {
			Scalar xdiff = xs[i] - xs[i-1];
			Scalar ydiff = ys[i] - ys[i-1];
			length += std::sqrt(xdiff * xdiff + ydiff * ydiff);
			knots(i) = length / curveLength_;
		}
		return knots;
	}

	Eigen::MatrixXd makeWideMatrix(const ScalarVector& xs, const ScalarVector& ys)
	{
		Eigen::MatrixXd mat{2, xs.size()};
		mat.row(0) = Eigen::VectorXd::Map(xs.data(), xs.size());
		mat.row(1) = Eigen::VectorXd::Map(ys.data(), ys.size());
		return mat;
	}
};

template <typename Scalar>
using CubicSpline = Spline<Scalar, 3>;

}

}
#endif
