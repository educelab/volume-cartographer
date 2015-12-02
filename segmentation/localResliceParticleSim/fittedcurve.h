#pragma once

#ifndef _VOLCART_SEGMENTATION_FITTED_CURVE_H_
#define _VOLCART_SEGMENTATION_FITTED_CURVE_H_

#include <vector>
#include <tuple>
#include <cinttypes>
#include <iostream>
#include <Eigen/Dense>

namespace volcart {

namespace segmentation {

// Simple curve fitting using nonlinear least squares
template <typename PointScalar=double, int32_t Degree=4>
class FittedCurve {
public:
	using PointType = typename std::tuple<PointScalar, PointScalar>;
    using PointVectorType = typename std::vector<PointType>;

    FittedCurve() { }

    FittedCurve(const PointVectorType& points)
    {
        fitPoints(points);
    }

    // Fit 'points' and generate coefficients for polynomial curve
    void fitPoints(const PointVectorType& points)
    {
        std::tie(x_, y_) = make_matrices(points);    

        // Seems to use least squares computation. Check Eigen documentation for
        // the Eigen::ComputeThinU/V.
        Eigen::VectorXd a = x_.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(y_);
        for (uint32_t i = 0; i < a.size(); ++i) {
            coefficients_.push_back(a(i));
        }
    }

    // Evaluate the polynomial at 'x'
    PointScalar at(PointScalar x) const
    {
        auto y = coefficients_.at(0);
        for (size_t i = 1; i < coefficients_.size(); ++i) {
            y += coefficients_.at(i) * std::pow(x, i);
        }
        return y;
    }

    // Pretty printing
    friend std::ostream& operator<<(std::ostream& s, FittedCurve& f)
    {
        s << std::fixed;
        s << *(f.coefficients_.begin()) << " + ";
        for (size_t i = 1; i < f.coefficients_.size() - 1; ++i) {
            s << f.coefficients_.at(i) << "*x" << i << " + ";
        }
        return s << *(f.coefficients_.end()) << "*x" << f.coefficients_.size() - 1;
    }

private:
    // Create the matrices for use later on in generating poly coefficients
    std::tuple<Eigen::MatrixXd, Eigen::VectorXd> make_matrices(
            const PointVectorType& points) const
    {
        Eigen::MatrixXd x(points.size(), Degree + 1);
        Eigen::VectorXd y(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            for (int32_t d = 0; d < Degree + 1; ++d) {
                x(i, d) = std::pow(std::get<0>(points.at(i)), d);
            }
            y(i) = std::get<1>(points.at(i));
        }
        return std::make_tuple(x, y);
    }

    std::vector<PointScalar> coefficients_;
    Eigen::MatrixXd x_;
    Eigen::VectorXd y_;
};

}

}

#endif
