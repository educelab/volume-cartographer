#include "GaussianDistribution3D.h"
#include <cmath>
#include <cassert>

using namespace volcart;

GaussianDistribution3D::GaussianDistribution3D() :
	order_(Ordering::ZYX), radius_(1)
{
	init();
}

GaussianDistribution3D::GaussianDistribution3D(int32_t voxelRadius) :
	order_(Ordering::ZYX),
	radius_(voxelRadius)
{
	init();
}

GaussianDistribution3D::GaussianDistribution3D(int32_t voxelRadius, GaussianDistribution3D::Ordering order) :
	order_(order),
	radius_(voxelRadius)
{
	init();
}

void GaussianDistribution3D::init()
{
	sideLength_ = 2 * radius_ + 1;
	values_ = std::unique_ptr<double[]>(new double[sideLength_ * sideLength_ * sideLength_]);
	make_gaussian_dist();
}

const double& GaussianDistribution3D::at(size_t x, size_t y, size_t z) const
{
	assert(order_ == GaussianDistribution3D::Ordering::XYZ ||
		   order_ == GaussianDistribution3D::Ordering::ZXY ||
		   order_ == GaussianDistribution3D::Ordering::ZYX ||
		   "order_ must be one of {XYZ, ZYX, ZXY}\n");
	switch (order_) {
		case GaussianDistribution3D::Ordering::XYZ:
			return values_[x * sideLength_ * sideLength_ + y * sideLength_ + z];
		case GaussianDistribution3D::Ordering::ZXY:
			return values_[z * sideLength_ * sideLength_ + x * sideLength_ + y];
		case GaussianDistribution3D::Ordering::ZYX:
			return values_[z * sideLength_ * sideLength_ + y * sideLength_ + x];
	}
}

void GaussianDistribution3D::make_gaussian_dist()
{
	assert(order_ == GaussianDistribution3D::Ordering::XYZ ||
		   order_ == GaussianDistribution3D::Ordering::ZXY ||
		   order_ == GaussianDistribution3D::Ordering::ZYX ||
		   "order_ must be one of {XYZ, ZYX, ZXY}\n");
	switch (order_) {
		case Ordering::XYZ:
			make_gaussian_dist_xyz();
			break;
		case Ordering::ZXY:
			make_gaussian_dist_zxy();
			break;
		case Ordering::ZYX:
			make_gaussian_dist_zyx();
			break;
	}
}

void GaussianDistribution3D::make_gaussian_dist_xyz()
{
	double sum = 0;
	const double sigma = 1.0;
	const double N = 1.0 / ((sigma * sigma * sigma) * std::pow(2 * M_PI, 3.0 / 2.0));
	for (int32_t x = -radius_; x <= radius_; ++x) {
		for (int32_t y = -radius_; y <= radius_; ++y) {
			for (int32_t z = -radius_; z <= radius_; ++z) {
				double val = std::exp(-(x * x + y * y + z * z));
				values_[x * sideLength_ * sideLength_ + y * sideLength_ + z] = val;
				sum += val;
			}
		}
	}

	// Normalize
	for (size_t i = 0; i < sideLength_ * sideLength_ * sideLength_; ++i) {
		values_[i] /= sum;
	}
}

void GaussianDistribution3D::make_gaussian_dist_zxy()
{
	double sum = 0;
	const double sigma = 1.0;
	const double N = 1.0 / ((sigma * sigma * sigma) * std::pow(2 * M_PI, 3.0 / 2.0));
	for (int32_t z = -radius_; z <= radius_; ++z) {
		for (int32_t x = -radius_; x <= radius_; ++x) {
			for (int32_t y = -radius_; y <= radius_; ++y) {
				double val = std::exp(-(x * x + y * y + z * z));
				values_[z * sideLength_ * sideLength_ + x * sideLength_ + y] = val;
				sum += val;
			}
		}
	}

	// Normalize
	for (size_t i = 0; i < sideLength_ * sideLength_ * sideLength_; ++i) {
		values_[i] /= sum;
	}
}

void GaussianDistribution3D::make_gaussian_dist_zyx()
{
	double sum = 0;
	const double sigma = 1.0;
	const double N = 1.0 / ((sigma * sigma * sigma) * std::pow(2 * M_PI, 3.0 / 2.0));
	for (int32_t z = -radius_; z <= radius_; ++z) {
		for (int32_t y = -radius_; y <= radius_; ++y) {
			for (int32_t x = -radius_; x <= radius_; ++x) {
				double val = std::exp(-(x * x + y * y + z * z));
				values_[z * sideLength_ * sideLength_ + y * sideLength_ + x] = val;
				sum += val;
			}
		}
	}

	// Normalize
	for (size_t i = 0; i < sideLength_ * sideLength_ * sideLength_; ++i) {
		values_[i] /= sum;
	}
}
