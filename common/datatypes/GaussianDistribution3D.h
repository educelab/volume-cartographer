#pragma once

#ifndef _VOLCART_COMMON_GAUSSIANDISTRIBUTION3D_H_
#define _VOLCART_COMMON_GAUSSIANDISTRIBUTION3D_H_

#include <memory>

namespace volcart
{
class GaussianDistribution3D
{
public:
	enum class Ordering
	{
		XYZ, ZXY, ZYX
	};

	GaussianDistribution3D();

	GaussianDistribution3D(int32_t radius);

	GaussianDistribution3D(int32_t radius, Ordering order);

	const double& operator[](const size_t index) const
	{
		return values_[index];
	}
	
	double& operator[](const size_t index)
	{
		return values_[index];
	}

	const double& at(size_t x, size_t y, size_t z) const;

private:
	Ordering order_;
	int32_t radius_;
	int32_t sideLength_;
	std::unique_ptr<double[]> values_;

	void init();

	void make_gaussian_dist();

	void make_gaussian_dist_xyz();

	void make_gaussian_dist_zxy();

	void make_gaussian_dist_zyx();
};
}

#endif
