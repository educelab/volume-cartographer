#pragma once

#ifndef _VOLCART_COMMON_GAUSSIANDISTRIBUTION3D_H_
#define _VOLCART_COMMON_GAUSSIANDISTRIBUTION3D_H_

#include <memory>
#include <iostream>

namespace volcart
{
class GaussianDistribution3D
{
public:
	enum class Ordering
	{
		XYZ, ZXY, ZYX
	};

	GaussianDistribution3D() :
        order_(Ordering::ZYX), radius_(1)
    {
        init();
    }

	GaussianDistribution3D(int32_t radius) :
        order_(Ordering::ZYX), radius_(radius)
    {
        init();
    }

	GaussianDistribution3D(int32_t radius, Ordering order) :
        order_(order), radius_(radius)
    {
        init();
    }

	const double& operator[](const size_t index) const
	{
		return values_[index];
	}
	
	const double& at(size_t x, size_t y, size_t z) const;

    size_t size() const
    {
        return size_;
    }

    friend std::ostream& operator<<(std::ostream& s, const GaussianDistribution3D& d)
    {
        for (int32_t i = 0; i < d.sideLength_; ++i) {
            for (int32_t j = 0; j < d.sideLength_; ++j) {
                for (int32_t k = 0; k < d.sideLength_; ++k) {
                    s << d[i * d.sideLength_ * d.sideLength_ + j * d.sideLength_ + k] << " ";
                }
                s << std::endl;
            }
            s << std::endl;
        }
        return s;
    }

private:
	Ordering order_;
	int32_t radius_;
	int32_t sideLength_;
    std::unique_ptr<double[]> values_;
    size_t size_;

	void init();

	void make_gaussian_dist();

	void make_gaussian_dist_xyz();

	void make_gaussian_dist_zxy();

	void make_gaussian_dist_zyx();
};
}

#endif
