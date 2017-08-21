#include "vc/segmentation/stps/Particle.hpp"

using namespace volcart::segmentation;
namespace vcs = volcart::segmentation;

Particle& Particle::operator+=(const cv::Vec3d& rhs)
{
    pos_ += rhs;
    return *this;
}

Particle& Particle::operator*=(const double& rhs)
{
    pos_ *= rhs;
    return *this;
}

Particle vcs::operator+(Particle lhs, const cv::Vec3d& rhs)
{
    return lhs += rhs;
}

Particle vcs::operator*(Particle lhs, const double& rhs) { return lhs *= rhs; }