#include "vc/segmentation/stps/Particle.hpp"

using namespace volcart::segmentation;
namespace vcs = volcart::segmentation;

auto Particle::operator+=(const cv::Vec3d& rhs) -> Particle&
{
    pos_ += rhs;
    return *this;
}

auto Particle::operator*=(const double& rhs) -> Particle&
{
    pos_ *= rhs;
    return *this;
}

auto vcs::operator+(Particle lhs, const cv::Vec3d& rhs) -> Particle
{
    return lhs += rhs;
}

auto vcs::operator*(Particle lhs, const double& rhs) -> Particle
{
    return lhs *= rhs;
}