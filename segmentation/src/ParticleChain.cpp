#include "vc/segmentation/stps/ParticleChain.hpp"

#include <exception>

using namespace volcart::segmentation;
namespace vcs = volcart::segmentation;

ParticleChain& ParticleChain::operator+=(const ForceChain& rhs)
{
    if (data_.size() != rhs.size()) {
        throw std::domain_error("Vector sizes don't match");
    }

    auto fIt = rhs.begin();
    std::for_each(
        data_.begin(), data_.end(), [&fIt](Particle& p) { p += *fIt++; });

    return *this;
}

ParticleChain& ParticleChain::operator*=(const double& rhs)
{
    std::for_each(
        std::begin(data_), std::end(data_), [rhs](Particle& p) { p *= rhs; });

    return *this;
}

ParticleChain vcs::operator+(ParticleChain lhs, const ForceChain& rhs)
{
    return lhs += rhs;
}

ParticleChain vcs::operator+(const ForceChain& rhs, ParticleChain lhs)
{
    return lhs += rhs;
}

ParticleChain vcs::operator*(ParticleChain lhs, const double& rhs)
{
    return lhs *= rhs;
}

ParticleChain vcs::operator*(const double& rhs, ParticleChain lhs)
{
    return lhs *= rhs;
}