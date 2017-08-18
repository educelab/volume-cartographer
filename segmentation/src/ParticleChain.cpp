#include "vc/segmentation/stps/ParticleChain.hpp"

using namespace volcart::segmentation;
namespace vcs = volcart::segmentation;

/// Particle Chain ////

ParticleChain& ParticleChain::operator+=(const ForceChain& rhs)
{
    auto fIt = rhs.begin();
    std::for_each(
        data.begin(), data.end(), [&fIt](Particle& p) { p += *fIt++; });

    return *this;
}

ParticleChain& ParticleChain::operator*=(const double& rhs)
{
    std::for_each(
        std::begin(data), std::end(data), [rhs](Particle& p) { p *= rhs; });

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