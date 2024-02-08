#include "vc/segmentation/stps/ParticleChain.hpp"

#include <exception>

using namespace volcart::segmentation;
namespace vcs = volcart::segmentation;

auto ParticleChain::operator+=(const ForceChain& rhs) -> ParticleChain&
{
    if (data_.size() != rhs.size()) {
        throw std::domain_error("Vector sizes don't match");
    }

    auto fIt = rhs.begin();
    std::for_each(
        data_.begin(), data_.end(), [&fIt](Particle& p) { p += *fIt++; });

    return *this;
}

auto ParticleChain::operator*=(const double& rhs) -> ParticleChain&
{
    std::for_each(
        std::begin(data_), std::end(data_), [rhs](Particle& p) { p *= rhs; });

    return *this;
}

auto vcs::operator+(ParticleChain lhs, const ForceChain& rhs) -> ParticleChain
{
    return lhs += rhs;
}

auto vcs::operator+(const ForceChain& rhs, ParticleChain lhs) -> ParticleChain
{
    return lhs += rhs;
}

auto vcs::operator*(ParticleChain lhs, const double& rhs) -> ParticleChain
{
    return lhs *= rhs;
}

auto vcs::operator*(const double& rhs, ParticleChain lhs) -> ParticleChain
{
    return lhs *= rhs;
}