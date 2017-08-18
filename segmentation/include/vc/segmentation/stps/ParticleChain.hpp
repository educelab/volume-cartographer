#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "vc/segmentation/stps/ForceChain.hpp"
#include "vc/segmentation/stps/Particle.hpp"

namespace volcart
{
namespace segmentation
{
class ParticleChain
{
public:
    using Chain = std::vector<Particle>;
    ParticleChain() = default;
    ParticleChain(const Chain& c) : data(c) {}

    ParticleChain& operator+=(const ForceChain& rhs);
    ParticleChain& operator*=(const double& rhs);
    auto operator[](size_t i) { return data[i]; }

    auto begin() { return data.begin(); }
    auto end() { return data.end(); }

    template <class... Args>
    void emplace_back(Args&&... args)
    {
        data.emplace_back(std::forward<Args>(args)...);
    }
    void push_back(const Particle& val) { data.push_back(val); }

    size_t size() { return data.size(); }

    Chain data;
};

ParticleChain operator+(ParticleChain lhs, const ForceChain& rhs);
ParticleChain operator+(const ForceChain& rhs, ParticleChain lhs);
ParticleChain operator*(ParticleChain lhs, const double& rhs);
ParticleChain operator*(const double& rhs, ParticleChain lhs);
}
}
