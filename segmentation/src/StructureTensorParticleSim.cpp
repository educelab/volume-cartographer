#include "vc/segmentation/StructureTensorParticleSim.hpp"

#include "vc/core/math/StructureTensor.hpp"

namespace vc = volcart;
using namespace vc::segmentation;

static constexpr double RK_STEP_SCALE = 1.0 / 6.0;

StructureTensorParticleSim::PointSet StructureTensorParticleSim::compute()
{
    // Convert the starting chain into a particle chain
    currentChain_.clear();
    for (const auto& s : startingChain_) {
        currentChain_.emplace_back(s);
    }

    // Calculate the resting lengths
    for (auto it = currentChain_.begin(); it != currentChain_.end(); it++) {
        // Calculate left resting
        if (it != currentChain_.begin()) {
            it->restingL() = cv::norm(it->pos() - std::prev(it)->pos());
        }

        // Calculate right resting
        if (it != currentChain_.end() - 1) {
            it->restingR() = cv::norm(std::next(it)->pos() - it->pos());
        }
    }

    // Reset the result vector and add the starting chain to it
    result_.clear();
    result_.setWidth(currentChain_.size());

    // Other params
    radius_ = static_cast<int>(
        std::ceil(materialThickness_ / vol_->voxelSize()) * 0.5);

    // Output iterations
    auto outIters = static_cast<size_t>(std::ceil(numSteps_ / stepSize_));
    // Runge-Kutta iterations
    auto rkIters = static_cast<size_t>(std::ceil(stepSize_ / rkStepSize_));

    // Sampled output iterations
    for (size_t it = 0; it < outIters; it++) {
        // Update progress
        progressUpdated(float(it) / float(outIters));

        // Run Runge-Kutta multiple times to accumulate one full output step
        for (size_t rkIt = 0; rkIt < rkIters; rkIt++) {
            // K1
            auto chainK1 = currentChain_;
            auto k1 = calc_prop_forces_(chainK1) + calc_spring_forces_(chainK1);
            ForceChain::Normalize(k1);
            // K2
            auto chainK2 = currentChain_ + (rkStepSize_ * 0.5 * k1);
            auto k2 = calc_prop_forces_(chainK2) + calc_spring_forces_(chainK2);
            ForceChain::Normalize(k2);
            // K3
            auto chainK3 = currentChain_ + (rkStepSize_ * 0.5 * k2);
            auto k3 = calc_prop_forces_(chainK3) + calc_spring_forces_(chainK3);
            ForceChain::Normalize(k3);
            // K4
            auto chainK4 = currentChain_ + (rkStepSize_ * k3);
            auto k4 = calc_prop_forces_(chainK4) + calc_spring_forces_(chainK4);
            ForceChain::Normalize(k4);

            currentChain_ +=
                (rkStepSize_ * RK_STEP_SCALE * (k1 + (2 * k2) + (2 * k3) + k4));
        }

        if (chain_stopped_()) {
            status_ = Status::ReturnedEarly;
            break;
        }

        add_chain_to_result_();
    }
    // Update progress
    progressUpdated(1.0);

    return result_;
}

bool StructureTensorParticleSim::chain_stopped_()
{
    return std::any_of(
        std::begin(currentChain_), std::end(currentChain_), [this](auto v) {
            return !bb_.isInBounds(v.pos()) || !vol_->isInBounds(v.pos());
        });
}

void StructureTensorParticleSim::add_chain_to_result_()
{
    std::vector<cv::Vec3d> row;
    for (const auto& p : currentChain_) {
        row.emplace_back(p.pos());
    }

    result_.pushRow(row);
}

ForceChain StructureTensorParticleSim::calc_prop_forces_(ParticleChain c)
{
    ForceChain res;
    Force zDir{0, 0, 1};

    for (const auto& p : c) {
        auto ep = ComputeSubvoxelEigenPairs(vol_, p.pos(), radius_);
        auto offset = ep[0].second;
        offset = zDir - (zDir.dot(offset)) / (offset.dot(offset)) * offset;
        cv::normalize(offset, offset);
        res.push_back(offset * propagationScaleFactor_);
    }

    return res;
}

ForceChain StructureTensorParticleSim::calc_spring_forces_(ParticleChain c)
{
    ForceChain res;
    for (auto it = c.begin(); it != c.end(); it++) {
        // Setup an empty force vector
        Force f{0, 0, 0};

        // Calculate left spring
        if (it != c.begin()) {
            auto vec = it->pos() - std::prev(it)->pos();
            auto dist = cv::norm(vec);
            cv::normalize(vec, vec, springConstantK_ * (dist - it->restingL()));
            f += vec;
        }

        // Calculate right resting
        if (it != c.end() - 1) {
            auto vec = std::next(it)->pos() - it->pos();
            auto dist = cv::norm(vec);
            cv::normalize(vec, vec, springConstantK_ * (dist - it->restingR()));
            f += vec;
        }

        res.push_back(f);
    }

    return res;
}