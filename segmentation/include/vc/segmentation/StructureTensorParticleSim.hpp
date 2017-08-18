#pragma once

#include <opencv2/core.hpp>

#include "vc/segmentation/ChainSegmentationAlgorithmBaseClass.hpp"
#include "vc/segmentation/stps/Particle.hpp"
#include "vc/segmentation/stps/ParticleChain.hpp"

namespace volcart
{
namespace segmentation
{
class StructureTensorParticleSim : public ChainSegmentationAlgorithmBaseClass
{
public:
    StructureTensorParticleSim() = default;

    void setTargetZIndex(int z) { endIndex_ = z; }

    void setPropagationScaleFactor(double g) { propagationScaleFactor_ = g; }
    void setMaterialThickness(double m) { materialThickness_ = m; }

    PointSet compute() override;

private:
    int endIndex_{0};
    double propagationScaleFactor_{0.5};
    double springConstantK_{-0.5};
    double materialThickness_{100};
    int radius_{5};

    ParticleChain currentChain_;

    double rkStepSize_{0.5};

    void add_chain_to_result_();
    bool chain_stopped_();
    ForceChain calc_prop_forces_(ParticleChain c);
    ForceChain calc_spring_forces_(ParticleChain c);
};
}
}