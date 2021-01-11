#pragma once

/** @file */

#include <opencv2/core.hpp>

#include "vc/segmentation/ChainSegmentationAlgorithmBaseClass.hpp"
#include "vc/segmentation/stps/Particle.hpp"
#include "vc/segmentation/stps/ParticleChain.hpp"

namespace volcart
{
namespace segmentation
{
/**
 * @class StructureTensorParticleSim
 * @brief Structure Tensor %Particle Simulation (STPS) segmentation
 *
 * This algorithm propagates a chain of points through a volume using an
 * estimate of the local neighborhood as a guide. Point movement is constrained
 * by way of a corrective spring force between each point.
 *
 * @ingroup stps
 */
class StructureTensorParticleSim : public ChainSegmentationAlgorithmBaseClass
{
public:
    /**@{*/
    /** @brief Default constructor */
    StructureTensorParticleSim() = default;
    /** Default destructor */
    ~StructureTensorParticleSim() override = default;
    /**@}*/

    /**@{*/
    /** @brief Set the propagation force scale factor */
    void setPropagationScaleFactor(double g) { propagationScaleFactor_ = g; }

    /**
     * @brief Set the estimated thickness of the substrate (in um)
     *
     * Used to generate the radius of the structure tensor calculation
     */
    void setMaterialThickness(double m) { materialThickness_ = m; }

    /**
     * @brief Set the Runge-Kutta step size
     *
     * Should be less than or equal to the value set by setStepSize(). Number of
     * RK iterations per output step is determined by `stepSize_ / rkStepSize_`.
     */
    void setRKStepSize(double s) { rkStepSize_ = s; }
    /**@}*/

    /**@{*/
    /** @brief Compute the segmentation */
    PointSet compute() override;
    /**@}*/

    /** Sends when the segmentation is updated with intermediate results */
    size_t progressIterations() const override;

private:
    /** Scale factor for propagation force */
    double propagationScaleFactor_{0.5};
    /** Spring constant for Hooke's law */
    double springConstantK_{-0.5};
    /** Estimated material thickness in um */
    double materialThickness_{100};
    /** Radius for structure tensor calculation kernel */
    int radius_{5};

    /** Most recent version of the chain */
    ParticleChain currentChain_;

    /**
     * Runge-Kutta step size. Number of RK iterations per step is determined by
     * `stepSize_ / rkStepSize_`
     */
    double rkStepSize_{0.5};

    /** Calculate the propagation force direction for each point in the chain */
    ForceChain calc_prop_forces_(ParticleChain c);

    /** Calculate the corrective spring force for each point in the chain */
    ForceChain calc_spring_forces_(ParticleChain c);

    /** Add the current chain to the final result point set */
    void add_chain_to_result_();

    /** Return whether or not any point in the chain is out of bounds */
    bool chain_stopped_();
};
}  // namespace segmentation
}  // namespace volcart