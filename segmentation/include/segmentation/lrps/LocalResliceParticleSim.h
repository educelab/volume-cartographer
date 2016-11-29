#pragma once

#include <iostream>

#include "core/types/OrderedPointSet.h"
#include "core/types/VolumePkg.h"
#include "segmentation/lrps/Common.h"
#include "segmentation/lrps/FittedCurve.h"

namespace volcart
{
namespace segmentation
{
/**
 * @class LocalResliceSegmentation
 * @brief Used to perform segmentation using the Local Reslice approach
 *
 * This algorithm performs segmentation by first making sure that all of
 * the points are evenly spaced. It then generates candidate positions for
 * the particles. It then uses the top maxima to generate an initial guess
 * for the particle position and clamps the points that moved too far
 * back. It then visualizes it if the user requests this, for example
 * if this is being done in a GUI. It then repeats this process
 * for all of the slices requested.
 *
 * @ingroup lrps
 */
class LocalResliceSegmentation
{
public:
    /** @brief Initializes and sets the volume package where the segmentation is
     * located */
    LocalResliceSegmentation(VolumePkg& pkg) : pkg_(pkg) {}

    /**
     * @brief Performs the actual segmentation
     * @param cloud PointSet that you want to segment
     * @param startIndex Slice to start segmentation on
     * @param endIndex Last slice to be segmented
     * @param numIters Number of interations to be performed for each slice
     * @param stepNumLayers Number of layers to move each iteration, NOTE TO
     * SETH: This doesn't match the CPP
     * @param alpha Used to calculate energy
     * @param k1 Used to calculate energy
     * @param k2 Used to calculate energy
     * @param beta Used to calculate energy
     * @param delta Used to calculate energy
     * @param peakDistanceWeight Used to generate the intensity map
     * @param shouldIncludeMiddle If you want to include middle points
     * @param dumpVis Used for debug, Outputs the whole chain to a file
     * @param visualize Used for GUI, Draws a curve in a window
     * @return The segmented path
     */
    volcart::OrderedPointSet<cv::Vec3d> segmentPath(
        std::vector<cv::Vec3d> cloud,
        int32_t startIndex,
        int32_t endIndex,
        int32_t numIters,
        int32_t stepNumLayers,
        double alpha,
        double k1,
        double k2,
        double beta,
        double delta,
        int32_t peakDistanceWeight,
        bool shouldIncludeMiddle,
        bool dumpVis,
        bool visualize);

private:
    /** Where the segmentation is stored */
    VolumePkg& pkg_;

    /** @brief Estimates the normals to the Voxels at the current slice  */
    cv::Vec3d estimateNormalAtIndex(const FittedCurve& curve, int32_t index);

    /** @brief Used for visualization, saves the points to a Mat that can be
     * displayed */
    cv::Mat drawParticlesOnSlice(
        const FittedCurve& curve,
        int32_t sliceIndex,
        int32_t particleIndex = -1,
        bool showSpline = false) const;

    /** Default minimum energy gradient */
    constexpr static double kDefaultMinEnergyGradient = 1e-7;
};
}
}
