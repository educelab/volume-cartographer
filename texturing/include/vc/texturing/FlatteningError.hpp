#pragma once

/** @file */

#include <opencv2/core.hpp>

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/util/ColorMaps.hpp"
#include "vc/core/util/Iteration.hpp"

namespace volcart::texturing
{

/**
 * @brief Plot generic per-face error information into an image
 * @tparam ErrContainer Container type supporting the .at() accessor. Return
 * values must support static_cast to float.
 * @param cellMap Cell map image produced by PPMGenerator or GenerateCellMap
 * @param errorMap Container of per-face error values
 * @param defaultValue Default pixel value if it doesn't have a cell mapping
 * @return Image of error plot with type `CV_32FC1`
 *
 * @ingroup UV Parameterization
 */
template <typename ErrContainer>
cv::Mat PlotPerFaceError(
    const cv::Mat& cellMap,
    const ErrContainer& errorMap,
    float defaultValue = 0)
{
    cv::Mat output(cellMap.rows, cellMap.cols, CV_32FC1);
    output = cv::Scalar::all(defaultValue);
    for (const auto& [y, x] : range2D(cellMap.rows, cellMap.cols)) {
        auto cell = cellMap.at<int32_t>(y, x);
        if (cell >= 0) {
            auto error = errorMap.at(cell);
            output.at<float>(y, x) = static_cast<float>(error);
        }
    }
    return output;
}

/**
 * @brief L stretch error metric output struct
 *
 * Holds global and per-face L2 and LInf flattening error metrics by Sander
 * et al. \cite sander2001texture. Per-face L2 represents the RMS stretch across
 * all directions of a 3D triangle when mapping a unit length vector from the 2D
 * domain. Per-face LInf represents the maximum stretch when mapping the same.
 * The global L2 metric summarizes the L2 stretch across all triangles and
 * provides an at-a-glance indicator of overall flattening error. The global
 * LInf metric is the maximum LInf value across all faces and represents the
 * worst case stretch across the entire surface. From Sheffer, Praun, and Rose
 * \cite sheffer2007mesh :
 *
 * > The name for the stretch metric comes from applications that map signals
 * > with regular sampling in the domain to 3D surfaces; these applications
 * > want to minimize the stretch of the signal over the surface, or the space
 * > between the locations of mapped samples.
 *
 * @note The L2 and LInf stretch metrics represent the stretch **from the 2D
 * domain to the 3D domain**. Thus, an LInf value of 0.5 indicates a flattening
 * where a 3D unit vector is **half** the length of its corresponding 2D unit
 * vector. This is the inverse of how stretch is generally considered in most
 * of Volume Cartographer, where we are interested in the stretch introduced
 * relative to the 3D domain. This function returns the L stretch metrics as
 * defined in the original paper, but VC applications should report the inverted
 * values indicating stretch from 3D-to-2D. These values can be calculated using
 * InvertLStretchMetrics.
 *
 * @ingroup UV Parameterization
 */
struct LStretchMetrics {
    /** @brief Global L2 error */
    double l2{0};
    /** @brief Global LInf error */
    double lInf{0};
    /** @brief Per-face L2 error */
    std::vector<double> faceL2;
    /** @brief Per-face LInf error */
    std::vector<double> faceLInf;
};

/**
 * @brief Calculate the L2 and LInf stretch between a 2D and 3D mesh
 *
 * Calculates the L2 and LInf stretch norms from "Texture Mapping Progressive
 * Meshes" by Sander et al. \cite sander2001texture. See LStretchMetrics for
 * more info on these metrics. Meshes are assumed to be pre-scaled to have the
 * same surface area.
 *
 * @ingroup UV Parameterization
 */
LStretchMetrics LStretch(
    const ITKMesh::Pointer& mesh3D, const ITKMesh::Pointer& mesh2D);

/**
 * @brief Calculates the inverse LStretchMetrics plotting error relative to the
 * 3D mesh
 *
 * The LStretch functions calculates the L stretch metrics from 2D-to-3D, but
 * this library is generally interested in the stretch from 3D-to-2D. This
 * function inverts the L stretch metrics as follows:
 * - L2 = 1 / L2
 * - LInf = max(1 / {Per-face LInf})
 * - {Per-face L2} = 1 / {Per-face L2}
 * - {Per-face LInf} = 1 / {Per-face LInf}
 */
LStretchMetrics InvertLStretchMetrics(const LStretchMetrics& metrics);

/**
 * @brief Plot per-face L stretch error metrics
 *
 * This function maps per-face LStretchMetrics to a 2D image using the provided
 * cell map. If drawLegend is true, a legend bar will be appended to the bottom
 * of the image showing the metric label, global metric, and color map scale
 * bar.
 *
 * @ingroup UV Parameterization
 */
std::vector<cv::Mat> PlotLStretchError(
    const LStretchMetrics& metrics,
    const cv::Mat& cellMap,
    ColorMap cm = ColorMap::Plasma,
    bool drawLegend = false);

}  // namespace volcart::texturing
