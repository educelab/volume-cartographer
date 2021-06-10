#include "vc/texturing/FlatteningError.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

#include "vc/core/types/Color.hpp"
#include "vc/core/util/ApplyLUT.hpp"
#include "vc/core/util/FloatComparison.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MeshMath.hpp"

using namespace volcart;
using namespace volcart::texturing;
namespace vct = volcart::texturing;

std::vector<cv::Vec3d> GetCellVertices(
    const ITKMesh::Pointer& mesh,
    const ITKCell::PointIdentifierContainerType& ids)
{

    // Output vector
    std::vector<cv::Vec3d> pts;

    // For each id, add a new vec3d
    for (const auto& id : ids) {
        auto p = mesh->GetPoint(id);
        pts.emplace_back(p[0], p[1], p[2]);
    }

    return pts;
}

static std::tuple<double, double, double, double, double> CalculateGammas(
    const cv::Vec3d& p1,
    const cv::Vec3d& p2,
    const cv::Vec3d& p3,
    const cv::Vec3d& q1,
    const cv::Vec3d& q2,
    const cv::Vec3d& q3)
{
    // Calculate 2D area
    auto a = cv::norm(p2 - p1);
    auto b = cv::norm(p3 - p1);
    auto c = cv::norm(p3 - p2);
    auto area = meshmath::TriangleArea(a, b, c);

    // Convenience handles to match the paper
    const auto& s1 = p1[0];
    const auto& t1 = p1[2];
    const auto& s2 = p2[0];
    const auto& t2 = p2[2];
    const auto& s3 = p3[0];
    const auto& t3 = p3[2];

    // δS/δs
    auto ds = (q1 * (t2 - t3) + q2 * (t3 - t1) + q3 * (t1 - t2)) / (2 * area);
    // δS/δt
    auto dt = (q1 * (s3 - s2) + q2 * (s1 - s3) + q3 * (s2 - s1)) / (2 * area);

    a = ds.dot(ds);
    b = ds.dot(dt);
    c = dt.dot(dt);

    // Calculate max = Γ, min = γ
    auto base = a + c;
    auto offset = std::sqrt(std::pow(a - c, 2) + 4 * std::pow(b, 2));
    auto max = std::sqrt(0.5 * (base + offset));
    auto min = std::sqrt(0.5 * (base - offset));

    return {max, min, a, b, c};
}

static std::pair<double, double> TriLStretch(
    const cv::Vec3d& p0,
    const cv::Vec3d& p1,
    const cv::Vec3d& p2,
    const cv::Vec3d& q0,
    const cv::Vec3d& q1,
    const cv::Vec3d& q2)
{
    const auto& [max, min, a, b, c] = CalculateGammas(p0, p1, p2, q0, q1, q2);
    return {std::sqrt(0.5 * (a + c)), max};
}

LStretchMetrics vct::LStretch(
    const ITKMesh::Pointer& mesh3D, const ITKMesh::Pointer& mesh2D)
{
    if (mesh3D->GetNumberOfCells() != mesh2D->GetNumberOfCells()) {
        throw std::runtime_error(
            "Original and flattened meshes have mismatched number of faces");
    }

    if (mesh3D->GetNumberOfPoints() != mesh2D->GetNumberOfPoints()) {
        throw std::runtime_error(
            "Original and flattened meshes have mismatched number of vertices");
    }

    // Iterate over the faces
    auto cell = mesh3D->GetCells()->Begin();
    auto end = mesh3D->GetCells()->End();

    // Calculate metrics
    LStretchMetrics metrics;
    double sumL2{0};
    double area3DTotal{0};
    for (; cell != end; ++cell) {
        // Get the vertices
        auto vIds = cell.Value()->GetPointIdsContainer();
        auto p = GetCellVertices(mesh2D, vIds);
        auto q = GetCellVertices(mesh3D, vIds);

        // Calculate LStretch(T) for this face
        const auto& [l2, lInf] =
            TriLStretch(p[0], p[1], p[2], q[0], q[1], q[2]);
        metrics.faceL2.push_back(l2);
        metrics.faceLInf.push_back(lInf);

        // Update global LInf
        metrics.lInf = std::max(metrics.lInf, lInf);

        // A'(T)
        auto a = cv::norm(q[1] - q[0]);
        auto b = cv::norm(q[2] - q[0]);
        auto c = cv::norm(q[2] - q[1]);
        auto area3D = meshmath::TriangleArea(a, b, c);

        // sum L2Stretch(T)^2 * A'(T)
        sumL2 += l2 * l2 * area3D;
        // sum A'(T)
        area3DTotal += area3D;
    }

    // Calculate global L2
    metrics.l2 = std::sqrt(sumL2 / area3DTotal);

    // Return L2Stretch
    return metrics;
}

LStretchMetrics vct::InvertLStretchMetrics(const LStretchMetrics& metrics)
{
    LStretchMetrics out;

    // Invert the global L2 metric
    out.l2 = 1.0 / metrics.l2;

    // Invert each per-face metric
    std::transform(
        metrics.faceL2.begin(), metrics.faceL2.end(),
        std::back_inserter(out.faceL2), [](auto& v) { return 1.0 / v; });
    std::transform(
        metrics.faceLInf.begin(), metrics.faceLInf.end(),
        std::back_inserter(out.faceLInf), [](auto& v) { return 1.0 / v; });

    // Get the global LInf metric
    out.lInf = *std::max_element(out.faceLInf.begin(), out.faceLInf.end());

    return out;
}

// Function for creating a plot legend bar for LStretch metrics
static cv::Mat CreateLegend(
    int width,
    float min,
    float max,
    const cv::Mat& lut,
    const std::string& label = "")
{
    // Output image
    int height{160};
    cv::Mat bar = cv::Mat::zeros(height, width, CV_8UC3);

    // params
    static auto font = cv::FONT_HERSHEY_SIMPLEX;
    static double scale{1};
    static int thickness{1};
    int elemPad{25};
    cv::String str;
    int x{0};
    int y{0};
    cv::Size size;

    // Plot label
    if (not label.empty()) {
        str = label;
        size = cv::getTextSize(str, font, scale * 2, thickness, nullptr);
        x = elemPad * 2;
        y = (height + size.height) / 2;
        cv::putText(
            bar, str, {x, y}, font, scale * 2, color::WHITE, thickness,
            cv::LINE_AA);
    }

    // Reset elem positions for R->L drawing
    x = width - 1;

    // Plot max label
    str = std::to_string(max);
    size = cv::getTextSize(str, font, scale, thickness, nullptr);
    x -= elemPad * 2 + size.width;
    y = (height + size.height) / 2;
    cv::putText(
        bar, str, {x, y}, font, scale, color::WHITE, thickness, cv::LINE_AA);

    // Plot scale bar
    auto scaleBar = GenerateLUTScaleBar(lut, false, 64, 500);
    x -= elemPad + scaleBar.cols;
    y = (height - scaleBar.rows) / 2;
    // Plot a line for 1 if it's not at the end of the scale bar
    if (not AlmostEqual(min, 1.F) and not AlmostEqual(max, 1.F)) {
        cv::line(scaleBar, {250, 0}, {250, 63}, color::WHITE);
        str = "1.0";
        size = cv::getTextSize(str, font, scale, thickness, nullptr);
        auto lx = x + 250 - size.width / 2;
        auto ly = static_cast<int>(std::floor(y - elemPad * 0.75));
        cv::putText(
            bar, str, {lx, ly}, font, scale, color::WHITE, thickness,
            cv::LINE_AA);
    }
    cv::Rect roi{x, y, scaleBar.cols, scaleBar.rows};
    scaleBar.copyTo(bar(roi));

    // Plot min label
    str = std::to_string(min);
    size = cv::getTextSize(str, font, scale, thickness, nullptr);
    x -= elemPad + size.width;
    y = (height + size.height) / 2;
    cv::putText(
        bar, str, {x, y}, font, scale, color::WHITE, thickness, cv::LINE_AA);

    return bar;
}

std::vector<cv::Mat> vct::PlotLStretchError(
    const LStretchMetrics& metrics,
    const cv::Mat& cellMap,
    ColorMap cm,
    bool drawLegend)
{
    // Get easy handles to the metrics
    const auto& faceL2 = metrics.faceL2;
    const auto& faceLInf = metrics.faceLInf;

    // Plot raw L stretch
    auto l2Plot = PlotPerFaceError(cellMap, faceL2, 1);
    auto lInfPlot = PlotPerFaceError(cellMap, faceLInf, 1);

    // Generate mask
    cv::Mat mask;
    cellMap.convertTo(mask, CV_32F);
    cv::threshold(mask, mask, -1, 255, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8U);

    // Apply LUT
    auto lut = GetColorMapLUT(cm);
    auto minMax = std::minmax_element(faceL2.begin(), faceL2.end());
    auto l2Min = static_cast<float>(*minMax.first);
    auto l2Max = static_cast<float>(*minMax.second);
    if (AlmostEqual(l2Min, 1.F) or AlmostEqual(l2Max, 1.F)) {
        l2Plot = ApplyLUT(l2Plot, lut, l2Min, l2Max);
    } else {
        l2Plot = ApplyLUT(l2Plot, lut, l2Min, 1.F, l2Max);
    }

    auto lInfMin =
        static_cast<float>(*std::min_element(faceLInf.begin(), faceLInf.end()));
    auto lInfMax = static_cast<float>(metrics.lInf);
    if (AlmostEqual(lInfMin, 1.F) or AlmostEqual(lInfMax, 1.F)) {
        lInfPlot = ApplyLUT(lInfPlot, lut, lInfMin, lInfMax);
    } else {
        lInfPlot = ApplyLUT(lInfPlot, lut, lInfMin, 1.F, lInfMax);
    }

    // Apply mask to image
    cv::Mat l2PlotTmp;
    cv::Mat lInfPlotTmp;
    l2Plot.copyTo(l2PlotTmp, mask);
    lInfPlot.copyTo(lInfPlotTmp, mask);

    // Add legend bar if requested
    if (drawLegend) {
        auto title = "L2 Stretch (Global: " + std::to_string(metrics.l2) + ")";
        auto legend = CreateLegend(l2Plot.cols, l2Min, l2Max, lut, title);
        l2Plot = cv::Mat::zeros(
            l2PlotTmp.rows + legend.rows, l2PlotTmp.cols, CV_8UC3);
        l2PlotTmp.copyTo(l2Plot({0, 0, l2PlotTmp.cols, l2PlotTmp.rows}));
        legend.copyTo(l2Plot({0, l2PlotTmp.rows, legend.cols, legend.rows}));

        title = "LInf Stretch (Global: " + std::to_string(metrics.lInf) + ")";
        legend = CreateLegend(lInfPlot.cols, lInfMin, lInfMax, lut, title);
        lInfPlot = cv::Mat::zeros(
            lInfPlotTmp.rows + legend.rows, lInfPlotTmp.cols, CV_8UC3);
        lInfPlotTmp.copyTo(
            lInfPlot({0, 0, lInfPlotTmp.cols, lInfPlotTmp.rows}));
        legend.copyTo(
            lInfPlot(cv::Rect{0, lInfPlotTmp.rows, legend.cols, legend.rows}));
    } else {
        l2Plot = l2PlotTmp;
        lInfPlot = lInfPlotTmp;
    }

    return {l2Plot, lInfPlot};
}
