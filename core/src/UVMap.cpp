#include "vc/core/types/UVMap.hpp"

#include "vc/core/util/Iteration.hpp"

/** Top-left UV Origin */
const static cv::Vec2d ORIGIN_TOP_LEFT(0, 0);
/** Top-right UV Origin */
const static cv::Vec2d ORIGIN_TOP_RIGHT(1, 0);
/** Bottom-left UV Origin */
const static cv::Vec2d ORIGIN_BOTTOM_LEFT(0, 1);
/** Bottom-right UV Origin */
const static cv::Vec2d ORIGIN_BOTTOM_RIGHT(1, 1);

/** Minimum size of a UVMap debug image */
constexpr static int MIN_DEBUG_WIDTH = 50;

using namespace volcart;

inline auto OriginVector(const UVMap::Origin& o) -> cv::Vec2d;

void UVMap::set(size_t id, const cv::Vec2d& uv, const Origin& o)
{
    // transform to be relative to top-left
    cv::Vec2d transformed;
    cv::absdiff(uv, OriginVector(o), transformed);
    map_[id] = transformed;
}

void UVMap::set(size_t id, const cv::Vec2d& uv) { set(id, uv, origin_); }

auto UVMap::get(size_t id, const Origin& o) const -> cv::Vec2d
{
    auto it = map_.find(id);
    if (it != map_.end()) {
        // transform to be relative to the provided origin
        cv::Vec2d transformed;
        cv::absdiff(it->second, OriginVector(o), transformed);
        return transformed;
    } else {
        return NULL_MAPPING;
    }
}

auto UVMap::get(size_t id) const -> cv::Vec2d { return get(id, origin_); }

auto UVMap::contains(std::size_t id) const -> bool
{
    return map_.count(id) > 0;
}

UVMap::UVMap(UVMap::Origin o) : origin_{o} {}

auto UVMap::size() const -> size_t { return map_.size(); }

auto UVMap::empty() const -> bool { return map_.empty(); }

void UVMap::setOrigin(const UVMap::Origin& o) { origin_ = o; }

auto UVMap::origin() const -> UVMap::Origin { return origin_; }

auto UVMap::ratio() const -> UVMap::Ratio { return ratio_; }

void UVMap::ratio(double a) { ratio_.aspect = a; }

void UVMap::ratio(double w, double h)
{
    ratio_.width = w;
    ratio_.height = h;
    ratio_.aspect = w / h;
}
auto UVMap::as_map() const -> std::map<size_t, cv::Vec2d> { return map_; }

auto OriginVector(const UVMap::Origin& o) -> cv::Vec2d
{
    switch (o) {
        case UVMap::Origin::TopLeft:
            return ORIGIN_TOP_LEFT;
        case UVMap::Origin::TopRight:
            return ORIGIN_TOP_RIGHT;
        case UVMap::Origin::BottomLeft:
            return ORIGIN_BOTTOM_LEFT;
        case UVMap::Origin::BottomRight:
            return ORIGIN_BOTTOM_RIGHT;
    }
}

auto UVMap::Plot(const UVMap& uv, const Color& color) -> cv::Mat
{
    auto w = static_cast<int>(std::ceil(uv.ratio_.width));
    if (w < MIN_DEBUG_WIDTH) {
        w = MIN_DEBUG_WIDTH;
    }
    auto h = static_cast<int>(std::ceil(w / uv.ratio_.aspect));
    cv::Mat r = cv::Mat::zeros(h, w, CV_8UC3);

    for (auto it : uv.map_) {
        cv::Point2d p(it.second[0] * w, it.second[1] * h);
        cv::circle(r, p, 1, color, -1);
    }

    return r;
}

auto UVMap::Plot(
    const UVMap& uv,
    const ITKMesh::Pointer& mesh2D,
    int width,
    int height,
    const Color& color) -> cv::Mat
{
    if (width < 0) {
        width = std::max(
            static_cast<int>(std::ceil(uv.ratio_.width)), MIN_DEBUG_WIDTH);
    }

    if (height < 0) {
        height = static_cast<int>(std::ceil(width / uv.ratio_.aspect));
    }
    cv::Mat r = cv::Mat::zeros(height, width, CV_8UC3);

    auto c = mesh2D->GetCells()->Begin();
    auto end = mesh2D->GetCells()->End();
    for (; c != end; ++c) {
        auto vIds = c.Value()->GetPointIdsContainer();
        for (const auto& [idx, vID] : enumerate(vIds)) {
            auto a = uv.get(vID);
            auto b = (idx + 1 == vIds.size()) ? uv.get(vIds[0])
                                              : uv.get(vIds[idx + 1]);

            cv::Point2d A{a[0] * (width - 1), a[1] * (height - 1)};
            cv::Point2d B{b[0] * (width - 1), b[1] * (height - 1)};
            cv::line(r, A, B, color, 1, cv::LINE_AA);
        }
    }

    return r;
}

void UVMap::Rotate(UVMap& uv, double theta, const cv::Vec2d& center)
{
    // Setup pts matrix
    cv::Mat pts = cv::Mat::zeros(uv.map_.size(), 3, CV_64F);
    int row = 0;
    for (const auto& p : uv.map_) {
        // transform so that operation happens relative to stored origin
        cv::Vec2d transformed;
        cv::absdiff(p.second, OriginVector(uv.origin_), transformed);

        // to do, rotate relative to stored origin
        pts.at<double>(row, 0) = transformed[0];
        pts.at<double>(row, 1) = transformed[1];
        row++;
    }

    // Get translation matrix
    cv::Mat transMat = cv::Mat::eye(3, 3, CV_64F);
    transMat.at<double>(0, 2) = -center[0];
    transMat.at<double>(1, 2) = -center[1];

    // Get scale matrix
    cv::Mat scaleMat = cv::Mat::eye(3, 3, CV_64F);
    scaleMat.at<double>(0, 0) = uv.ratio_.width;
    scaleMat.at<double>(1, 1) = uv.ratio_.height;

    // Get counter-clockwise rotation matrix
    auto cos = std::cos(theta);
    auto sin = std::sin(theta);
    cv::Mat rotMat = cv::Mat::eye(3, 3, CV_64F);
    rotMat.at<double>(0, 0) = cos;
    rotMat.at<double>(0, 1) = sin;
    rotMat.at<double>(1, 0) = -sin;
    rotMat.at<double>(1, 1) = cos;

    // Composite transform matrix
    cv::Mat composite = rotMat * scaleMat * transMat;

    // Apply the transform to the col-major pts matrix
    pts = composite * pts.t();

    // Transpose back to row major
    pts = pts.t();

    // Get new min-max u & v
    double uMin, uMax;
    double vMin, vMax;
    cv::minMaxLoc(pts.col(0), &uMin, &uMax);
    cv::minMaxLoc(pts.col(1), &vMin, &vMax);

    // Set new width and height
    auto aspectWidth = std::abs(uMax - uMin);
    auto aspectHeight = std::abs(vMax - vMin);
    uv.ratio(aspectWidth, aspectHeight);

    // Update UVs within new bounds
    cv::Vec2d newPos;
    row = 0;
    for (auto& p : uv.map_) {
        // rescale within bounds
        newPos[0] = (pts.at<double>(row, 0) - uMin) / (uMax - uMin);
        newPos[1] = (pts.at<double>(row, 1) - vMin) / (vMax - vMin);

        // transform back to storage origin
        cv::Vec2d transformed;
        cv::absdiff(newPos, OriginVector(uv.origin_), transformed);

        // store
        p.second[0] = transformed[0];
        p.second[1] = transformed[1];

        // Advance the row counter
        row++;
    }
}

void UVMap::Flip(UVMap& uv, FlipAxis axis)
{
    for (auto& p : uv.map_) {
        switch (axis) {
            case FlipAxis::Horizontal:
                p.second[0] = 1 - p.second[0];
                continue;
            case FlipAxis::Vertical:
                p.second[1] = 1 - p.second[1];
                continue;
            case FlipAxis::Both:
                p.second = cv::Vec2d{1, 1} - p.second;
                continue;
        }
    }
}
