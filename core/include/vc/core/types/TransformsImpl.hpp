#pragma once

/** DEBUG ONLY: Print AffineTransform to std::ostream */
auto operator<<(std::ostream& os, const volcart::AffineTransform& t)
    -> std::ostream&;

namespace volcart
{

template <class PointSetT>
auto ApplyTransform(const PointSetT& ps, const Transform3D::Pointer& transform)
    -> PointSetT
{
    PointSetT output(ps);
    std::transform(
        output.begin(), output.end(), output.begin(),
        [transform](const auto& a) { return transform->applyPoint(a); });
    return output;
}

}  // namespace volcart