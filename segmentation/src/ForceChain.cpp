#include "vc/segmentation/stps/ForceChain.hpp"

using namespace volcart::segmentation;
namespace vcs = volcart::segmentation;

/// Force Chain ///
ForceChain& ForceChain::operator+=(const ForceChain& rhs)
{
    auto fIt = rhs.begin();
    std::for_each(data.begin(), data.end(), [&fIt](Force& f) { f += *fIt++; });

    return *this;
}

ForceChain& ForceChain::operator*=(const double& rhs)
{
    std::for_each(
        std::begin(data), std::end(data), [rhs](Force& p) { p *= rhs; });

    return *this;
}

ForceChain vcs::operator+(ForceChain lhs, const ForceChain& rhs)
{
    return lhs += rhs;
}

ForceChain vcs::operator*(ForceChain lhs, const double& rhs)
{
    return lhs *= rhs;
}

ForceChain vcs::operator*(const double& rhs, ForceChain lhs)
{
    return lhs *= rhs;
}

void ForceChain::normalize(ForceChain& c)
{
    std::for_each(c.begin(), c.end(), [](auto& f) { cv::normalize(f, f); });
}