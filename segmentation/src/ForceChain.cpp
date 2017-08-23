#include "vc/segmentation/stps/ForceChain.hpp"

using namespace volcart::segmentation;
namespace vcs = volcart::segmentation;

ForceChain& ForceChain::operator+=(const ForceChain& rhs)
{
    if (data_.size() != rhs.size()) {
        throw std::domain_error("Vector sizes don't match");
    }

    auto fIt = rhs.begin();
    std::for_each(
        data_.begin(), data_.end(), [&fIt](Force& f) { f += *fIt++; });

    return *this;
}

ForceChain& ForceChain::operator*=(const double& rhs)
{
    std::for_each(
        std::begin(data_), std::end(data_), [rhs](Force& p) { p *= rhs; });

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

void ForceChain::Normalize(ForceChain& c, double alpha)
{
    std::for_each(
        c.begin(), c.end(), [alpha](auto& f) { cv::normalize(f, f, alpha); });
}