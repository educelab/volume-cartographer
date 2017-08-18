#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace volcart
{
namespace segmentation
{
using Force = cv::Vec3d;
class ForceChain
{
public:
    using Chain = std::vector<Force>;
    ForceChain() = default;
    ForceChain(const Chain& c) : data(c) {}

    ForceChain& operator+=(const ForceChain& rhs);
    ForceChain& operator*=(const double& rhs);
    auto operator[](size_t i) { return data[i]; }

    auto begin() { return data.begin(); }
    auto begin() const { return data.begin(); }
    auto end() { return data.end(); }
    auto end() const { return data.end(); }

    template <class... Args>
    void emplace_back(Args&&... args)
    {
        data.emplace_back(std::forward<Args>(args)...);
    }
    void push_back(const Force& val) { data.push_back(val); }

    size_t size() { return data.size(); }

    static void normalize(ForceChain& c);

    Chain data;
};

ForceChain operator+(ForceChain lhs, const ForceChain& rhs);
ForceChain operator*(ForceChain lhs, const double& rhs);
ForceChain operator*(const double& rhs, ForceChain lhs);
}
}
