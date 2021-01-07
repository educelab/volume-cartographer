#include "vc/core/types/VolumetricMask.hpp"

using namespace volcart;

void VolumetricMask::setIn(const Voxel& v) { mask_.insert(v); }

void VolumetricMask::setOut(const Voxel& v) { mask_.erase(v); }

bool VolumetricMask::isIn(const Voxel& v) const { return mask_.count(v) > 0; }

bool VolumetricMask::isOut(const Voxel& v) const { return not isIn(v); }

bool VolumetricMask::isIn(const cv::Vec3d& v) const
{
    auto x = static_cast<int>(std::floor(v[0]));
    auto y = static_cast<int>(std::floor(v[1]));
    auto z = static_cast<int>(std::floor(v[2]));
    return isIn(Voxel{x, y, z});
}

bool VolumetricMask::isOut(const cv::Vec3d& v) const { return not isIn(v); }

VolumetricMask::iterator VolumetricMask::begin() noexcept
{
    return mask_.begin();
}

VolumetricMask::const_iterator VolumetricMask::begin() const noexcept
{
    return mask_.begin();
}

VolumetricMask::const_iterator VolumetricMask::cbegin() const noexcept
{
    return mask_.cbegin();
}

VolumetricMask::iterator VolumetricMask::end() noexcept { return mask_.end(); }

VolumetricMask::const_iterator VolumetricMask::end() const noexcept
{
    return mask_.end();
}

VolumetricMask::const_iterator VolumetricMask::cend() const noexcept
{
    return mask_.end();
}

void VolumetricMask::clear() { mask_.clear(); }

std::vector<VolumetricMask::Voxel> VolumetricMask::as_vector() const
{
    return {mask_.begin(), mask_.end()};
}