#include "vc/core/types/VolumetricMask.hpp"

using namespace volcart;

void VolumetricMask::setIn(const Voxel& v) { mask_.insert(v); }

void VolumetricMask::setOut(const Voxel& v) { mask_.erase(v); }

auto VolumetricMask::isIn(const Voxel& v) const -> bool
{
    return mask_.count(v) > 0;
}

auto VolumetricMask::isOut(const Voxel& v) const -> bool { return not isIn(v); }

auto VolumetricMask::isIn(const cv::Vec3d& v) const -> bool
{
    auto x = static_cast<int>(std::floor(v[0]));
    auto y = static_cast<int>(std::floor(v[1]));
    auto z = static_cast<int>(std::floor(v[2]));
    return isIn(Voxel{x, y, z});
}

auto VolumetricMask::isOut(const cv::Vec3d& v) const -> bool
{
    return not isIn(v);
}

auto VolumetricMask::begin() noexcept -> VolumetricMask::iterator
{
    return mask_.begin();
}

auto VolumetricMask::begin() const noexcept -> VolumetricMask::const_iterator
{
    return mask_.begin();
}

auto VolumetricMask::cbegin() const noexcept -> VolumetricMask::const_iterator
{
    return mask_.cbegin();
}

auto VolumetricMask::end() noexcept -> VolumetricMask::iterator
{
    return mask_.end();
}

auto VolumetricMask::end() const noexcept -> VolumetricMask::const_iterator
{
    return mask_.end();
}

auto VolumetricMask::cend() const noexcept -> VolumetricMask::const_iterator
{
    return mask_.end();
}

void VolumetricMask::clear() { mask_.clear(); }

auto VolumetricMask::empty() const -> bool { return mask_.empty(); }

auto VolumetricMask::as_vector() const -> std::vector<VolumetricMask::Voxel>
{
    return {mask_.begin(), mask_.end()};
}
