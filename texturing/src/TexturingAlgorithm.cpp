#include "vc/texturing/TexturingAlgorithm.hpp"

using namespace volcart::texturing;

void TexturingAlgorithm::setPerPixelMap(PerPixelMap::Pointer ppm)
{
    ppm_ = std::move(ppm);
}

void TexturingAlgorithm::setVolume(Volume::Pointer vol)
{
    vol_ = std::move(vol);
}

auto TexturingAlgorithm::getTexture() -> Texture { return result_; }

auto TexturingAlgorithm::progressIterations() const -> std::size_t
{
    return ppm_->getMappings().size();
}