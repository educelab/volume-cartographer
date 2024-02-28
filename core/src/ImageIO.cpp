#include "vc/core/io/ImageIO.hpp"

#include <opencv2/imgcodecs.hpp>

#include "vc/core/io/FileFilters.hpp"
#include "vc/core/io/TIFFIO.hpp"
#include "vc/core/types/Exceptions.hpp"
#include "vc/core/util/ImageConversion.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;

namespace vc = volcart;
namespace fs = volcart::filesystem;
namespace tio = volcart::tiffio;

auto vc::ReadImage(const fs::path& path) -> cv::Mat
{
    return cv::imread(path.string(), cv::IMREAD_UNCHANGED);
}

void vc::WriteImage(
    const fs::path& path, const cv::Mat& img, WriteImageOpts opts)
{
    if (img.empty()) {
        vc::Logger()->warn(
            "Image is empty. File will not be written: {}", path.string());
        return;
    }

    auto isJPG = IsFileType(path, {"jpg", "jpeg"});
    auto isPNG = IsFileType(path, {"png"});
    auto isTIF = IsFileType(path, {"tif", "tiff"});

    // Use our TIFF writer
    if (isTIF) {
        auto compress = (opts.compression)
                            ? static_cast<tio::Compression>(*opts.compression)
                            : tio::Compression::LZW;
        tio::WriteTIFF(path, img, compress);
    } else {
        cv::Mat output = img.clone();
        if (img.channels() == 4 and isJPG) {
            vc::Logger()->warn(
                "Image is 4-channel (RGBA) but format {} does not support "
                "4-channels. Extra channel will be removed.",
                path.extension().string());
            output = ColorConvertImage(img, 3);
        } else if (img.channels() == 2) {
            vc::Logger()->warn(
                "Image is 2-channel (Gray + Alpha) but format {} does not "
                "support 2-channels. Extra channel will be removed.",
                path.extension().string());
            output = ColorConvertImage(img, 1);
        }

        // Rescale values as needed
        bool needsRescale = (img.depth() == CV_32F or img.depth() == CV_64F) or
                            (isJPG and img.depth() != CV_8U);

        if (needsRescale) {
            auto depth = DepthToString(img.depth());
            vc::Logger()->warn(
                "Image is {} but format {} does not support this bit "
                "depth. Image will be min-max scaled to the maximum "
                "supported bit depth.",
                depth, path.extension().string());
            if (isPNG) {
                output = QuantizeImage(output, CV_16U);
            } else {
                output = QuantizeImage(output, CV_8U);
            }
        }

        // imwrite params
        std::vector<int> params;
        if (opts.compression) {
            if (isJPG) {
                params.push_back(cv::IMWRITE_JPEG_QUALITY);
            } else if (isPNG) {
                params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            }
            params.push_back(*opts.compression);
        }

        if (not cv::imwrite(path.string(), output, params)) {
            auto msg = "failure writing file '" + path.string() + "'";
            throw IOException(msg);
        }
    }
}
