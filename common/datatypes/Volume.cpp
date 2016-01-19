#include "Volume.h"
#include "GaussianDistribution3D.h"
#include <sstream>
#include <iomanip>

using namespace volcart;

StructureTensor makeStructureTensor(const cv::Vec3d gradient);

// Trilinear Interpolation: Particles are not required
// to be at integer positions so we estimate their
// normals with their neighbors's known normals.
//
// formula from http://paulbourke.net/miscellaneous/interpolation/
uint16_t Volume::interpolateAt(const Voxel point) const
{
    double int_part;
    double dx = modf(point(VC_INDEX_X), &int_part);
    int x0 = int(int_part);
    int x1 = x0 + 1;
    double dy = modf(point(VC_INDEX_Y), &int_part);
    int y0 = int(int_part);
    int y1 = y0 + 1;
    double dz = modf(point(VC_INDEX_Z), &int_part);
    int z0 = int(int_part);
    int z1 = z0 + 1;

    // insert safety net
    if (x0 < 0 || y0 < 0 || z0 < 0 ||
        x1 >= sliceWidth_ || y1 >= sliceHeight_ || z1 >= ssize_t(numSlices_)) {
        return 0;
    }

    // from: https://en.wikipedia.org/wiki/Trilinear_interpolation
    auto c00 = getIntensityAtCoord(x0, y0, z0) * (1 - dx) +
               getIntensityAtCoord(x1, y0, z0) * dx;
    auto c10 = getIntensityAtCoord(x0, y1, z0) * (1 - dx) +
               getIntensityAtCoord(x1, y0, z0) * dx;
    auto c01 = getIntensityAtCoord(x0, y0, z1) * (1 - dx) +
               getIntensityAtCoord(x1, y0, z1) * dx;
    auto c11 = getIntensityAtCoord(x0, y1, z1) * (1 - dx) +
               getIntensityAtCoord(x1, y1, z1) * dx;

    auto c0 = c00 * (1 - dy) + c10 * dy;
    auto c1 = c01 * (1 - dy) + c11 * dy;

    auto c = c0 * (1 - dz) + c1 * dz;
    return uint16_t(cvRound(c));
}

cv::Mat Volume::getSliceData(const size_t index) const
{
    // Take advantage of caching layer
    const auto possibleSlice = cache_.get(index);
    if (possibleSlice != nullptr) {
        return *possibleSlice;
    }

    const auto slicePath = getSlicePath(index);
    const cv::Mat sliceImg = cv::imread(slicePath.string(), -1);

    // Put into cache so we can use it later
    cache_.put(index, sliceImg);

    return sliceImg;
}

// Data Assignment
bool Volume::setSliceData(const size_t index, const cv::Mat& slice)
{
    if (index >= numSlices_) {
        std::cerr << "ERROR: Atttempted to save a slice image to an out of "
                     "bounds index."
                  << std::endl;
        return false;
    }

    const auto filepath = getSlicePath(index);
    cv::imwrite(filepath.string(), slice);
    return true;
}

boost::filesystem::path Volume::getNormalPathAtIndex(const size_t index) const
{
    std::stringstream ss;
    ss << std::setw(numSliceCharacters_) << std::setfill('0') << index << ".pcd";
    return normalPath_ / ss.str(); 
}

boost::filesystem::path Volume::getSlicePath(const size_t index) const
{
    std::stringstream ss;
    ss << std::setw(numSliceCharacters_) << std::setfill('0') << index << ".tif";
    return slicePath_ / ss.str();
}

uint16_t Volume::getIntensityAtCoord(const int32_t x, const int32_t y,
                                     const int32_t z) const
{
    return getSliceData(z).at<uint16_t>(y, x);
}

void Volume::setCacheSize(const size_t newCacheSize)
{
    cache_.setSize(newCacheSize);
}

void Volume::setCacheMemoryInBytes(const size_t nbytes)
{
    size_t sliceSize = getSliceData(0).step[0] * getSliceData(0).rows;
    setCacheSize(nbytes / sliceSize);
}

Slice Volume::reslice(const Voxel center, const cv::Vec3d xvec,
                      const cv::Vec3d yvec, const int32_t width,
                      const int32_t height) const
{
    const auto xnorm = cv::normalize(xvec);
    const auto ynorm = cv::normalize(yvec);
    const auto origin = center - ((width / 2) * xnorm + (height / 2) * ynorm);

    cv::Mat m(height, width, CV_16UC1);
    for (int h = 0; h < height; ++h) {
        for (int w = 0; w < width; ++w) {
            auto val = interpolateAt(origin + (h * ynorm) + (w * xnorm));
            m.at<uint16_t>(h, w) = val;
        }
    }

    return Slice(m, origin, xnorm, ynorm);
}

StructureTensor Volume::getStructureTensor(const int32_t vx, const int32_t vy,
                                           const int32_t vz,
                                           const int32_t voxelRadius) const
{
    // Safety checks
    assert(vx >= 0 && vx < sliceWidth_ && "x must be in range [0, slice_width]\n");
    assert(vy >= 0 && vy < sliceHeight_ && "y must be in range [0, slice_height]\n");
    assert(vz >= 0 && vz < numSlices_ && "z must be in range [0, #slices]\n");

    // Get voxels in radius voxelRadius around voxel (x, y, z)
    // Since OpenCV doesn't have good accessing mechanisms for tensor-like objects, need
    // to use a std::vector<cv::Mat> to make it easier to compute gradients
    const int32_t n = 2 * voxelRadius + 1;
    std::vector<cv::Mat> cube;
    cube.reserve(n);
    for (int32_t k = vz - voxelRadius; k <= vz + voxelRadius; ++k) {
        auto tmpSlice = cv::Mat(n, n, CV_64F, cv::Scalar(0));

        // If k index is out of bounds, then just add zeros and go on
        if (k < 0) {
            cube.push_back(tmpSlice);
            continue;
        }

        for (int32_t j = vy - voxelRadius, b = 0; j <= vy + voxelRadius; ++j, ++b) {
            for (int32_t i = vx - voxelRadius, a = 0; i <= vx + voxelRadius; ++i, ++a) {
                if (i >= 0 && j >= 0) {
                    tmpSlice.at<double>(b, a) = double(getIntensityAtCoord(i, j, k));
                }
            }
        }
        cube.push_back(tmpSlice);
    }

    // Calculate gradient field around specified voxel
	std::vector<cv::Mat> gradientField;
	gradientField.reserve(n);

	// First do XY gradients
	for (int32_t z = 0; z < n; ++z) {
		cv::Mat xGradient(n, n, CV_64F);
    	cv::Mat yGradient(n, n, CV_64F);
    	cv::Scharr(cube[z], xGradient, CV_64F, 1, 0);
    	cv::Scharr(cube[z], yGradient, CV_64F, 0, 1);
		cv::Mat xyGradients(n, n, CV_64FC3);
		for (int32_t y = 0; y < n; ++y) {
			for (int32_t x = 0; x < n; ++x) {
				xyGradients.at<cv::Vec3d>(y, x) =
					cv::Vec3d(xGradient.at<double>(y, x),
							  yGradient.at<double>(y, x), 0);
			}
		}
		gradientField.push_back(xyGradients);
	}

	// Then Z gradients
	for (int32_t layer = 0; layer < n; ++layer) {
		cv::Mat zSlice(n, n, CV_64F);
		for (int32_t slice = 0; slice < n; ++slice) {
			cube[slice].row(layer).copyTo(zSlice.row(slice));
		}
    	cv::Mat zGradient(n, n, CV_64F);
		cv::Scharr(zSlice, zGradient, CV_64F, 0, 1);
		for (int32_t slice = 0; slice < n; ++slice) {
			for (int32_t x = 0; x < n; ++x) {
				gradientField[slice].at<cv::Vec3d>(layer, x)(2) =
					zGradient.at<double>(slice, x);
			}
		}
	}

    // Convert to tensor volume
    // XXX: Still doing Mike's algorithm for calculating. Assumes radius=1
    /*
    return (1.0 / 7.0) *
           (makeStructureTensor(gradientField[0].at<cv::Vec3d>(1, 1)) +
            makeStructureTensor(gradientField[1].at<cv::Vec3d>(1, 1)) +
            makeStructureTensor(gradientField[2].at<cv::Vec3d>(1, 1)) +
            makeStructureTensor(gradientField[1].at<cv::Vec3d>(0, 1)) +
            makeStructureTensor(gradientField[1].at<cv::Vec3d>(2, 1)) +
            makeStructureTensor(gradientField[1].at<cv::Vec3d>(1, 0)) +
            makeStructureTensor(gradientField[1].at<cv::Vec3d>(1, 2)));
            */

	// Make tensor field
	// Note: This can just be a 1-D array of StructureTensor since we no longer care about
	// the ordering
	auto tensorField = std::vector<StructureTensor>();
	tensorField.reserve(n * n * n);
	for (int32_t z = 0; z < n; ++z) {
		for (int32_t y = 0; y < n; ++y) {
			for (int32_t x = 0; x < n; ++x) {
				tensorField.push_back(makeStructureTensor(gradientField[z].at<cv::Vec3d>(y, x)));
			}
		}
	}

	// Modulate by gaussian distribution (element-wise) and sum
	// The ordeings must match - i.e. the order of adding the tensors to the field must match that of the
	// Gaussian weightings
	const GaussianDistribution3D gaussianField{voxelRadius, GaussianDistribution3D::Ordering::ZYX};
	StructureTensor sum = StructureTensor(0, 0, 0,
										  0, 0, 0,
										  0, 0, 0);
	for (size_t i = 0; i < size_t(n * n * n); ++i) {
		sum += tensorField[i] * gaussianField[i];
	}

    return sum;
}

StructureTensor makeStructureTensor(const cv::Vec3d gradient)
{
    double Ix = gradient(0);
    double Iy = gradient(1);
    double Iz = gradient(2);
    return StructureTensor(Ix * Ix, Ix * Iy, Ix * Iz,
                           Ix * Iy, Iy * Iy, Iy * Iz,
                           Ix * Iz, Iy * Iz, Iz * Iz);
}
