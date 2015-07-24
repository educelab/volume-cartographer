// main.cpp
// Abigail Coleman Feb. 2015

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "volumepkg.h"
#include "vc_defines.h"
#include "io/ply2itk.h"

#include "texturingUtils.h"
#include <itkRGBPixel.h>
#include "UPointMapping.h"

#define VC_INDEX_X 0
#define VC_INDEX_Y 1
#define VC_INDEX_Z 2

#define VC_DIRECTION_I cv::Vec3f(1,0,0)
#define VC_DIRECTION_J cv::Vec3f(0,1,0)
#define VC_DIRECTION_K cv::Vec3f(0,0,1)

namespace MIKE {
  class Triangle {
  public:
    std::vector<cv::Vec3f> corner;
    std::vector<unsigned short> color;

    double minZ() {
      return std::min(corner[0](VC_INDEX_Z), std::min(corner[1](VC_INDEX_Z), corner[2](VC_INDEX_Z)));
    }
    double maxZ() {
      return std::max(corner[0](VC_INDEX_Z), std::max(corner[1](VC_INDEX_Z), corner[2](VC_INDEX_Z)));
    }

    bool pointInTriangle(cv::Vec3f point) {
      if (sameSide(point, corner[0], corner[1], corner[2]) &&
          sameSide(point, corner[1], corner[0], corner[2]) &&
          sameSide(point, corner[2], corner[0], corner[1])) {
        return true;
      }
      return false;
    }

    cv::Vec3f intersect(cv::Vec3f ray_origin, cv::Vec3f ray_direction) {
      cv::Vec3f normal = (corner[2] - corner[0]).cross(corner[1] - corner[0]);
      double scale_factor = (corner[0] - ray_origin).dot(normal) / ray_direction.dot(normal);
      cv::Vec3f point = scale_factor * ray_direction + ray_origin;
      return point;
    }

  private:

    // a Barycentric method will probably be faster but this works for the time being.
    bool sameSide(cv::Vec3f point0, cv::Vec3f point1, cv::Vec3f alpha, cv::Vec3f beta) {
      cv::Vec3f a2b = beta - alpha;
      cv::Vec3f a2_point0 = point0 - alpha;
      cv::Vec3f a2_point1 = point1 - alpha;
      cv::Vec3f cp0 = a2b.cross(a2_point0);
      cv::Vec3f cp1 = a2b.cross(a2_point1);

      if (cp0.dot(cp1) >= 0) {
        return true;
      }
      return false;
    }
  };

  class TriangleStore {
  public:
    TriangleStore(double e) {
      epsilon = e;
    }
    void push_back(Triangle t) {
      double min = t.minZ();
      double max = t.maxZ();

      min -= epsilon;
      max += epsilon;

      for (int i = std::floor(min); i < std::ceil(max); ++i) {
        bin[i].push_back(t);
      }
    }

    void info() {
      std::cout << bin.size() << " bins used" << std::endl;

    }
  private:
    double epsilon;
    std::map<int,std::vector<Triangle> > bin;
  };


}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
  if ( argc < 6 ) {
    std::cout << "Usage: vc_texture2 volpkg seg-id radius texture-method sample-direction" << std::endl;
    std::cout << "Texture methods: " << std::endl;
    std::cout << "      0 = Intersection" << std::endl;
    std::cout << "      1 = Non-Maximum Suppression" << std::endl;
    std::cout << "      2 = Maximum" << std::endl;
    std::cout << "      3 = Minimum" << std::endl;
    std::cout << "      4 = Median w/ Averaging" << std::endl;
    std::cout << "      5 = Median" << std::endl;
    std::cout << "      6 = Mean" << std::endl;
    std::cout << std::endl;
    std::cout << "Sample Direction: " << std::endl;
    std::cout << "      0 = Omni" << std::endl;
    std::cout << "      1 = Positive" << std::endl;
    std::cout << "      2 = Negative" << std::endl;
    exit( -1 );
  }

  VolumePkg vpkg = VolumePkg( argv[ 1 ] );
  std::string segID = argv[ 2 ];
  if (segID == "") {
    std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
    exit(EXIT_FAILURE);
  }
  if ( vpkg.getVersion() < 2.0) {
    std::cerr << "ERROR: Volume package is version " << vpkg.getVersion() << " but this program requires a version >= 2.0."  << std::endl;
    exit(EXIT_FAILURE);
  }
  vpkg.setActiveSegmentation(segID);
  std::string meshName = vpkg.getMeshPath();
  double radius, minorRadius;
  radius = atof( argv[ 3 ] );
  if((minorRadius = radius / 3) < 1) minorRadius = 1;

  int aFindBetterTextureMethod = atoi( argv[ 4 ] );
  EFilterOption aFilterOption = ( EFilterOption )aFindBetterTextureMethod;

  int aSampleDir = atoi( argv[ 5 ] ); // sampleDirection (0=omni, 1=positive, 2=negative)
  EDirectionOption aDirectionOption = ( EDirectionOption )aSampleDir;

  // declare pointer to new Mesh object
  VC_MeshType::Pointer mesh = VC_MeshType::New();
  int meshWidth = -1;
  int meshHeight = -1;

  if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight)) {
    exit(-1);
  }

  // Matrix to store the output texture
  int textureW = meshWidth;
  int textureH = meshHeight;
  cv::Mat outputTexture = cv::Mat::zeros( textureH, textureW, CV_16UC1 );

  // pointID == point's position in 1D list of points
  // [meshX, meshY] == point's position if list was a 2D matrix
  // [u, v] == point's position in the output matrix
  unsigned long pointID, meshX, meshY;
  double u, v;

  // Load the slices from the volumepkg
  std::vector< cv::Mat > aImgVol;

  /*  This function is a hack to avoid a refactoring the texturing
      methods. See Issue #12 for more details. */
  // Setup
  int meshLowIndex = (int) mesh->GetPoint(0)[2];
  int meshHighIndex = meshLowIndex + meshHeight;
  int aNumSlices = vpkg.getNumberOfSlices();

  int bufferLowIndex = meshLowIndex - (int) radius;
  if (bufferLowIndex < 0) bufferLowIndex = 0;

  int bufferHighIndex = meshHighIndex + (int) radius;
  if (bufferHighIndex >= vpkg.getNumberOfSlices()) {
    bufferHighIndex = vpkg.getNumberOfSlices();
  }

  // Slices must be loaded into aImgVol at the correct index: slice 005 == aImgVol[5]
  // To avoid loading the whole volume, pad the beginning indices with 1x1 null mats
  cv::Mat nullMat = cv::Mat::zeros(1, 1, CV_16U);
  for ( int i = 0; i < bufferLowIndex; ++i ) {
    std::cout << "\rLoading null buffer slices: " << i + 1 << "/" << bufferLowIndex << std::flush;
    aImgVol.push_back( nullMat.clone() );
  }
  std::cout << std::endl;

  // Load the actual volume into a tempVol with a buffer of nRadius
  for ( int i = bufferLowIndex; i < bufferHighIndex; ++i ) {
    std::cout << "\rLoading real slices: " << i - bufferLowIndex + 1 << "/" << bufferHighIndex - bufferLowIndex << std::flush;
    aImgVol.push_back( vpkg.getSliceData( i ).clone() );
  }
  std::cout << std::endl;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  double epsilon = 0.1;

  MIKE::TriangleStore storage(epsilon);

  VC_CellIterator cellIterator = mesh->GetCells()->Begin();
  while(cellIterator != mesh->GetCells()->End()) {
    VC_CellType* cell = cellIterator.Value();
    VC_PointsInCellIterator pointsIterator = cell->PointIdsBegin();

    MIKE::Triangle tri;
    while (pointsIterator != cell->PointIdsEnd()) {
      pointID = *pointsIterator;
      VC_PointType p = mesh->GetPoint(pointID);
      VC_PixelType normal;

      mesh->GetPointData( pointID, &normal );
      meshX = pointID % meshWidth;
      meshY = (pointID - meshX) / meshWidth;

      u = (double)textureW * (double)meshX / (double)meshWidth;
      v = (double)textureH * (double)meshY / (double)meshHeight;

      double color = textureWithMethod(cv::Vec3f(p[0], p[1], p[2]),
                                       cv::Vec3f(normal[0], normal[1], normal[2]),
                                       aImgVol,
                                       aFilterOption,
                                       radius,
                                       minorRadius,
                                       0.5,
                                       aDirectionOption);

      outputTexture.at<unsigned short>(v, u) = (unsigned short)color;

      tri.corner.push_back(cv::Vec3f(p[0], p[1], p[2]));
      tri.color.push_back(color);
      ++pointsIterator;
    }

    storage.push_back(tri);
    ++cellIterator;
  }

  storage.info();

  vpkg.saveTextureData(outputTexture);
  return 0;
}
