// main.cpp
// Abigail Coleman Feb. 2015

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "volumepkg.h"
#include "vc_defines.h"
#include "io/ply2itk.h"
#include "itk2vtk.h"

#include "texturingUtils.h"
#include <itkRGBPixel.h>
#include "UPointMapping.h"

#include <vtkSmartPointer.h>
#include <vtkOBBTree.h>

#define VC_INDEX_X 0
#define VC_INDEX_Y 1
#define VC_INDEX_Z 2

#define VC_DIRECTION_I cv::Vec3f(1,0,0)
#define VC_DIRECTION_J cv::Vec3f(0,1,0)
#define VC_DIRECTION_K cv::Vec3f(0,0,1)

namespace rayTrace {
  class Triangle {
  public:
    std::vector<cv::Vec3f> corner;

    double minBy(int index) {
      return std::min(corner[0](index), std::min(corner[1](index), corner[2](index)));
    }
    double maxBy(int index) {
      return std::max(corner[0](index), std::max(corner[1](index), corner[2](index)));
    }

    // Determine if a point is within a triangle by checking if the point
    // is on the same side of each line of the triangle in reference to
    // the third point of the triangle
    bool pointInTriangle(cv::Vec3f point) {
      if (sameSide(point, corner[0], corner[1], corner[2]) &&
          sameSide(point, corner[1], corner[0], corner[2]) &&
          sameSide(point, corner[2], corner[0], corner[1])) {
        return true;
      }
      return false;
    }

    // Determine intersection point using the origin and direction of the ray
    cv::Vec3f intersect(cv::Vec3f ray_origin, cv::Vec3f ray_direction) {
      cv::Vec3f normal = this->normal();
      double scale_factor = (corner[0] - ray_origin).dot(normal) / ray_direction.dot(normal);
      cv::Vec3f point = scale_factor * ray_direction + ray_origin;
      return point;
    }

    // Calculate the normal of the triangle
    cv::Vec3f normal() {
      cv::Vec3f a2b = corner[1] - corner[0];
      cv::Vec3f a2c = corner[2] - corner[0];
      return a2b.cross(a2c);
    }

  private:

    // Simple same side technique, Barycentric would be more robust
    // beta and alpha are the two points that form the line to check the side of the reference point
    // point0 is the reference point and point1 is the third point of the triangle
    bool sameSide(cv::Vec3f point0, cv::Vec3f point1, cv::Vec3f alpha, cv::Vec3f beta) {
      cv::Vec3f a2b = beta - alpha;
      cv::Vec3f a2_point0 = point0 - alpha;

      // Margin used to check if a reference point lies on a line of a triangle
      double epsilon = 0.00001;

      // Check if the vector to the reference point is very close to zero
      if (cv::norm(a2_point0) <= epsilon) {
        return true;
      }

      cv::Vec3f a2_point1 = point1 - alpha;
      cv::Vec3f cp0 = a2b.cross(a2_point0);
      cv::Vec3f cp1 = a2b.cross(a2_point1);

      // check if the reference point is on the line
      if (cv::norm(cp0) < epsilon) {
        return true;
      }

      // Check if the vectors are facing the same way
      // to see if the reference point is on the same side
      if (cp0.dot(cp1) >= (0 - epsilon)) {
        return true;
      }
      return false;
    }
  }; // Triangle class

  // The triangles are stored in bins where each bin represents a row/slice 
  class TriangleStore {
  public:
    TriangleStore(double e = 0.1) {
      epsilon_ = e;

      // Used to keep track of the dimensions of the mesh
      upper_bound_x_ = 0;
      lower_bound_x_ = std::numeric_limits<double>::max();

      upper_bound_y_ = 0;
      lower_bound_y_ = std::numeric_limits<double>::max();

      upper_bound_z_ = 0;
      lower_bound_z_ = std::numeric_limits<double>::max();

    }

    void push_back(Triangle t) {
      double minZ = t.minBy(VC_INDEX_Z);
      double maxZ = t.maxBy(VC_INDEX_Z);

      minZ -= epsilon_;
      maxZ += epsilon_;

      for (int i = std::floor(minZ); i < std::ceil(maxZ); ++i) {
        bin_[i].push_back(t);
      }

      // record upper and lower bounds of triangle Xs, Ys, and Zs
      if (t.minBy(VC_INDEX_X) < lower_bound_x_) {
        lower_bound_x_ = t.minBy(VC_INDEX_X);
      }
      if (t.maxBy(VC_INDEX_X) > upper_bound_x_) {
        upper_bound_x_ = t.maxBy(VC_INDEX_X);
      }
      if (t.minBy(VC_INDEX_Y) < lower_bound_y_) {
        lower_bound_y_ = t.minBy(VC_INDEX_Y);
      }
      if (t.maxBy(VC_INDEX_Y) > upper_bound_y_) {
        upper_bound_y_ = t.maxBy(VC_INDEX_Y);
      }
      if (minZ < lower_bound_z_) {
        lower_bound_z_ = minZ;
      }
      if (maxZ > upper_bound_z_) {
        upper_bound_z_ = maxZ;
      }
    }

    void info() {
      std::cout << bin_.size() << " bins used" << std::endl;
      std::cout << "x bounds: [" << lower_bound_x_ << " .. " << upper_bound_x_ << "]" << std::endl;
      std::cout << "y bounds: [" << lower_bound_y_ << " .. " << upper_bound_y_ << "]" << std::endl;
      std::cout << "z bounds: [" << lower_bound_z_ << " .. " << upper_bound_z_ << "]" << std::endl;
    }

    double epsilon_;
    double upper_bound_x_;
    double lower_bound_x_;

    double upper_bound_y_;
    double lower_bound_y_;

    double upper_bound_z_;
    double lower_bound_z_;

    std::map<int,std::vector<Triangle> > bin_;
  }; // TriangleStore class

} // namespace rayTrace
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
  if ( argc < 7 ) {
    std::cout << "Usage: vc_texture2 volpkg seg-id radius texture-method sample-direction tracing-direction" << std::endl;
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
    std::cout << "Ray Tracing Direction: " << std::endl;
    std::cout << "      0 = Clockwise" << std::endl;
    std::cout << "      1 = Counterclockwise" << std::endl;
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

  int aTraceDir = atoi( argv[ 6 ] ); //rayTracingDirection (0=clockwise, 1=counterclockwise)

  // declare pointer to new Mesh object
  VC_MeshType::Pointer mesh = VC_MeshType::New();
  int meshWidth = -1;
  int meshHeight = -1;

  if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight)) {
    exit(-1);
  }

  // Convert the itk mesh to a vtk mesh
  vtkPolyData *vtkMesh = vtkPolyData::New();
  volcart::meshing::itk2vtk(mesh, vtkMesh);

  // Matrix to store the output texture
  int textureW = meshWidth;
  int textureH = meshHeight;
  //  cv::Mat outputTexture = cv::Mat::zeros( textureH, textureW, CV_16UC1 );

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

  // Important data structure used to search for a
  // ray's intersection within a triangle
  rayTrace::TriangleStore storage;

  // Load triangles into the storage data structure
  // according to the z index of the triangle's points
  VC_CellIterator cellIterator = mesh->GetCells()->Begin();
  while(cellIterator != mesh->GetCells()->End()) {
    VC_CellType* cell = cellIterator.Value();
    VC_PointsInCellIterator pointsIterator = cell->PointIdsBegin();

    rayTrace::Triangle tri;
    while (pointsIterator != cell->PointIdsEnd()) {
      pointID = *pointsIterator;
      VC_PointType p = mesh->GetPoint(pointID);
      tri.corner.push_back(cv::Vec3f(p[0], p[1], p[2]));
      ++pointsIterator;
    }

    storage.push_back(tri);
    ++cellIterator;
  }

  // print the bounds of the storage data structure
  storage.info();

  // Create point cloud to save new interpolated points into
  // mainly used for debugging
  pcl::PointCloud<pcl::PointNormal>::Ptr new_cloud ( new pcl::PointCloud<pcl::PointNormal> );

// Parameters used to create cylindrical ray tracing
#define PI_X2 (2 * 3.1415926)
#define OUT_X 2000
#define D_THETA (PI_X2 / OUT_X)

  cv::Mat outputTexture = cv::Mat::zeros( (int)(storage.upper_bound_z_ - storage.lower_bound_z_), OUT_X, CV_16UC1 );
  
  // // For each slice/row generate rays and interpolate new points
  // for (int z = (int)storage.lower_bound_z_; z < (int)storage.upper_bound_z_; ++z) {
  //   int counter = 0;
  //   // grab current row from the storage data structure
  //   std::vector<rayTrace::Triangle> triangle_row = storage.bin_[z];

  //   int ycount = 0;
  //   double radian = 0;
  //   // generate rays cylindrically
  //   for (double r = 0; r < PI_X2; r += D_THETA, ycount++) {
  //     // Calculate the ray according to ray tracing direction
  //     if (aTraceDir == 1) {
  //       // counterclockwise
  //       radian -= D_THETA;
  //     } else {
  //       // clockwise (default)
  //       radian += D_THETA;
  //     }
  //     // Calculate the origin by averaging the bounds of each coordinate
  //     cv::Vec3f origin;
  //     origin(VC_INDEX_X) = (storage.lower_bound_x_ + storage.upper_bound_x_) / 2;
  //     origin(VC_INDEX_Y) = (storage.lower_bound_y_ + storage.upper_bound_y_) / 2;
  //     origin(VC_INDEX_Z) = z;
  //     // Calculate direction of ray according to current degree of rotation along the cylinder
  //     cv::Vec3f direction(cos(radian), sin(radian), 0);

  //     // Check if each triangle in the current storage bin intersects with the current ray
  //     for (int t = 0; t < triangle_row.size(); ++t) {
  //       rayTrace::Triangle tri = triangle_row[t];
  //       cv::Vec3f p = tri.intersect(origin,direction);

  //       // If the ray intersects with current triangle then texture the intersecting point
  //       if (tri.pointInTriangle(p) && direction.dot(tri.normal()) < 0) {
  //         double color = textureWithMethod( p,
  //                                           tri.normal(),
  //                                           aImgVol,
  //                                           aFilterOption,
  //                                           radius,
  //                                           minorRadius,
  //                                           0.5,
  //                                           aDirectionOption);
  //         outputTexture.at<unsigned short>(z - storage.lower_bound_z_, ycount) = (unsigned short)color;

  //         // Generate pcl point cloud
  //         pcl::PointNormal asdf;
  //         asdf.x = p[0];
  //         asdf.y = p[1];
  //         asdf.z = p[2];
  //         new_cloud->push_back(asdf);
  //         break;
  //       } else {
  //         // Keep track of how many rays had no intersections
  //         if (t == triangle_row.size() - 1) {
  //           counter++;
  //         }
  //       }
  //     }
  //   }
  //   std::cout << "line " << z << ": " << counter << " misses" <<  "\t\t" << counter / (OUT_X / PI_X2) << std::endl;
  // }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Using vtk's OBBTree to test a ray's intersection with the faces/cells/triangles in the mesh

  // Creat vtk OBBTree
  vtkSmartPointer<vtkOBBTree> obbTree = vtkSmartPointer<vtkOBBTree>::New();
  obbTree->SetDataSet(vtkMesh);
  obbTree->BuildLocator();

  vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> intersectCells = vtkSmartPointer<vtkCellArray>::New();

  // For each slice/row generate rays and interpolate new points
  for (int z = (int)storage.lower_bound_z_; z < (int)storage.upper_bound_z_; ++z) {
    int counter = 0;

    int ycount = 0;
    double radian = 0;
    // generate rays cylindrically
    for (double r = 0; r < PI_X2; r += D_THETA, ycount++) {
      // Calculate the ray according to ray tracing direction
      if (aTraceDir == 1) {
        // counterclockwise
        radian -= D_THETA;
      } else {
        // clockwise (default)
        radian += D_THETA;
      }
      // Calculate the origin by averaging the bounds of each coordinate
      cv::Vec3f origin;
      origin(VC_INDEX_X) = (storage.lower_bound_x_ + storage.upper_bound_x_) / 2;
      origin(VC_INDEX_Y) = (storage.lower_bound_y_ + storage.upper_bound_y_) / 2;
      origin(VC_INDEX_Z) = z;
      // Calculate direction of ray according to current degree of rotation along the cylinder
      cv::Vec3f direction(cos(radian), sin(radian), 0);
      cv::normalize(direction);
      // Create a second point along the ray using the origin and direction
      cv::Vec3f end_point;
      end_point = origin + 1000*direction;
      origin = origin - 1000*direction;
      double start[3] = {origin[0], origin[1], origin[2]};
      double end[3] = {end_point[0], end_point[1], end_point[2]};

      obbTree->IntersectWithLine(start, end, intersectPoints, NULL);

      std::cout << "NumPoints: " << intersectPoints->GetNumberOfPoints() <<  " | Z: " << z << std::endl;

      if ( intersectPoints->GetNumberOfPoints() > 0 ) {
        cv::Vec3f p = intersectPoints[0];
        pcl::PointNormal asdf;
        asdf.x = p[0];
        asdf.y = p[1];
        asdf.z = p[2];
        new_cloud->push_back(asdf);
      }
    }
  }


  pcl::io::savePCDFileASCII("resampled.pcd", *new_cloud);
  vpkg.saveTextureData(outputTexture);
  return 0;
} // main
