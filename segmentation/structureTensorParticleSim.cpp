#include "structureTensorParticleSim.h"

// structureTensorParticleSim uses "particles" to trace out the surfaces in a volume.
// A Particle Chain maintains their ordering and is responsible for updating their
// positions. This update moves particles according to the estimated normal vector.
//
//

// used for tangent calculation
#define DELTA 0.01
#define COLOR_NORMAL cv::Scalar(0, 255, 255)
#define COLOR_TANGENT cv::Scalar(255, 0, 255)

// (x, y, slice)
#define SLICE_DIR cv::Vec3f(0,0,1)

// cubic interpolation
template<typename T> T
interpolate(T y0, T y1 ,T y2, T y3, double p) {
  double mu = p;
  double mu2 = p * p;
  T a0 = y3 - y2 - y0 + y1;
  T a1 = y0 - y1 - a0;
  T a2 = y2 - y0;
  T a3 = y1;
  return (a0 * mu * mu2) + (a1 * mu2) + (a2 * mu) + a3;
}

// tangent error is bounded by O(delta^2)
// this result is derivable via taylor expansion of f(x + delta)
template<typename T> T
tangent(T y1, T left, T right, T y4, double p) {
  T lhs = interpolate(y1, left, right, y4, p - DELTA);
  T rhs = interpolate(y1, left, right, y4, p + DELTA);
  return rhs - lhs;
}

// paths don't really have "normal" vectors so we that it only has components in the xy plane
cv::Vec3f normal(cv::Vec3f y1, cv::Vec3f left, cv::Vec3f right, cv::Vec3f y4, double p) {
  cv::Vec3f t = tangent(y1, left, right, y4, p);
  return t.cross(SLICE_DIR);
}

std::vector<cv::Vec3f> click_list;

// callback for getting control points
// will likely be removed later
void mouse(int event, int x, int y, int flags, void* param) {
  switch (event) {
  case cv::EVENT_LBUTTONDOWN:
    cv::Mat img = *((cv::Mat*)(param));
    circle(img, cv::Point(x,y), 5, cv::Scalar(0,255,0),1);
    click_list.push_back(cv::Vec3f(x,y,0));

    int size = click_list.size();
    if (size >= 4) {
      cv::Vec3f y1 = click_list[size - 4];
      cv::Vec3f left = click_list[size - 3];
      cv::Vec3f right = click_list[size - 2];
      cv::Vec3f y4 = click_list[size - 1];

      for (int i = 1; i < 10; ++i) {
        cv::Vec3f v = interpolate(y1, left, right, y4, i / 10.0);
        cv::Point p((int)v(0),(int)v(1));
        circle(img, p, 2, cv::Scalar(255,255,0),1);
      }

      cv::Vec3f v = interpolate(y1, left, right, y4, 0.5);
      cv::Vec3f t = tangent(    y1, left, right, y4, 0.5);
      cv::Vec3f n = normal(     y1, left, right, y4, 0.5);

      normalize(t,t,20);
      normalize(n,n,20);
      cv::Point start((int)v(0),(int)v(1));
      cv::Point tan((int)t(0)  ,(int)t(1));
      cv::Point norm((int)n(0) ,(int)n(1));
      arrowedLine(img,start,start + tan, COLOR_TANGENT);
      arrowedLine(img,start,start + norm, COLOR_NORMAL);
    }
    imshow("IMAGE", img);
    break;
  }
}


pcl::PointCloud<pcl::PointXYZRGB> structureTensorParticleSim(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg volpkg, double gravity_scale, int threshold, int endOffset) {

  Field f(&volpkg);

  cv::Mat in = volpkg.getSliceAtIndex(42);
  in *= 1./255;

  cv::Mat slice;
  in.convertTo(slice, CV_8UC3);
  cvtColor(slice, slice, CV_GRAY2BGR);

  namedWindow("IMAGE", cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback("IMAGE", mouse, &slice);
  imshow("IMAGE", slice);
  cv::waitKey(0);

  cv::Vec3f p(160,210,50);
  cv::Vec3f n(1,0,0);
  
  cv::Mat m = f.reslice(p, n);
  m *= 1./255;
  cv::Mat reslice;
  m.convertTo(reslice, CV_8UC3);
  cvtColor(reslice, reslice, CV_GRAY2BGR);

  cv::Point imcenter(RESLICE_WIDTH/2, p(2));
  arrowedLine(reslice, imcenter, imcenter + cv::Point(RESLICE_WIDTH/2 - 1, 0), COLOR_NORMAL);
  circle(reslice, imcenter, 2, COLOR_TANGENT, -1);
  
  namedWindow("RESLICE", cv::WINDOW_AUTOSIZE);
  imshow("RESLICE", reslice);
  cv::waitKey(0);
  

  return pcl::PointCloud<pcl::PointXYZRGB>();
}
