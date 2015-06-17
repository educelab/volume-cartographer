#include <iostream>

#include <opencv2/opencv.hpp>

#define BLUR_SIZE 13
#define THRESHOLD 150

#define CENTER cv::Point(original.rows/2, original.cols/2)
#define RADIUS 100

#define WHITE 255
#define WHITE3 cv::Scalar(WHITE,WHITE,WHITE)
#define GRAY 200
#define GRAY3 cv::Scalar(GRAY,GRAY,GRAY)
#define RED cv::Scalar(0,0,255)

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cout << "Usage " << argv[0] << " original" << " /output/dir/" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string inputFile = argv[1];
  std::string outputDir = argv[2];

  cv::Mat original = cv::imread(inputFile);
  cv::Mat workspace;
  cvtColor(original, workspace, CV_BGR2GRAY);
  
  // blur before equalization for better noise reduction
  GaussianBlur(workspace, workspace, cv::Size(BLUR_SIZE, BLUR_SIZE), 0);  
  equalizeHist(workspace, workspace);

  // this is terrible
  threshold(workspace, workspace, THRESHOLD, GRAY, cv::THRESH_BINARY);  
  circle(workspace, CENTER, RADIUS, GRAY, -1);
  floodFill(workspace, CENTER, WHITE);
  threshold(workspace, workspace, 254, WHITE, cv::THRESH_BINARY);

  // calculate contours for convex hull
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(workspace, contours, hierarchy,
                   CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,
                   cv::Point(0,0));

  // calculate convex hulls based on the contours
  std::vector<std::vector<cv::Point> > hull(contours.size());
  for (int i = 0; i < contours.size(); ++i)
    convexHull(cv::Mat(contours[i]), hull[i], false);

  // the outermost hull is the border of the scroll mask
  cv::Mat mask = cv::Mat::zeros(workspace.size(), CV_8UC3);  
  drawContours(mask, hull, 0, GRAY3, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
  drawContours(original, hull, 0, RED, 2, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

  // and again
  floodFill(mask, cv::Point(0,0), WHITE3);
  threshold(mask, mask, 254, WHITE, cv::THRESH_BINARY);

  // invert and apply mask
  bitwise_not(mask, mask);
  //bitwise_and(original, mask, mask);

  // write image to disk
  cv::imwrite( outputDir + "/" + inputFile, mask);
  cv::imwrite( outputDir + "/contour_" + inputFile, original);

  exit(EXIT_SUCCESS);
}
