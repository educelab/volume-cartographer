// denoise_tvl1.h
// Chao Du Oct 2014
// method from ROF 92 paper
// implementation from OpenCV 3, optim module

//#include <opencv2/opencv.hpp>

using namespace cv;

void denoise_TVL1( const std::vector< Mat > &observations, 
					Mat &result,
					double lambda = 1.0,
					int niters = 30 );
