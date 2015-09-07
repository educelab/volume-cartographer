#include "localResliceParticleSim.h"

// structureTensorParticleSim uses "particles" to trace out the surfaces in a volume.
// A Particle Chain maintains their ordering and is responsible for updating their
// positions. This update moves particles according to the estimated normal vector.
//
// NOTE: This segmentation stores points as XYZ!!!
// field.h defines slice direction this way
// it will eventually be moved to volumepkg

// RADIAL RESLICE DEMO
// used to set core_fst and core_lst for finding the axis of rotation
/*
void core_callback(int event, int x, int y, int flags, void *point) {
    switch (event) {
        case cv::EVENT_LBUTTONDOWN: {
            cv::Point *p = (cv::Point *) point;
            p->x = x;
            p->y = y;

            std::cout << "point " << *p << std::endl;
            break;
        }
        default: break;
    }
}
 */

namespace volcart {

namespace segmentation {

pcl::PointCloud<pcl::PointXYZRGB> localResliceParticleSim(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath,
                                                          VolumePkg& volpkg, int threshold,
                                                          int endOffset) {
    Field f(volpkg);

    // // RESLICE DEMO
    // // test point and normal for checking with fiji
    // cv::Vec3f p(168,200,50);
    // cv::Vec3f n(1,0,0);
    // for (int i = 0; i < 100; ++i) {
    //   Slice s = f.reslice(p, n, VC_DIRECTION_K);
    //   p = s.findNextPosition();
    //   s.debugDraw(DEBUG_DRAW_CENTER);
    //   s.debugAnalysis();
    //   cv::waitKey(0);
    // }
    // cv::destroyAllWindows();

    // // RADIAL RESLICE DEMO
    // // click on the center of the scroll for the first and last slices
    // // then press a button
    // cv::Point core_fst;
    // cv::Mat first_slice = volpkg.getSliceData(0);
    // namedWindow("FIRST SLICE", cv::WINDOW_AUTOSIZE);
    // cv::setMouseCallback("FIRST SLICE", core_callback, &core_fst);
    // imshow("FIRST SLICE", first_slice);

    // cv::Point core_lst;
    // cv::Mat last_slice = volpkg.getSliceData(volpkg.getNumberOfSlices() - 1);
    // namedWindow("LAST SLICE", cv::WINDOW_AUTOSIZE);
    // cv::setMouseCallback("LAST SLICE", core_callback, &core_lst);
    // imshow("LAST SLICE", last_slice);

    // cv::waitKey(0);
    // cv::destroyAllWindows();

    // cv::Point diff = core_lst - core_fst;
    // cv::Vec3f axis(diff.x, diff.y, volpkg.getNumberOfSlices() - 1);
    // cv::Vec3f origin(core_fst.x, core_fst.y, 0);

    // double PI = 3.14159;
    // for (double theta = 0; theta < 2*PI; theta += (PI / 360.0)) {
    //   Slice s = f.resliceRadial(origin, axis, theta, 250, 300);
    //   s.debugDraw(DEBUG_DRAW_XYZ | DEBUG_DRAW_CORNER_COORDINATES);
    //   cv::waitKey(0);
    // }

    // CHAIN DEMO
    // try to find the surface from slices along the chain
    std::cout << "Using local reslice particle sim algorithm" << std::endl;
    Chain c(segPath, volpkg, threshold, endOffset, 10);
    for (int i = 0; c.isMoving() && i < 200; ++i) {
        c.step(f);
        c.debug();
    }

    return c.orderedPCD();
}
}// namespace segmentation
}// namespace volcart
