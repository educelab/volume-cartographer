#include "SliceProjectionThread.hpp"

SliceProjectionThread::SliceProjectionThread(
    vtkSmartPointer<vtkPlane> cutPlane,
    vtkSmartPointer<vtkStripper> stripper,
    QObject* parent)
    : QThread(parent)
    , cutPlane_(std::move(cutPlane))
    , stripper_(std::move(stripper))
{
}

SliceProjectionThread::~SliceProjectionThread()
{
    mutex_.lock();
    abort_ = true;
    condition_.wakeOne();
    mutex_.unlock();
    wait();
}

void SliceProjectionThread::runProjection(
    cv::Mat& mat, volcart::ProjectionSettings settings, int sliceIdx)
{
    QMutexLocker locker(&mutex_);

    mat_ = mat.clone();
    settings_ = std::move(settings);
    sliceIdx_ = sliceIdx;

    if (!isRunning()) {
        start(LowPriority);
    } else {
        restart_ = true;
        condition_.wakeOne();
    }
}

void SliceProjectionThread::run()
{
    forever
    {
        if (abort_) {
            return;
        }

        mutex_.lock();
        cv::Mat src = mat_;
        volcart::ProjectionSettings settings = settings_;
        // Cut the mesh and get the intersection
        cutPlane_->SetOrigin(
            src.size().width / 2.0, src.size().height / 2.0, sliceIdx_);
        stripper_->Update();
        auto* intersection = stripper_->GetOutput();
        mutex_.unlock();

        if (!restart_) {
            cv::Mat outputImg;
            std::vector<cv::Point> contour;

            // Setup the output image
            if (settings.intersectOnly) {
                outputImg = cv::Mat::zeros(
                    src.size().height, src.size().width, CV_8UC3);
            } else {
                outputImg = src;
            }

            // Draw the intersections
            for (auto cId = 0; cId < intersection->GetNumberOfCells(); ++cId) {
                auto* inputCell = intersection->GetCell(cId);

                contour.clear();
                for (auto pIt = 0; pIt < inputCell->GetNumberOfPoints();
                     ++pIt) {
                    auto pId = inputCell->GetPointId(pIt);
                    contour.emplace_back(cv::Point(
                        static_cast<int>(intersection->GetPoint(pId)[0]),
                        static_cast<int>(intersection->GetPoint(pId)[1])));
                }
                cv::polylines(
                    outputImg, contour, false, settings.color,
                    settings.thickness, cv::LINE_AA);
            }
            emit ranProjection(outputImg);
        }

        mutex_.lock();
        if (!restart_) {
            condition_.wait(&mutex_);
        }
        restart_ = false;
        mutex_.unlock();
    }
}
