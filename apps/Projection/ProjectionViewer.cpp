#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"

#include <QGuiApplication>
#include <QScreen>
#include <utility>
#include <opencv2/opencv.hpp>

#include "ProjectionViewer.hpp"

FetchSliceThread::FetchSliceThread(
    volcart::Volume::Pointer volume, QObject* parent)
    : QThread(parent), volume_(std::move(volume))
{
}

FetchSliceThread::~FetchSliceThread()
{
    mutex_.lock();
    abort_ = true;
    condition_.wakeOne();
    mutex_.unlock();
    wait();
}

void FetchSliceThread::fetchSlice(int sliceIdx)
{
    QMutexLocker locker(&mutex_);

    sliceIdx_ = sliceIdx;

    if (!isRunning()) {
        start(LowPriority);
    } else {
        restart_ = true;
        condition_.wakeOne();
    }
}

void FetchSliceThread::run()
{
    forever
    {
        if (abort_) {
            return;
        }

        mutex_.lock();
        const int sliceIdx = sliceIdx_;
        mutex_.unlock();

        if (!restart_) {
            const auto slice = volume_->getSliceDataCopy(sliceIdx);
            auto src = volcart::QuantizeImage(slice, CV_8U, false);
            cv::cvtColor(src, src, cv::COLOR_GRAY2BGR);
            emit fetchedSlice(src);
        }

        mutex_.lock();
        if (!restart_) {
            condition_.wait(&mutex_);
        }
        restart_ = false;
        mutex_.unlock();
    }
}

ProjectionThread::ProjectionThread(
    vtkSmartPointer<vtkPlane> cutPlane,
    vtkSmartPointer<vtkStripper> stripper,
    QObject* parent)
    : QThread(parent)
    , cutPlane_(std::move(cutPlane))
    , stripper_(std::move(stripper))
{
}

ProjectionThread::~ProjectionThread()
{
    mutex_.lock();
    abort_ = true;
    condition_.wakeOne();
    mutex_.unlock();
    wait();
}

void ProjectionThread::runProjection(
    cv::Mat& mat, volcart::ProjectionSettings settings, int sliceIdx)
{
    QMutexLocker locker(&mutex_);

    mat_ = mat;
    settings_ = std::move(settings);
    sliceIdx_ = sliceIdx;

    if (!isRunning()) {
        start(LowPriority);
    } else {
        restart_ = true;
        condition_.wakeOne();
    }
}

void ProjectionThread::run()
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

ProjectionViewer::ProjectionViewer(
    volcart::ProjectionSettings* settings,
    vtkSmartPointer<vtkPlane> cutPlane,
    vtkSmartPointer<vtkStripper> stripper,
    const volcart::Volume::Pointer& volume,
    QWidget* parent)
    : QMainWindow(parent)
    , settings_(settings)
    , scaleFactor_(1.0)
    , imageLabel_(new QLabel)
    , scrollArea_(new QScrollArea)
    , splitter_(new QSplitter)
    , sliceLabel_(new QLabel)
    , sliceSlider_(new QSlider(Qt::Horizontal))
    , colorLabel_(new QLabel("Color"))
    , colorComboBox_(new QComboBox)
    , thicknessLabel_(new QLabel("Thickness"))
    , thicknessSpinBox_(new QSpinBox)
    , intersectOnlyCheckBox_(new QCheckBox("Intersect Only"))
    , sliderLayout_(new QVBoxLayout)
    , sliderWidget_(new QWidget)
    , fetchSliceThread_(volume)
    , projectionThread_(std::move(cutPlane), std::move(stripper))
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(
        &fetchSliceThread_, &FetchSliceThread::fetchedSlice, this,
        &ProjectionViewer::update_slice_image_);
    connect(
        &projectionThread_, &ProjectionThread::ranProjection, this,
        &ProjectionViewer::update_pixmap_);

    imageLabel_->setBackgroundRole(QPalette::Base);
    imageLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel_->setScaledContents(true);

    scrollArea_->setBackgroundRole(QPalette::Dark);
    scrollArea_->setWidget(imageLabel_);
    scrollArea_->verticalScrollBar()->installEventFilter(this);

    sliceSlider_->setMinimum(settings_->zMin);
    sliceSlider_->setMaximum(settings_->zMax);
    sliceSlider_->setValue(settings_->zMin);
    connect(
        sliceSlider_, &QSlider::valueChanged, this,
        &ProjectionViewer::handle_slice_change_);

    colorLabel_->setMinimumWidth(10);
    colorComboBox_->addItems({"White", "Red", "Green", "Blue"});
    if (settings_->color == WHITE) {
        colorComboBox_->setCurrentText("White");
    } else if (settings_->color == RED) {
        colorComboBox_->setCurrentText("Red");
    } else if (settings_->color == GREEN) {
        colorComboBox_->setCurrentText("Green");
    } else if (settings_->color == BLUE) {
        colorComboBox_->setCurrentText("Blue");
    }
    colorComboBox_->setMinimumWidth(10);
    connect(
        colorComboBox_, &QComboBox::currentTextChanged, this,
        &ProjectionViewer::handle_settings_change_);

    thicknessLabel_->setMinimumWidth(10);
    thicknessSpinBox_->setMinimum(1);
    thicknessSpinBox_->setValue(settings_->thickness);
    thicknessSpinBox_->setMinimumWidth(10);
    connect(
        thicknessSpinBox_, &QSpinBox::textChanged, this,
        &ProjectionViewer::handle_settings_change_);

    intersectOnlyCheckBox_->setChecked(settings_->intersectOnly);
    intersectOnlyCheckBox_->setMinimumWidth(10);
    connect(
        intersectOnlyCheckBox_, &QCheckBox::stateChanged, this,
        &ProjectionViewer::handle_settings_change_);

    sliderLayout_->addWidget(sliceLabel_);
    sliderLayout_->addWidget(sliceSlider_);
    sliderLayout_->addWidget(colorLabel_);
    sliderLayout_->addWidget(colorComboBox_);
    sliderLayout_->addWidget(thicknessLabel_);
    sliderLayout_->addWidget(thicknessSpinBox_);
    sliderLayout_->addWidget(intersectOnlyCheckBox_);
    sliderLayout_->addStretch();
    sliderWidget_->setLayout(sliderLayout_);

    splitter_->addWidget(scrollArea_);
    splitter_->addWidget(sliderWidget_);
    splitter_->setSizes({2, 1});

    setCentralWidget(splitter_);

    const float sizeRatio = 3.0 / 5.0;
    resize(QGuiApplication::primaryScreen()->availableSize() * sizeRatio);

    handle_slice_change_();
}

void ProjectionViewer::update_slice_image_(const cv::Mat& mat)
{
    originalSliceMat_ = mat;
    bool isFirstImage = false;
    if (imageLabel_->pixmap(Qt::ReturnByValue).isNull()) {
        isFirstImage = true;
    }
    update_pixmap_(mat);
    if (isFirstImage) {
        auto windowWidth = static_cast<float>(this->width());
        auto estimatedScrollAreaWidth = windowWidth * 2 / 3;
        scaleFactor_ =
            estimatedScrollAreaWidth /
            static_cast<float>(imageLabel_->pixmap(Qt::ReturnByValue).width());
        imageLabel_->resize(
            scaleFactor_ * imageLabel_->pixmap(Qt::ReturnByValue).size());
    }
    handle_settings_change_();
}

void ProjectionViewer::update_pixmap_(const cv::Mat& mat)
{
    QImage image = QImage(
        mat.data, mat.cols, mat.rows, static_cast<int>(mat.step),
        QImage::Format_BGR888);
    QPixmap pixmap = QPixmap::fromImage(image);
    imageLabel_->setPixmap(pixmap);
}

void ProjectionViewer::handle_slice_change_()
{
    sliceLabel_->setText("Slice: " + QString::number(sliceSlider_->value()));
    fetchSliceThread_.fetchSlice(sliceSlider_->value());
}

void ProjectionViewer::handle_settings_change_()
{
    switch (Color(colorComboBox_->currentIndex())) {
        case Color::White:
            settings_->color = WHITE;
            break;
        case Color::Red:
            settings_->color = RED;
            break;
        case Color::Green:
            settings_->color = GREEN;
            break;
        case Color::Blue:
            settings_->color = BLUE;
            break;
    }
    settings_->thickness = thicknessSpinBox_->value();
    settings_->intersectOnly = intersectOnlyCheckBox_->isChecked();

    projectionThread_.runProjection(
        originalSliceMat_, *settings_, sliceSlider_->value());
}

// https://stackoverflow.com/questions/22053102/scrollbar-captures-wheel-event-by-focus
auto ProjectionViewer::eventFilter(QObject* obj, QEvent* event) -> bool
{
    if (event->type() == QEvent::Wheel) {
        auto* wEvent = dynamic_cast<QWheelEvent*>(event);

        if (obj == scrollArea_->verticalScrollBar()) {
            wheelEvent(wEvent);
            return true;
        }
    }

    return false;
}

void ProjectionViewer::wheelEvent(QWheelEvent* event)
{
    int numDegrees = event->angleDelta().y() / 8;

    if (numDegrees > 0) {
        zoom_in_();
    } else if (numDegrees < 0) {
        zoom_out_();
    }

    event->accept();
}

void ProjectionViewer::zoom_in_() { scale_image_(1.25); }

void ProjectionViewer::zoom_out_() { scale_image_(0.8); }

void ProjectionViewer::scale_image_(double factor)
{
    scaleFactor_ *= factor;
    imageLabel_->resize(
        scaleFactor_ * imageLabel_->pixmap(Qt::ReturnByValue).size());

    AdjustScrollBar(scrollArea_->horizontalScrollBar(), factor);
    AdjustScrollBar(scrollArea_->verticalScrollBar(), factor);
}

void ProjectionViewer::AdjustScrollBar(QScrollBar* scrollBar, double factor)
{
    scrollBar->setValue(
        int(factor * scrollBar->value() +
            ((factor - 1) * scrollBar->pageStep() / 2)));
}
#pragma clang diagnostic pop
