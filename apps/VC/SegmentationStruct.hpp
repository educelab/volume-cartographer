#pragma once

#include <QCloseEvent>
#include <QComboBox>
#include <QMessageBox>
#include <QObject>
#include <QRect>
#include <QShortcut>
#include <QSpinBox>
#include <QThread>
#include <QTimer>
#include <QtWidgets>

#include "BlockingDialog.hpp"
#include "CBSpline.hpp"
#include "CXCurve.hpp"
#include "MathUtils.hpp"
#include "ui_VCMain.h"

#include "vc/core/types/VolumePkg.hpp"
#include "vc/segmentation/ChainSegmentationAlgorithm.hpp"

#include <thread>
#include <condition_variable>
#include <atomic>
#include <SDL2/SDL.h>
#include <cmath>
#include <unordered_map>

namespace ChaoVis
{

struct SegmentationStruct {
    volcart::VolumePkg::Pointer fVpkg;
    std::string fSegmentationId;
    volcart::Segmentation::Pointer fSegmentation;
    volcart::Volume::Pointer currentVolume = nullptr;
    std::vector<CXCurve> fIntersections;
    CXCurve fIntersectionCurve;
    int fMaxSegIndex = 0;
    int fMinSegIndex = 0;
    volcart::OrderedPointSet<cv::Vec3d> fMasterCloud;
    volcart::OrderedPointSet<cv::Vec3d> fUpperPart;
    std::vector<cv::Vec3d> fStartingPath;
    int fPathOnSliceIndex = 0;
    bool display = false;
    bool compute = false;
    bool highlighted = false;

    // Constructor
    SegmentationStruct() { // Default
    }
    SegmentationStruct(volcart::VolumePkg::Pointer vpkg, std::string segID, volcart::Segmentation::Pointer seg,
                       volcart::Volume::Pointer curVolume,
                       std::vector<CXCurve> intersections,
                       CXCurve intersectionCurve, int maxSegIndex,
                       int minSegIndex,
                       volcart::OrderedPointSet<cv::Vec3d> masterCloud,
                       volcart::OrderedPointSet<cv::Vec3d> upperPart,
                       std::vector<cv::Vec3d> startingPath,
                       int pathOnSliceIndex, bool display, bool compute)
        : fVpkg(vpkg),
          fSegmentationId(segID),
          fSegmentation(seg),
          currentVolume(curVolume),
          fIntersections(intersections),
          fIntersectionCurve(intersectionCurve),
          fMaxSegIndex(maxSegIndex),
          fMinSegIndex(minSegIndex),
          fMasterCloud(masterCloud),
          fUpperPart(upperPart),
          fStartingPath(startingPath),
          fPathOnSliceIndex(pathOnSliceIndex),
          display(display),
          compute(compute) {
    }

    // Destructor to free the dynamically allocated memory
    ~SegmentationStruct() {
        fVpkg = nullptr;
        fSegmentationId.clear();
        fSegmentation = nullptr;
        currentVolume = nullptr;
        fIntersections.clear();
        fMaxSegIndex = 0;
        fMinSegIndex = 0;
        fMasterCloud.clear();
        fUpperPart.clear();
        fStartingPath.clear();
        fPathOnSliceIndex = 0;
        display = false;
        compute = false;
    }

    inline SegmentationStruct(volcart::VolumePkg::Pointer vpkg, std::string segID, int pathOnSliceIndex)
                                                        : fVpkg(vpkg) {
        fSegmentationId = segID;
        SetPathOnSliceIndex(pathOnSliceIndex);

        // reset pointcloud
        ResetPointCloud();

        // Activate requested segmentation
        fSegmentation = fVpkg->segmentation(fSegmentationId);

        // load proper point cloud
        if (fSegmentation->hasPointSet()) {
            fMasterCloud = fSegmentation->getPointSet();
        } else {
            fMasterCloud.reset();
        }

        if (fSegmentation->hasVolumeID()) {
            currentVolume = fVpkg->volume(fSegmentation->getVolumeID());
        }

        SetUpCurves();
        SetPathOnSliceIndex(fMinSegIndex);

        SetCurrentCurve(fPathOnSliceIndex);
    }

    inline void SetPathOnSliceIndex(int nPathOnSliceIndex) {
        fPathOnSliceIndex = nPathOnSliceIndex;
    }

    // Reset point cloud
    inline void ResetPointCloud(void)
    {
        fMasterCloud.reset();
        fUpperPart.reset();
        fStartingPath.clear();
        fIntersections.clear();
        CXCurve emptyCurve;
        fIntersectionCurve = emptyCurve;
    }

    inline void SplitCloud(void)
    {
        // Convert volume z-index to PointSet index
        auto pathIndex = fPathOnSliceIndex - fMinSegIndex;

        if(fMasterCloud.empty() || fPathOnSliceIndex < fMinSegIndex || fPathOnSliceIndex > fMaxSegIndex) {
            fStartingPath = std::vector<cv::Vec3d>();
            return;
        }

        // Upper, "immutable" part
        if (fPathOnSliceIndex > fMinSegIndex) {
            fUpperPart = fMasterCloud.copyRows(0, pathIndex);
        } else {
            fUpperPart = volcart::OrderedPointSet<cv::Vec3d>(fMasterCloud.width());
        }

        // Lower part, the starting path
        fStartingPath = fMasterCloud.getRow(pathIndex);

        // Remove silly -1 points if they exist
        fStartingPath.erase(
            std::remove_if(
                std::begin(fStartingPath), std::end(fStartingPath),
                [](auto e) { return e[2] == -1; }),
            std::end(fStartingPath));

        // Make sure the sizes match now
        if (fStartingPath.size() != fMasterCloud.width()) {
            CleanupSegmentation();
            return;
        }
    }

    inline void CleanupSegmentation(void)
    {
        SetUpCurves();
        SetCurrentCurve(fPathOnSliceIndex);
    }

    // Get the curves for all the slices
    inline void SetUpCurves(void)
    {
        if (fVpkg == nullptr || fMasterCloud.empty()) {
            return;
        }
        fIntersections.clear();
        int minIndex, maxIndex;
        if (fMasterCloud.empty()) {
            minIndex = maxIndex = fPathOnSliceIndex;
        } else {
            minIndex = static_cast<int>(floor(fMasterCloud[0][2]));
            maxIndex = static_cast<int>(fMasterCloud.getRow(fMasterCloud.height()-1)[fMasterCloud.width()-1][2]);            
        }   

        fMinSegIndex = minIndex;
        fMaxSegIndex = maxIndex;

        // assign rows of particles to the curves
        for (size_t i = 0; i < fMasterCloud.height(); ++i) {
            CXCurve aCurve;
            for (size_t j = 0; j < fMasterCloud.width(); ++j) {
                int pointIndex = j + (i * fMasterCloud.width());
                aCurve.SetSliceIndex(
                    static_cast<int>(floor(fMasterCloud[pointIndex][2])));
                aCurve.InsertPoint(Vec2<double>(
                    fMasterCloud[pointIndex][0], fMasterCloud[pointIndex][1]));
            }
            fIntersections.push_back(aCurve);
        }
    }

    // Set the current curve
    inline void SetCurrentCurve(int nCurrentSliceIndex)
    {
        fPathOnSliceIndex = nCurrentSliceIndex;
        int curveIndex = nCurrentSliceIndex - fMinSegIndex;
        if (curveIndex >= 0 &&
            curveIndex < static_cast<int>(fIntersections.size()) &&
            fIntersections.size() != 0) {
            fIntersectionCurve = fIntersections[curveIndex];
        } else {
            CXCurve emptyCurve;
            fIntersectionCurve = emptyCurve;
        }
    }

    // Handle path change event
    void OnPathChanged(void)
    {
        // update current slice
        fStartingPath.clear();
        cv::Vec3d tempPt;
        for (size_t i = 0; i < fIntersectionCurve.GetPointsNum(); ++i) {
            tempPt[0] = fIntersectionCurve.GetPoint(i)[0];
            tempPt[1] = fIntersectionCurve.GetPoint(i)[1];
            tempPt[2] = fPathOnSliceIndex;
            fStartingPath.push_back(tempPt);
        }
    }
};

}  // namespace ChaoVis