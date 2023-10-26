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
#include <set>

namespace ChaoVis
{

struct AnnotationStruct {
    bool anchor{false};
    bool manual{false}; // at least one point was manually changed on that slice this annotation belongs to
};

struct SegmentationStruct {
    volcart::VolumePkg::Pointer fVpkg;
    std::string fSegmentationId;
    volcart::Segmentation::Pointer fSegmentation;
    volcart::Volume::Pointer currentVolume = nullptr;
    std::vector<CXCurve> fIntersections;
    std::map<int, CXCurve> fIntersectionsChanged; // manually changed curves that were not saved yet into the master cloud (key = slice index)
    CXCurve fIntersectionCurve; // current active/shown curve
    int fMaxSegIndex = 0; // index on which the segment ends
    int fMinSegIndex = 0; // index on which the segment starts
    volcart::OrderedPointSet<cv::Vec3d> fMasterCloud;
    volcart::OrderedPointSet<cv::Vec3d> fUpperPart;
    volcart::OrderedPointSet<cv::Vec2i> fAnnotationCloud;
    std::vector<cv::Vec3d> fStartingPath;
    std::map<int, AnnotationStruct> fAnnotations; // decoded annotations
    std::set<int> fBufferedChangedPoints; // values are in range [1..(number of points on curve)] (not global cloud index, but locally to the edited curve)
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
                       volcart::OrderedPointSet<cv::Vec2i> annotations,
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
          fAnnotationCloud(annotations),
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
        fIntersectionsChanged.clear();
        fMaxSegIndex = 0;
        fMinSegIndex = 0;
        fMasterCloud.clear();
        fUpperPart.clear();
        fAnnotationCloud.clear();
        fAnnotations.clear();
        fStartingPath.clear();
        fPathOnSliceIndex = 0;
        display = false;
        compute = false;
    }

    inline SegmentationStruct(volcart::VolumePkg::Pointer vpkg, std::string segID, int pathOnSliceIndex)
                                                        : fVpkg(vpkg) {
        fSegmentationId = segID;
        SetPathOnSliceIndex(pathOnSliceIndex);

        // reset point cloud
        ResetPointCloud();

        // Activate requested segmentation
        fSegmentation = fVpkg->segmentation(fSegmentationId);

        // load master point cloud
        if (fSegmentation->hasPointSet()) {
            fMasterCloud = fSegmentation->getPointSet();

            // load annotations
            if (fSegmentation->hasAnnotations()) {
                fAnnotationCloud = fSegmentation->getAnnotationSet();
            } else {
                // create and store annotation set if not present            
                fSegmentation->setAnnotationSet(CreateInitialAnnotationSet(fMasterCloud.height(), fMasterCloud.width()));
            }  
        } else {
            fMasterCloud.reset();
            fAnnotationCloud.reset();
        }

        if (fSegmentation->hasVolumeID()) {
            currentVolume = fVpkg->volume(fSegmentation->getVolumeID());
        }

        SetUpCurves();
        SetUpAnnotations();

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
        fIntersectionsChanged.clear();
        CXCurve emptyCurve;
        fIntersectionCurve = emptyCurve;
        fAnnotationCloud.reset();
        fAnnotations.clear();
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
        SetUpAnnotations();
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
        minIndex = static_cast<int>(floor(fMasterCloud[0][2]));
        maxIndex = static_cast<int>(fMasterCloud.getRow(fMasterCloud.height()-1)[fMasterCloud.width()-1][2]);      

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

    // Get the annotations for all the slices
    inline void SetUpAnnotations(void)
    {
        if (fVpkg == nullptr || fMasterCloud.empty() || fAnnotationCloud.empty()) {
            return;
        }
        
        fAnnotations.clear();
        for (size_t i = 0; i < fAnnotationCloud.height(); ++i) {
            AnnotationStruct an;
            int pointIndex;

            for (size_t j = 0; j < fAnnotationCloud.width(); ++j) {
                pointIndex = j + (i * fAnnotationCloud.width());
                
                if(fAnnotationCloud[pointIndex][0] == 1)
                    an.anchor = true;
                
                if(fAnnotationCloud[pointIndex][1] == 1)
                    an.manual = true;
            }

            fAnnotations[fMasterCloud[pointIndex][2]] = an;
        }
    }

    // Set the current curve
    inline void SetCurrentCurve(int nCurrentSliceIndex)
    {
        SetPathOnSliceIndex(nCurrentSliceIndex);
        int curveIndex = nCurrentSliceIndex - fMinSegIndex;
        if (curveIndex >= 0 &&
            curveIndex < static_cast<int>(fIntersections.size()) &&
            fIntersections.size() != 0) {

            // If we have a buffered changed curve, use that one.
            // Note: The map of changed intersections uses the slice nubmer as key,
            // where as the intersections vector needs to be accessed by the curveIndex (offset)
            auto it = fIntersectionsChanged.find(fPathOnSliceIndex);
            if(it != fIntersectionsChanged.end())
                fIntersectionCurve = it->second;
            else
                fIntersectionCurve = fIntersections[curveIndex];
        } else {
            CXCurve emptyCurve;
            fIntersectionCurve = emptyCurve;
        }
    }

    inline void ForgetChangedCurves()
    {
        fIntersectionsChanged.clear();
    }

    inline void MergePointSetIntoPointCloud(const volcart::Segmentation::PointSet ps)
    {        
        // Ensure that everything matches
        if(fMasterCloud.width() != ps.width() || fMasterCloud.width() != fAnnotationCloud.width()) {
            std::cout << "Error: Width mismatch during cloud merging" << std::endl;
            return;
        }
        if(fMasterCloud.height() != fAnnotationCloud.height()) {
            std::cout << "Error: Height mismatch during cloud merging" << std::endl;
            return;
        }

        // Determine starting min and max Z value (= slice) from the master point cloud
        auto minZ = fMasterCloud[0][2];
        auto maxZ = fMasterCloud[fMasterCloud.size() - 1][2];

        // Determine new min and max Z values based on input point set
        auto minZPS = ps[0][2];
        auto maxZPS = ps[ps.size() - 1][2];

        // Handle point cloud logic
        int i;
        for (i= 0; i < fMasterCloud.height(); i++) {
            auto masterRowI = fMasterCloud.getRow(i);
            if (ps[0][2] <= masterRowI[fUpperPart.width()-1][2]){
                // We found the entry where the 3rd vector component (= index 2 = which means the slice index)
                // of the new point set matches the value in the existing row of our master point cloud 
                // => starting point for merge
                break;
            }
        }

        // Copy everything below the index for merge start that we just determined (copyRows will not return back the
        // the row of "i", so no duplicates with our to-be merged point set). If i reached the end of our point cloud
        // above, then there are no duplicates and will simply continue with the append below.
        fUpperPart = fMasterCloud.copyRows(0, i);
        fUpperPart.append(ps);

        // Check if remaining rows already exist in fMasterCloud behind the new point set
        for(; i < fMasterCloud.height(); i++) {
            auto masterRowI = fMasterCloud.getRow(i);
            if (ps[ps.size() - 1][2] < masterRowI[fUpperPart.width()-1][2]) {
                break;
            }
        }

        // Add the remaining rows (if there are any left; potentially all are left if the input
        // points all have lower slice index values than our existing master cloud contained so far)
        if (i < fMasterCloud.height()) {
            fUpperPart.append(fMasterCloud.copyRows(i, fMasterCloud.height()));
        }

        fMasterCloud = fUpperPart;

        // Handle annotation cloud logic
        volcart::OrderedPointSet<cv::Vec2i> fUpperAnnotations(fAnnotationCloud.width());
        const AnnotationStruct defaultAnnotation;

        if (minZPS < minZ) {
            // New anntotaaion points required at the start to match the new size of the master point cloud
            volcart::Segmentation::AnnotationSet as(fAnnotationCloud.width());
            std::vector<cv::Vec2i> annotations;

            for (int ia = 0; ia < (minZ - minZPS); ia++) {
                annotations.clear();
                for (int ja = 0; ja < ps.width(); ja++) {
                    // We have no annotation info for the new points, so just create initial rows and entries
                    annotations.emplace_back(defaultAnnotation.anchor, defaultAnnotation.manual);
                }
                as.pushRow(annotations);
            }
            fUpperAnnotations.append(as);
        }

        fUpperAnnotations.append(fAnnotationCloud.copyRows(minZ, maxZ + 1));

        if(maxZPS > maxZ) {
            volcart::Segmentation::AnnotationSet as(fAnnotationCloud.width());
            std::vector<cv::Vec2i> annotations;
            
            for (int ia = 0; ia < (maxZPS - maxZ); ia++) {
                annotations.clear();
                for (int ja = 0; ja < ps.width(); ja++) {
                    // We have no annotation info for the new points, so just create initial rows and entries
                    annotations.emplace_back(defaultAnnotation.anchor, defaultAnnotation.manual);
                }
                as.pushRow(annotations);
            }
            fUpperAnnotations.append(as);
        }

        fAnnotationCloud = fUpperAnnotations;
    }

    inline void MergeChangedCurveIntoPointCloud(int sliceIndex)
    {
        // Check if we have a buffered changed curve for this index. If not exit.
        auto it = fIntersectionsChanged.find(sliceIndex);
        if(it == fIntersectionsChanged.end())
            return;

        volcart::Segmentation::PointSet ps(fMasterCloud.width());
        cv::Vec3d tempPt;
        std::vector<cv::Vec3d> row;
        for (size_t i = 0; i < it->second.GetPointsNum(); ++i) {
            tempPt[0] = it->second.GetPoint(i)[0];
            tempPt[1] = it->second.GetPoint(i)[1];
            tempPt[2] = it->second.GetSliceIndex();
            row.push_back(tempPt);
        }
        ps.pushRow(row);
        
        MergePointSetIntoPointCloud(ps);
    }

    inline volcart::OrderedPointSet<cv::Vec2i> CreateInitialAnnotationSet(int height, int width)
    {
        fAnnotationCloud.reset();
        fAnnotationCloud = volcart::OrderedPointSet<cv::Vec2i>(width);
        const AnnotationStruct defaultAnnotation;

        std::vector<cv::Vec2i> annotations;

        for (int i = 0; i < height; i++) {
            annotations.clear();
            for (int j = 0; j < width; j++) {
                // We have no annotation info, so just create initial rows and entries
                annotations.emplace_back(defaultAnnotation.anchor, defaultAnnotation.manual);
            }
            fAnnotationCloud.pushRow(annotations);
        }

        return fAnnotationCloud;
    }

    inline void SetSliceAsAnchor(int sliceIndex, bool anchor)
    {
        // Calculate index via master point cloud
        int pointIndex;
        for (int i= 0; i < fMasterCloud.height(); i++) {
            auto masterRowI = fMasterCloud.getRow(i);
            if (sliceIndex <= masterRowI[0][2]){
                pointIndex = i * fMasterCloud.width();
                break;
            }
        }

        for(int i = pointIndex; i < (pointIndex + fAnnotationCloud.width()); i++) {
            fAnnotationCloud[i][0] = anchor;
        }

        auto it = fAnnotations.find(sliceIndex);
        if(it != fAnnotations.end()) {
            // Update existing entry
            it->second.anchor = anchor;
        } else {
            // Create new entry
            AnnotationStruct an;
            an.anchor = anchor;
            fAnnotations[sliceIndex] = an;
        }
    }

    inline bool IsSliceAnAnchor(int sliceIndex) {
        return fAnnotations[sliceIndex].anchor;
    }

    inline void AddPointsToManualBuffer(std::set<int> pointIndexes)
    {
        // We need to buffer the points that we potentially have to store as "manually changed" in
        // annotations, but we cannot directly update the cloud, since the manual changes might be
        // discarded, e.g. by leaving the segmentation tool. Only once they are "confirmed" by
        // being used ina  segmentation run, can we update the annotation cloud.
        fBufferedChangedPoints.insert(pointIndexes.begin(), pointIndexes.end());
    }

    inline void ConfirmPointsAsManual(int sliceIndex)
    {
        if(fBufferedChangedPoints.size() > 0) {
            for (auto index : fBufferedChangedPoints) {
                fAnnotationCloud[sliceIndex * fAnnotationCloud.width() - 1 + index][1] = true;
            }

            auto it = fAnnotations.find(sliceIndex);
            if(it != fAnnotations.end()) {
                it->second.manual = true;
            }     

            fBufferedChangedPoints.clear();
        }
    }

    inline int FindNearestLowerAnchor(int sliceIndex)
    {
        // From provided start slice go backwards until we have an anchor
        for(int i = sliceIndex - 1; i > fMinSegIndex; i--) {
            if(fAnnotations[i].anchor) {
                return i;
            }
        }

        // No anchor found
        return -1;
    }

    inline int FindNearestHigherAnchor(int sliceIndex)
    {
        // From provided start slice go forward until we have an anchor or reached the end
        for(int i = sliceIndex + 1; i < currentVolume->numSlices(); i++) {
            if(fAnnotations[i].anchor) {
                return i;
            }
        }

        // No anchor found
        return -1;
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

        // Buffer the changed path, so that if we change the displayed slice we do not loose 
        // the manual changes that were made to the points of the path
        fIntersectionsChanged[fPathOnSliceIndex] = fIntersectionCurve;
    }
};

}  // namespace ChaoVis