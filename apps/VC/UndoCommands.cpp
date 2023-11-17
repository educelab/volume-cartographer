// UndoCommands.cpp
// Philip Allgaier 2023 October

#include "UndoCommands.hpp"

using namespace ChaoVis;

PathChangeCommand::PathChangeCommand(CVolumeViewerWithCurve* viewer, SegmentationStruct* segStruct, const PathChangePointVector before, const PathChangePointVector after, QUndoCommand* parent)
    : QUndoCommand(parent), viewer(viewer), segStruct(segStruct), before(before), after(after)
{
}

void PathChangeCommand::undo()
{
    segStruct->UpdateChangedCurvePoints(segStruct->fPathOnSliceIndex, before);

    // Check if we have annotation flags we need to adjust
    std::set<int> noLongerManual;
    for (int i = 0; i < after.size(); i++) {
        if (before.at(i).manuallyChanged != after.at(i).manuallyChanged) {
            if (!before.at(i).manuallyChanged) {
                noLongerManual.insert(after.at(i).pointIndex);
            }
        }
    }

    segStruct->RemovePointsFromManualBuffer(noLongerManual);
    segStruct->SetCurrentCurve(segStruct->fPathOnSliceIndex);
    viewer->UpdateView();
}

void PathChangeCommand::redo()
{
    segStruct->UpdateChangedCurvePoints(segStruct->fPathOnSliceIndex, after);

    // Check if we have annotation flags we need to adjust
    std::set<int> nowManual;
    for (int i = 0; i < after.size(); i++) {
        if (before.at(i).manuallyChanged != after.at(i).manuallyChanged) {
            if (after.at(i).manuallyChanged) {
                nowManual.insert(after.at(i).pointIndex);
            }
        }
    }

    segStruct->AddPointsToManualBuffer(nowManual);
    segStruct->SetCurrentCurve(segStruct->fPathOnSliceIndex);
    viewer->UpdateView();
}