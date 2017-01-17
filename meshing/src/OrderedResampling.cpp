//
// Created by Hannah Hatch on 7/25/16.
//

/* @file OrderedResampling.cpp*/

#include "meshing/OrderedResampling.h"
#include "meshing/CalculateNormals.h"

using namespace volcart::meshing;

//// Set Inputs/Get Output ////
void OrderedResampling::setMesh(
    const ITKMesh::Pointer& mesh, int inWidth, int inHeight)
{
    input_ = mesh;
    inWidth_ = inWidth;
    inHeight_ = inHeight;
}

volcart::ITKMesh::Pointer OrderedResampling::getOutputMesh() const
{
    if (output_.IsNull()) {
        std::cerr << "Error: Output Mesh is not set" << std::endl;
        return nullptr;
    } else {
        return output_;
    }
}

///// Processing /////
void OrderedResampling::compute()
{
    output_ = ITKMesh::New();

    // Dimensions of resampled, ordered mesh
    outWidth_ = (inWidth_ + 1) / 2;
    outHeight_ = (inHeight_ + 1) / 2;

    // Tells the loop whether or not the points in that line should be added to
    // the new mesh
    bool lineSkip = false;

    // Loop iterator
    int k = 0;
    // Adds certain points from old mesh into the new mesh
    for (auto pointsIt = input_->GetPoints()->Begin();
         pointsIt != input_->GetPoints()->End(); pointsIt++, k++) {

        // If we've reached the end of a row, reset k and flip the lineSkip
        // bool
        if (k == inWidth_) {
            k = 0;
            lineSkip = !lineSkip;
        }

        // Skip this point if we're skipping this line
        if (lineSkip) {
            continue;
        }

        // Only add every other point
        if (k % 2 == 0) {
            output_->SetPoint(output_->GetNumberOfPoints(), pointsIt.Value());
        }
    }

    // Something went wrong with resampling. Number of points aren't what we
    // expect...
    assert(
        static_cast<int>(output_->GetNumberOfPoints()) ==
            outWidth_ * outHeight_ &&
        "Error resampling. Output and expected output don't match.");

    // Vertices for each face in the new mesh
    uint32_t point1, point2, point3, point4;

    // Create two new faces each iteration based on new set of points and keeps
    // normals same as original
    for (int i = 0; i < outHeight_ - 1; i++) {
        for (int j = 0; j < outWidth_ - 1; j++) {

            // 4 points allows us to create the upper and lower faces at the
            // same time
            point1 = i * outWidth_ + j;
            point2 = point1 + 1;
            point3 = point2 + outWidth_;
            point4 = point3 - 1;

            if (point1 >= output_->GetNumberOfPoints() ||
                point2 >= output_->GetNumberOfPoints() ||
                point3 >= output_->GetNumberOfPoints() ||
                point4 >= output_->GetNumberOfPoints()) {
                throw std::out_of_range(
                    "Predicted vertex index for face generation out of range "
                    "of point set.");
            }

            // Add both these faces to the mesh
            add_cell_(point2, point3, point4);
            add_cell_(point1, point2, point4);
        }
    }

    volcart::meshing::CalculateNormals calcNorm(output_);
    calcNorm.compute();
    output_ = calcNorm.getMesh();

    std::cerr
        << "volcart::meshing::OrderedResampling: Points in resampled mesh "
        << output_->GetNumberOfPoints() << std::endl;
    std::cerr << "volcart::meshing::OrderedResampling: Cells in resampled mesh "
              << output_->GetNumberOfCells() << std::endl;
}

void OrderedResampling::add_cell_(uint32_t a, uint32_t b, uint32_t c)
{
    ITKCell::CellAutoPointer currentCell;

    currentCell.TakeOwnership(new ITKTriangle);

    currentCell->SetPointId(0, a);
    currentCell->SetPointId(1, b);
    currentCell->SetPointId(2, c);

    output_->SetCell(output_->GetNumberOfCells(), currentCell);
}
