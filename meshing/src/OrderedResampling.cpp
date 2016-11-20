//
// Created by Hannah Hatch on 7/25/16.
//

/* @file OrderedResampling.cpp*/

#include "meshing/OrderedResampling.h"
#include "meshing/CalculateNormals.h"

using namespace volcart::meshing;

//// Constructors ////
OrderedResampling::OrderedResampling() : _inWidth{0}, _inHeight{0} {}

OrderedResampling::OrderedResampling(
    ITKMesh::Pointer mesh, uint32_t in_width, uint32_t in_height)
    : _input{mesh}, _inWidth{in_width}, _inHeight{in_height}
{
}

//// Set Inputs/Get Output ////
void OrderedResampling::setMesh(
    ITKMesh::Pointer mesh, uint32_t in_width, uint32_t in_height)
{
    _input = mesh;
    _inWidth = in_width;
    _inHeight = in_height;
}

volcart::ITKMesh::Pointer OrderedResampling::getOutputMesh() const
{
    if (_output.IsNull()) {
        std::cerr << "Error: Output Mesh is not set" << std::endl;
        return NULL;
    } else
        return _output;
}

uint32_t OrderedResampling::getOutputWidth() const { return _outWidth; };
uint32_t OrderedResampling::getOutputHeight() const { return _outHeight; };

///// Processing /////
void OrderedResampling::compute()
{
    _output = ITKMesh::New();

    // Dimensions of resampled, ordered mesh
    _outWidth = (_inWidth + 1) / 2;
    _outHeight = (_inHeight + 1) / 2;

    // Tells the loop whether or not the points in that line should be added to
    // the new mesh
    bool line_skip = false;

    // Loop iterator
    uint32_t k = 0;
    ITKPointIterator pointsIterator = _input->GetPoints()->Begin();

    // Adds certain points from old mesh into the new mesh
    for (; pointsIterator != _input->GetPoints()->End();
         pointsIterator++, k++) {

        // If we've reached the end of a row, reset k and flip the line_skip
        // bool
        if (k == _inWidth) {
            k = 0;
            line_skip = !line_skip;
        }

        // Skip this point if we're skipping this line
        if (line_skip)
            continue;

        // Only add every other point
        if (k % 2 == 0)
            _output->SetPoint(
                _output->GetNumberOfPoints(), pointsIterator.Value());
    }

    // Something went wrong with resampling. Number of points aren't what we
    // expect...
    assert(
        _output->GetNumberOfPoints() == _outWidth * _outHeight &&
        "Error resampling. Output and expected output don't match.");

    // Vertices for each face in the new mesh
    uint32_t point1, point2, point3, point4;

    // Create two new faces each iteration based on new set of points and keeps
    // normals same as original
    for (uint32_t i = 0; i < _outHeight - 1; i++) {
        for (uint32_t j = 0; j < _outWidth - 1; j++) {

            // 4 points allows us to create the upper and lower faces at the
            // same time
            point1 = i * _outWidth + j;
            point2 = point1 + 1;
            point3 = point2 + _outWidth;
            point4 = point3 - 1;

            if (point1 >= _output->GetNumberOfPoints() ||
                point2 >= _output->GetNumberOfPoints() ||
                point3 >= _output->GetNumberOfPoints() ||
                point4 >= _output->GetNumberOfPoints()) {
                throw std::out_of_range(
                    "Predicted vertex index for face generation out of range "
                    "of point set.");
            }

            // Add both these faces to the mesh
            _addCell(point2, point3, point4);
            _addCell(point1, point2, point4);
        }
    }

    volcart::meshing::CalculateNormals calcNorm(_output);
    calcNorm.compute();
    _output = calcNorm.getMesh();

    std::cerr
        << "volcart::meshing::OrderedResampling: Points in resampled mesh "
        << _output->GetNumberOfPoints() << std::endl;
    std::cerr << "volcart::meshing::OrderedResampling: Cells in resampled mesh "
              << _output->GetNumberOfCells() << std::endl;
}

void OrderedResampling::_addCell(uint32_t a, uint32_t b, uint32_t c)
{
    ITKCell::CellAutoPointer current_C;

    current_C.TakeOwnership(new ITKTriangle);

    current_C->SetPointId(0, a);
    current_C->SetPointId(1, b);
    current_C->SetPointId(2, c);

    _output->SetCell(_output->GetNumberOfCells(), current_C);
}
