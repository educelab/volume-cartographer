#include "vc/texturing/OrthographicProjectionFlattening.hpp"

#include <array>

#include <vtkOBBTree.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "vc/meshing/DeepCopy.hpp"
#include "vc/meshing/ITK2VTK.hpp"

using namespace volcart;
using namespace texturing;
namespace vcm = volcart::meshing;

OrthographicProjectionFlattening::Pointer
OrthographicProjectionFlattening::New()
{
    return std::make_shared<OrthographicProjectionFlattening>();
}

ITKMesh::Pointer OrthographicProjectionFlattening::compute()
{
    // Setup output
    output_ = ITKMesh::New();
    vcm::DeepCopy(mesh_, output_);

    // convert input mesh to vtkMesh
    vtkSmartPointer<vtkPolyData> vtkMesh = vtkSmartPointer<vtkPolyData>::New();
    vcm::ITK2VTK(mesh_, vtkMesh);
    cv::Vec3d origin, xAxis, yAxis, zAxis;

    // Computes the OBB and returns the 3 axes
    std::array<double, 3> size;
    auto obbTree = vtkSmartPointer<vtkOBBTree>::New();
    obbTree->ComputeOBB(
        vtkMesh, origin.val, xAxis.val, yAxis.val, zAxis.val, size.data());

    // Calculate UV positions
    // Largest two BB axes are the projection plane
    auto uLen = cv::norm(xAxis);
    auto vLen = cv::norm(yAxis);
    auto uVec = xAxis / uLen;
    auto vVec = yAxis / vLen;
    ITKPoint oldPt;
    ITKPoint newPt;
    for (size_t i = 0; i < mesh_->GetNumberOfPoints(); ++i) {
        // Get the original point
        mesh_->GetPoint(i, &oldPt);
        cv::Vec3d pt(oldPt.GetDataPointer());

        // Project point onto bb plane and calculate relative position
        newPt[0] = (pt - origin).dot(uVec);
        newPt[1] = 0;
        newPt[2] = (pt - origin).dot(vVec);

        output_->SetPoint(i, newPt);
    }

    return output_;
}
