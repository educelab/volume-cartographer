#include <iostream>
#include <string>

#include "vc/core/landmarks/PlaneLandmark.hpp"
#include "vc/core/landmarks/PointLandmark.hpp"
#include "vc/core/landmarks/PolylineLandmark.hpp"
#include "vc/core/util/DateTime.hpp"

namespace vc = volcart;
namespace vcld = volcart::landmarks;
namespace fs = boost::filesystem;

using Ldm = vcld::VolumeLandmark;
using PlnLdm = vcld::PlaneLandmark;
using PtLdm = vcld::PointLandmark;
using PlyLdm = vcld::PolylineLandmark;

int main(int argc, char* argv[])
{
    if (argc == 3) {
        // test read function if we have two path arguments
        auto read = Ldm::Read(argv[1]);
        Ldm::Write(argv[2], read);
    } else if (argc == 2 || argc > 3) {
        std::cout << "Usage: " << argv[0] << " [input.ldm output.ldm]\n";
        return EXIT_FAILURE;
    }

    ///// Get metadata /////
    // Get landmark type
    std::cout << "Landmark Type: ";
    std::string type;
    std::getline(std::cin, type);

    // Get landmark path
    std::cout << "Path: ";
    boost::filesystem::path path;
    std::string pathTemp;
    std::getline(std::cin, pathTemp);
    path = pathTemp;

    // Get landmark name
    std::cout << "Name: ";
    std::string name;
    std::getline(std::cin, name);

    // Generate uuid
    auto uuid = volcart::DateTime();

    ///// Get landmark data /////
    double x, y, z;
    Ldm::Pointer ldm;
    // Handle point
    if (type == "point") {
        auto point = PtLdm::New(uuid, name);

        std::cout << "Provide (x, y, z) point: ";
        std::cin >> x >> y >> z;
        point->setPosition(x, y, z);

        ldm = std::static_pointer_cast<Ldm>(point);
    }

    // Handle polyline
    else if (type == "polyline") {
        auto polyline = PlyLdm::New(uuid, name);

        std::string done;
        do {
            std::cout << "Provide (x, y, z) point ";
            std::cin >> x >> y >> z;
            polyline->addPoint(x, y, z);

            std::cout << "type \"done\" when finished ";
            std::cin.ignore();
            std::getline(std::cin, done);
        } while (done != "done");

        ldm = std::static_pointer_cast<Ldm>(polyline);
    }

    // Handle plane
    else if (type == "plane") {
        auto plane = PlnLdm::New(uuid, name);

        std::cout << "Provide (x, y, z) center point ";
        std::cin >> x >> y >> z;
        plane->setCenter(x, y, z);

        std::cout << "Provide <x, y, z> normal vector ";
        std::cin >> x >> y >> z;
        plane->setNormal(x, y, z);

        ldm = std::static_pointer_cast<Ldm>(plane);
    }

    // Single write call using base class pointer
    Ldm::Write(path, ldm);

    return EXIT_SUCCESS;
}
