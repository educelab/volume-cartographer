#include <vc/core/io/OBJWriter.hpp>
#include <vc/core/shapes/Spiral.hpp>

namespace vc = volcart;

int main()
{
    vc::io::OBJWriter writer;
    std::string stem = "spiral-";

    double a = 0.5;
    double aD = 0.1;
    double b = 0.1;
    double bD = 0.05;
    int i = 0;
    while (b < 1.0) {

        vc::shapes::Spiral spiral(20, 10, 55, 10, a, b);
        writer.setMesh(spiral.itkMesh());
        writer.setPath(stem + std::to_string(i) + ".obj");
        writer.write();

        i++;
        b += bD;
        a += aD;
    }
}
