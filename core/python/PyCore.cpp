#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_Volume(py::module&);

PYBIND11_MODULE(Core, m)
{
    // init types
    init_Volume(m);
}
