#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_Reslice(py::module&);
void init_Volume(py::module&);
void init_VolumePkg(py::module&);

PYBIND11_MODULE(Core, m)
{
    // init types
    init_Reslice(m);
    init_Volume(m);
    init_VolumePkg(m);
}
