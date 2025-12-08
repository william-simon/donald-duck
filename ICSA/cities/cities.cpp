// cppimport
/*
<%
from graph_hook_library import cppimport
cfg.update(cppimport.default_cfg())
cfg["dependencies"] = ["cities.h"]
%>
*/

#include "cities.h"
#include <algorithm>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ghl/bindings/graph_bindings.h>

namespace py = pybind11;

PYBIND11_MODULE(cities, m) {
  py::module_::import("graph_hook_library.ghl_bindings");

  py::class_<MapGraph, ghl::UndirectedExtendedGraph<VertexProperty, EdgeProperty>, std::shared_ptr<MapGraph>>(
      m, "MapGraph")
      .def(py::init<>());

  py::class_<City, ghl::BaseNode, std::shared_ptr<City>>(m, "City")
      .def(py::init<const std::string &, int, bool>(), py::arg("name"), py::arg("population") = 100,
           py::arg("has_post_office") = true)
      .def("name", &City::name)
      .def("population", &City::population)
      .def("has_post_office", &City::hasPostOffice);

  py::class_<HUB, ghl::BaseNode, std::shared_ptr<HUB>>(m, "HUB").def(py::init<>());

  py::class_<Road, ghl::PrimitiveEdge, std::shared_ptr<Road>>(m, "Road")
      .def(py::init<int>(), py::arg("num_lanes"))
      .def("num_lanes", &Road::numLanes);

  py::class_<InsertHub, ImplementedIsomorphism, std::shared_ptr<InsertHub>>(m, "InsertHub")
      .def(py::init<const MapGraph::GraphType &>());
}
