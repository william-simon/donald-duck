// cppimport
/*
<%
from graph_hook_library import cppimport
cfg.update(cppimport.default_cfg())
cfg["dependencies"] = ["ghl_city.h"]
%>
*/

#include <memory>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ghl_city.h"
#include <ghl/bindings/graph_bindings.h>

namespace py = pybind11;

PYBIND11_MODULE(ghl_city, m) {
  py::module_::import("graph_hook_library.ghl_bindings");
  py::class_<MapGraph, ghl::DirectedExtendedGraph<VertexProperty, EdgeProperty>, std::shared_ptr<MapGraph>>(m,
                                                                                                            "MapGraph")
      .def(py::init<>());
  py::class_<City, ghl::BaseNode, std::shared_ptr<City>>(m, "City")
      .def(py::init<const std::string &, int, bool>(), py::arg("name"), py::arg("population"),
           py::arg("has_post_office"))
      .def("name", &City::name)
      .def("clone", &City::clone);
  py::class_<Road, ghl::PrimitiveEdge, std::shared_ptr<Road>>(m, "Road")
      .def(py::init<int>(), py::arg("num_lanes"))
      .def("numLanes", &Road::numLanes);
  py::class_<Capitals, ImplementedIsomorphism, std::shared_ptr<Capitals>>(m, "Capitals")
      .def(py::init<const MapGraph::GraphType &, std::set<std::string>>(), py::arg("iso_graph"), py::arg("capitals"));
}
