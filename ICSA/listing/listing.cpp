// cppimport
/*
<%
# Load the GHL default cppimport config
from graph_hook_library import cppimport
cfg.update(cppimport.default_cfg())
%>
*/

#include <ghl/ghl.h>
#include <ghl/ghl_isomorphisms.h>
#include <pybind11/pybind11.h>

class City : public ghl::BaseNode { // Extend BaseNode to represent a City
public:
  explicit City(int id, int population) : BaseNode(id), population_(population) {}
  int population_ = 100; // City population
};

using ImplementedIsomorphism =
    ghl::UndirectedIsomorphism<std::shared_ptr<ghl::BaseNode>, std::shared_ptr<ghl::PrimitiveEdge>>;

// Declare an Isomorphism that will filter cities under a certain population and modify them
class LanesIso : public ImplementedIsomorphism {
  using ImplementedIsomorphism::ImplementedIsomorphism;
  // Override vertex_comp_function hook to match cities with a population of <= 100
  VertexCompFunction vertex_comp_function() final {
    return [](const GraphType &, const GraphType &target_graph, const VertexDescriptor,
              const VertexDescriptor target_edge) {
      return std::dynamic_pointer_cast<City>(target_graph[target_edge])->population_ <= 100;
    };
  }
  // Modify matched cities by multiplying their population
  InPlaceUpdateFunction inplace_update_function() {
    return [](GraphType &graph, const GraphType &iso_graph, const IsoMap &isomorphism) {
      std::dynamic_pointer_cast<City>(graph[isomorphism.at(0)[0]])->population_ *= 100;
    };
  }
};

// Expose the extended class and isomorphism to Python
PYBIND11_MODULE(listing, m) {
  pybind11::module_::import("graph_hook_library.ghl_bindings");
  pybind11::class_<City, ghl::BaseNode, std::shared_ptr<City>>(m, "City")
      .def(pybind11::init<int, int>(), pybind11::arg("id"), pybind11::arg("population"))
      .def_readwrite("population", &City::population_);
  pybind11::class_<LanesIso, ImplementedIsomorphism, std::shared_ptr<LanesIso>>(m, "LanesIso")
      .def(pybind11::init<const ImplementedIsomorphism::GraphType &>(), pybind11::arg("pattern"));
}
