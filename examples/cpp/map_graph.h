/**
 * @file map_graph.h
 *
 * @brief Defines specialized graph type and visualization settings for map
 * examples.
 *
 * This file provides:
 * - Type definitions for a map-based graph using cities and roads
 * - Specialized vertex property writer for city visualization
 * - Specialized edge property writer for road visualization
 *
 * These specializations customize how cities and roads are displayed in
 * GraphViz output, making the resulting visualizations more informative and
 * domain-specific.
 */

#ifndef EXAMPLES_MAP_GRAPH_H
#define EXAMPLES_MAP_GRAPH_H

#include "city.h"
#include <ghl/ghl.h>
#include <memory>

// Type alias for vertex properties in the map graph
using VertexProperty = std::shared_ptr<ghl::BaseNode>;

// Type alias for edge properties in the map graph
using EdgeProperty = std::shared_ptr<ghl::PrimitiveEdge>;

/**
 * @class MapGraph
 * @brief ExtendedGraph specialization with custom DOT writers.
 */
class MapGraph : public ghl::DirectedExtendedGraph<VertexProperty, EdgeProperty> {
  using Base = ghl::DirectedExtendedGraph<VertexProperty, EdgeProperty>;

public:
  using Base::Base;

  void vertex_property_writer(std::ostream &out, const VertexDescriptor &v) const override {
    auto city = std::dynamic_pointer_cast<City>(this->graph()[v]);
    if (city) {
      out << std::format("[label=\"City: {}\nPopulation: {}\"]", city->name(), city->population());
    } else {
      out << std::format("[label=\"HUB \"]");
    }
  }

  void edge_property_writer(std::ostream &out, const EdgeDescriptor &e) const override {
    auto road = std::dynamic_pointer_cast<Road>(this->graph()[e]);
    out << std::format("[label=\"num lanes: {}\"]", road->numLanes());
  }
};

#endif