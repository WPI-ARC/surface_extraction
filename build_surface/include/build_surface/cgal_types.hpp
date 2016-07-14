//
// Created by will on 7/13/16.
//

#ifndef PROJECT_CGAL_TYPES_HPP
#define PROJECT_CGAL_TYPES_HPP

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

#include <CGAL/Polygon_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Polyline_simplification_2/Squared_distance_cost.h>


// Along with each point, store a PCL point index (uint32_t) and a visited boolean
struct CustomPointInfo {
    uint32_t pcl_index;
    uint32_t pcl_index2;
    bool visited;

    // For CGAL
    CustomPointInfo() : pcl_index(std::numeric_limits<uint32_t>::max()), pcl_index2(pcl_index), visited(false) {}
    // For me
    CustomPointInfo(uint32_t i) : pcl_index(i), pcl_index2(std::numeric_limits<uint32_t>::max()), visited(false) {}
    // For completeness
    CustomPointInfo(uint32_t i, uint32_t i2, bool v) : pcl_index(i), pcl_index2(i), visited(v) {}
};

struct CustomFaceInfo {
    CustomFaceInfo() : nesting_level(-1) {}
    int nesting_level;
    bool in_domain() { return nesting_level % 2 == 1; }
};

// Alias the fairly long polyline simplification namespace
namespace PS = CGAL::Polyline_simplification_2;

// Get a kernel (which AFAIK is in charge of geometric operations) and the relevant contained types
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_2 CGALPoint;
typedef K::Segment_2 CGALSegment;
// Convenience typedef for the type CGAL will create for each point
typedef std::pair<CGALPoint, CustomPointInfo> CustomPoint;
// Get a vertex base type that stores a 2D point and my custom info
typedef CGAL::Triangulation_vertex_base_with_info_2<CustomPointInfo, K> CustomVb;
// Get a vertex base type that works with Alpha shapes, and make it inherit from the type with my custom info
typedef CGAL::Alpha_shape_vertex_base_2<K, CustomVb> AlphaVb;

// Get the rest of the types required for alpha shapes
typedef CGAL::Alpha_shape_face_base_2<K> AlphaFb;
typedef CGAL::Triangulation_data_structure_2<AlphaVb, AlphaFb> AlphaTds;
typedef CGAL::Delaunay_triangulation_2<K, AlphaTds> Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2> Alpha_shape_2;

// Get the rest of the types required for simplification
typedef PS::Vertex_base_2<K> SimplificationVbb;
typedef CGAL::Triangulation_vertex_base_with_info_2<CustomPointInfo, K, SimplificationVbb> SimplificationVb;
typedef CGAL::Constrained_triangulation_face_base_2<K> SimplificationFbb;
typedef CGAL::Triangulation_face_base_with_info_2<CustomFaceInfo, K, SimplificationFbb> SimplificationFb;
typedef CGAL::Triangulation_data_structure_2<SimplificationVb, SimplificationFb> SimplificationTds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, SimplificationTds, CGAL::Exact_predicates_tag> CDT;
typedef CGAL::Constrained_triangulation_plus_2<CDT> CT;
typedef PS::Stop_above_cost_threshold SimplificationStop;
typedef PS::Squared_distance_cost SimplificationCost;


#endif //PROJECT_CGAL_TYPES_HPP
