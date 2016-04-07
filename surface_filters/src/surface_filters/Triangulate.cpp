//
// Created by will on 3/30/16.
//

#include <surface_filters/Triangulate.h>

libtriangle::Triangulate::Triangulate(pcl::PointCloud<pcl::PointXYZ> cloud, Eigen::Affine3f plane_tf)
        : plane_tf_(plane_tf.cast<double>()), tri_in_(), tri_out_(), triangle_allocated_(),
          pointlist_vector_(), segmentlist_vector_(), segmentmarkerlist_vector_(), holelist_vector_() {

    pointlist_vector_.reserve(cloud.size() * 2);
    for (const auto &point : cloud) {
        Eigen::Vector3d eigenpt(point.x, point.y, point.z);
        Eigen::Vector3d flat_pt = plane_tf_ * eigenpt;

        pointlist_vector_.push_back(flat_pt[0]);
        pointlist_vector_.push_back(flat_pt[1]);
    }
}

libtriangle::Triangulate::~Triangulate() {
    if (triangle_allocated_.pointlist && tri_out_.pointlist != NULL) {
        trifree(tri_out_.pointlist);
    }
    if (triangle_allocated_.pointattributelist && tri_out_.pointattributelist != NULL) {
        trifree(tri_out_.pointattributelist);
    }
    if (triangle_allocated_.pointmarkerlist && tri_out_.pointmarkerlist != NULL) {
        trifree(tri_out_.pointmarkerlist);
    }
    if (triangle_allocated_.trianglelist && tri_out_.trianglelist != NULL) {
        trifree(tri_out_.trianglelist);
    }
    if (triangle_allocated_.neighborlist && tri_out_.neighborlist != NULL) {
        trifree(tri_out_.neighborlist);
    }
    if (triangle_allocated_.segmentlist && tri_out_.segmentlist != NULL) {
        trifree(tri_out_.segmentlist);
    }
    if (triangle_allocated_.segmentmarkerlist && tri_out_.segmentmarkerlist != NULL) {
        trifree(tri_out_.segmentmarkerlist);
    }
    if (triangle_allocated_.edgelist && tri_out_.edgelist != NULL) {
        trifree(tri_out_.edgelist);
    }
    if (triangle_allocated_.edgemarkerlist && tri_out_.edgemarkerlist != NULL) {
        trifree(tri_out_.edgemarkerlist);
    }

}

void libtriangle::Triangulate::triangulate(const std::string &flags) {
    auto flag = [&flags](const std::string &test_flag) { return flags.find(test_flag) != std::string::npos; };
    // Initialize tri_in according to triangle.h:175
    tri_in_.pointlist = pointlist_vector_.data();
    tri_in_.numberofpoints = static_cast<int>(pointlist_vector_.size()) / 2;
    tri_in_.numberofpointattributes = 0; // Point attributes not supported
    tri_in_.pointmarkerlist = NULL; // Input point markers not supported
    assert(!flag("r")); // 'r' flag not supported
    assert(!flag("a")); // 'a' flag not supported
    if (flag("p")) {
        tri_in_.segmentlist = segmentlist_vector_.data();
        tri_in_.numberofsegments = static_cast<int>(segmentlist_vector_.size()) / 2;
        tri_in_.segmentmarkerlist = segmentlist_vector_.empty() ? NULL : segmentlist_vector_.data();
        
        if (!flag("r")) {
            tri_in_.holelist = holelist_vector_.data();
            tri_in_.numberofholes = static_cast<int>(holelist_vector_.size()) / 2;
            tri_in_.numberofregions = 0; // Regions not supported
        }
    }

    // Initialize tri_out according to triangle.h:206
    // NULL indicates Triangulate will initialize the memory
    if (!flag("N")) {
        tri_out_.pointlist = NULL;
        triangle_allocated_.pointlist = true;

        if (!flag("B")) { // 'B' was not used
            tri_out_.pointmarkerlist = NULL;
            triangle_allocated_.pointmarkerlist = true;
        }
    } else if (tri_in_.numberofpointattributes != 0) {
        tri_out_.pointattributelist = NULL;
        triangle_allocated_.pointattributelist = true;
    }
    if (!flag("E")) {
        tri_out_.trianglelist = NULL;
        triangle_allocated_.trianglelist = true;
    }
    if (flag("n")) {
        tri_out_.neighborlist = NULL;
        triangle_allocated_.neighborlist = true;
    }
    assert(tri_in_.numberoftriangleattributes== 0 && !flag("A")); // Triangulate attributes not supported
    if ((flag("p") || flag("c")) && !flag("P")) {
        tri_out_.segmentlist = NULL;
        triangle_allocated_.segmentlist = true;

        if (!flag("B")) {
            tri_out_.segmentmarkerlist = NULL;
            triangle_allocated_.segmentmarkerlist = true;
        }
    }
    if (flag("e")) {
        tri_out_.edgelist = NULL;
        triangle_allocated_.edgelist = true;

        if (!flag("B")) {
            tri_out_.edgemarkerlist = NULL;
            triangle_allocated_.edgemarkerlist = true;
        }
    }

    // Initialize tri_vorout according to triangle.h:226
    assert(!flag("v")); // Voroni output not supported

    // Convert const char* to char* by copying just to make sure nothing unexpected happens
    std::vector<char> flags_str(flags.begin(), flags.end());
    flags_str.push_back('\0');
    ::triangulate(flags_str.data(), &tri_in_, &tri_out_, NULL); // Needs the :: or becomes an incorrect recursive call
}

double libtriangle::Triangulate::edge_length_sqr(const Edge edge) const {
    const auto dx = get_x(std::get<0>(edge)) - get_x(std::get<1>(edge)); // x0 - x1
    const auto dy = get_y(std::get<0>(edge)) - get_y(std::get<1>(edge)); // y0 - y1
    return dx * dx + dy * dy;
}

auto libtriangle::Triangulate::make_edge(const point_id id_a, const point_id id_b) const -> Edge {
    return (id_a < id_b) ? Edge{id_a, id_b} : Edge{id_b, id_a};
}

double libtriangle::Triangulate::edge_length_sqr(const edge_id edge) const {
    return edge_length_sqr(get_edge(edge));
}

auto libtriangle::Triangulate::get_point(const point_id id) const -> Point {
    assert(tri_out_.pointlist != NULL && id < tri_out_.numberofpoints);
    return {tri_out_.pointlist[id * 2], tri_out_.pointlist[id * 2 + 1]};
}

double libtriangle::Triangulate::get_x(const point_id id) const {
    assert(tri_out_.pointlist != NULL && id < tri_out_.numberofpoints);
    return tri_out_.pointlist[id * 2];
}
double libtriangle::Triangulate::get_y(const point_id id) const {
    assert(tri_out_.pointlist != NULL && id < tri_out_.numberofpoints);
    return tri_out_.pointlist[id * 2 + 1];
}

Eigen::Map<Eigen::Vector2d> libtriangle::Triangulate::get_point_eigen(const point_id id) const {
    assert(tri_out_.pointlist != NULL && id < tri_out_.numberofpoints);
    return {tri_out_.pointlist + (id * 2)};
}

auto libtriangle::Triangulate::get_edge(const edge_id id) const -> Edge {
    assert(tri_out_.edgelist != NULL && id < tri_out_.numberofedges);
    return make_edge(tri_out_.edgelist[id * 2], tri_out_.edgelist[id * 2 + 1]);
}

auto libtriangle::Triangulate::get_triangle(const triangle_id id) const -> Triangle {
    assert(tri_out_.trianglelist != NULL && id < tri_out_.numberoftriangles);
    return {tri_out_.trianglelist[id * 3], tri_out_.trianglelist[id * 3 + 1], tri_out_.trianglelist[id * 3 + 2]};
}

auto libtriangle::Triangulate::triangle_edges(const Triangle &triangle) const -> std::array<Edge, 3> {
    return {{make_edge(std::get<0>(triangle), std::get<1>(triangle)),
                    make_edge(std::get<1>(triangle), std::get<2>(triangle)),
                    make_edge(std::get<2>(triangle), std::get<0>(triangle))}};
}


auto libtriangle::Triangulate::triangles() const -> decltype(transform(boost::irange(0, 0), std::bind(&Triangulate::get_triangle, (const Triangulate*) 0, place::_1))) {
    return transform(boost::irange(0, tri_out_.numberoftriangles), std::bind(&Triangulate::get_triangle, this, place::_1));
}

auto libtriangle::Triangulate::edges() const -> decltype(transform(boost::irange(0, 0), std::bind(&Triangulate::get_edge, (const Triangulate*) 0, place::_1))) {
    return transform(boost::irange(0, tri_out_.numberofedges), std::bind(&Triangulate::get_edge, this, place::_1));
}

auto libtriangle::Triangulate::boundary_point_ids() const -> decltype(filter(boost::irange(0, 0), std::bind(&Triangulate::is_boundary_point, (const Triangulate*) 0, place::_1))) {
    return filter(boost::irange(0, tri_out_.numberofpoints), std::bind(&Triangulate::is_boundary_point, this, place::_1));
}

auto libtriangle::Triangulate::edge_ids() const -> decltype(boost::irange(0, 0)) {
    return boost::irange(0, tri_out_.numberofedges);
}

bool libtriangle::Triangulate::is_boundary_point(const point_id id) const {
    return tri_out_.pointmarkerlist[id] == 1;
}

// Removes the directed edge pointed at by `it` and, if it exists, the reverse of the edge. Returns true if the
// reverse existed (and therefore was removed), false otherwise
bool libtriangle::AdjacencyList::remove(const_iterator it) {
    // Both <pt_a, pt_b> and <pt_b, pt_a> are elements.
    const Triangulate::point_id pt_a = it->first, pt_b = it->second;

    // Delete <pt_a, pt_b>.
    adjacency_.erase(it);  // Invalidates it

    // Find and delete <pt_b, pt_a>
    auto neighbors_it = this->neighbors(pt_b);
    auto backwards_edge = std::find(neighbors_it.first, neighbors_it.second, value_type(pt_b, pt_a));
    if (backwards_edge != neighbors_it.second) {
        adjacency_.erase(backwards_edge);
        return true;
    } else {
        return false;
    }
}
