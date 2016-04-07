//
// Created by will on 3/30/16.
//

#ifndef SURFACE_FILTERS_TRIANGLE_H
#define SURFACE_FILTERS_TRIANGLE_H

#include <iterator>
#include <unordered_map>

#define BOOST_BIND_NO_PLACEHOLDERS
#include <boost/range/adaptors.hpp>
#include <boost/range/irange.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <surfaces/utils.hpp>
#include <boost/bind.hpp>

extern "C" {
#include <triangle.h>
}

namespace libtriangle {
    class AdjacencyList;
}

namespace libtriangle {
    using boost::adaptors::transform;
    using boost::adaptors::filter;
    namespace place = std::placeholders;

    template <typename T>
    struct ArrayHash {
        std::size_t operator()(const T &arr) const {
            std::size_t seed = 0;
            for (const auto &elem : arr) boost::hash_combine(seed, elem);
            return seed;
        };
    };

    class Triangulate {
    public:
        typedef int point_id;
        typedef int edge_id;
        typedef int triangle_id;

        typedef std::array<const double, 2> Point;
        typedef std::array<const point_id, 2> Edge;
        typedef std::array<const point_id, 3> Triangle;

        typedef ArrayHash<Point> PointHash;
        typedef ArrayHash<Edge> EdgeHash;
        typedef ArrayHash<Triangle> TriangleHash;

    private:
        const Eigen::Affine3d plane_tf_;

        struct triangulateio tri_in_;
        struct triangulateio tri_out_;

        struct {
            bool pointlist;
            bool pointmarkerlist;
            bool pointattributelist;
            bool trianglelist;
            bool neighborlist;
            bool segmentlist;
            bool segmentmarkerlist;
            bool edgelist;
            bool edgemarkerlist;
        } triangle_allocated_;

        std::vector<double> pointlist_vector_;
        std::vector<int> segmentlist_vector_;
        std::vector<int> segmentmarkerlist_vector_;
        std::vector<double> holelist_vector_;

    public:
        Triangulate(pcl::PointCloud<pcl::PointXYZ> cloud, Eigen::Affine3f plane_tf);

        ~Triangulate();

        void triangulate(const std::string &flags);

        double edge_length_sqr(const edge_id edge) const;
        double edge_length_sqr(const Edge edge) const;

        Eigen::Map<Eigen::Vector2d> get_point_eigen(const point_id id) const;

        Edge make_edge(const point_id id_a, const point_id id_b) const;

        double get_x(const point_id id) const;
        double get_y(const point_id id) const;

        Point get_point(const point_id id) const;
        Edge get_edge(const edge_id id) const;
        Triangle get_triangle(const triangle_id id) const;

        std::array<Edge, 3> triangle_edges(const Triangle &triangle) const;

        bool is_boundary_point(const point_id id) const;

        decltype(transform(boost::irange(0, 0), std::bind(&Triangulate::get_triangle, (const Triangulate*) 0, place::_1))) triangles() const;
        decltype(transform(boost::irange(0, 0), std::bind(&Triangulate::get_edge, (const Triangulate*) 0, place::_1))) edges() const;

        decltype(filter(boost::irange(0, 0), std::bind(&Triangulate::is_boundary_point, (const Triangulate*) 0, place::_1))) boundary_point_ids() const;

        decltype(boost::irange(0, 0)) edge_ids() const;

        int numberofinputpoints() const { return tri_in_.numberofpoints; }
        int numberofedges() const { return tri_out_.numberofedges; }
        int numberofsegments() const { return tri_out_.numberofsegments; }
        int numberofpoints() const { return tri_out_.numberofpoints; }
        int numberoftriangles() const { return tri_out_.numberoftriangles; }
        int numberofcorners() const { return tri_out_.numberofcorners; }
        int numberofholes() const { return tri_out_.numberofholes; }

        template <typename Iterator>
        bool point_in_polygon(const point_id test_point_i, Iterator polygon_begin, Iterator polygon_end) const;

        template <typename PointT>
        pcl::PointCloud<PointT> cloud_from_polygons(std::vector<pcl::Vertices> &polygons) const;

        template <typename PointT>
        pcl::PointCloud<PointT> cloud_from_point_ids(std::vector<point_id> &point_ids) const;
    };

    class AdjacencyList {
        typedef Triangulate::point_id point_id;
        typedef std::unordered_multimap<point_id, point_id> container;

        container adjacency_;

    public:
        typedef container::iterator iterator;
        typedef container::const_iterator const_iterator;
        typedef container::value_type value_type;

    public:
        void add(Triangulate::Edge edge) {
            adjacency_.insert({std::get<0>(edge), std::get<1>(edge)});
            adjacency_.insert({std::get<1>(edge), std::get<0>(edge)});
        }

        // Removes the directed edge pointed at by `it` and, if it exists, the reverse of the edge. Returns true if the
        // reverse existed (and therefore was removed), false otherwise
        bool remove(const_iterator it);

        std::pair<const_iterator, const_iterator> neighbors(const point_id &pt) const {
            return adjacency_.equal_range(pt);
        }

        std::pair<iterator, iterator> neighbors(point_id &pt) {
            return adjacency_.equal_range(pt);
        }

        bool empty() const { return adjacency_.empty(); }

        container::size_type size() const { return adjacency_.size(); }

        const_iterator begin() const { return adjacency_.begin(); }
        iterator begin() { return adjacency_.begin(); }

        const_iterator end() const { return adjacency_.end(); }
        iterator end() { return adjacency_.end(); }
    };

}

namespace std { // Potentially non-standard
    inline std::ostream& operator<< (std::ostream& os, const libtriangle::AdjacencyList::value_type& pair) {
        return os << "(" << std::get<0>(pair) << " => " << std::get<1>(pair) << ")";
    }

    inline std::ostream& operator<< (std::ostream& os, const libtriangle::AdjacencyList& adj_list) {
        for (const libtriangle::AdjacencyList::value_type &val : adj_list) {
            os << val << std::endl;
        }
        return os;
    }
}

template <typename Iterator>
bool libtriangle::Triangulate::point_in_polygon(const point_id test_point_i, Iterator polygon_begin, Iterator polygon_end) const {
    bool inside = false;

    const auto test_x = get_x(test_point_i);
    const auto test_y = get_y(test_point_i);

    // Make every polygon implicitly closed by ignoring the last element if it's not
    if (*polygon_begin == *std::prev(polygon_end)) std::advance(polygon_end, -1);

    // The point that's previous to the first point is the last point, since it's implicitly closed
    auto prev_x = get_x(*std::prev(polygon_end));
    auto prev_y = get_y(*std::prev(polygon_end));

    for (Iterator point_i = polygon_begin; point_i < polygon_end; ++point_i) {
        const auto next_x = get_x(*point_i);
        const auto next_y = get_y(*point_i);

        if ((((prev_y <= test_y) && (test_y < next_y)) || ((next_y <= test_y) && (test_y < prev_y))) &&
            (test_x < (next_x - prev_x) * (test_y - prev_y) / (next_y - prev_y) + prev_x)) {
            inside = !inside;
        }

        prev_x = next_x;
        prev_y = next_y;
    }

    return inside;
}



template <typename PointT>
pcl::PointCloud<PointT> libtriangle::Triangulate::cloud_from_polygons(std::vector<pcl::Vertices> &polygons) const {
    std::vector<long int> points_reindex_map(numberoftriangles(), -1); // -1 indicates no reindexing yet
    pcl::PointCloud<PointT> new_points;

    const auto t_inv = plane_tf_.inverse();

    // First, add every point in a polygon in order. This maximises cache-friendliness when iterating over polygons
    // (a common operation)
    for (auto &polygon : polygons) {
        // Might reserve too much (when some of the vertices in polygon have already been added), but this object
        // doesn't live very long so it doesn't affect much
        new_points.reserve(new_points.size() + polygon.vertices.size());
        for (auto &point_i : polygon.vertices) {
            if (points_reindex_map.at(point_i) == -1) {
                points_reindex_map.at(point_i) = new_points.size();

                Eigen::Vector3d point_2d(get_x(point_i), get_y(point_i), 0);
                Eigen::Vector3d point_3d = t_inv * point_2d;
                new_points.push_back({point_3d[0], point_3d[1], point_3d[2]});
            }
            point_i = static_cast<uint32_t>(points_reindex_map.at(point_i));
        }

    }

    // Then add any remaining boundary points in the triangulation
    for (const auto &point_i : boundary_point_ids()) {
        if (points_reindex_map.at(point_i) == -1) {
            points_reindex_map.at(point_i) = new_points.size();

            Eigen::Vector3d point_2d(get_x(point_i), get_y(point_i), 0);
            Eigen::Vector3d point_3d = t_inv * point_2d;
            new_points.push_back({point_3d[0], point_3d[1], point_3d[2]});
        }
    }

    return std::move(new_points);
}


template <typename PointT>
pcl::PointCloud<PointT> libtriangle::Triangulate::cloud_from_point_ids(std::vector<point_id> &point_ids) const {
    std::vector<long int> points_reindex_map(numberoftriangles(), -1); // -1 indicates no reindexing yet
    pcl::PointCloud<PointT> new_points;

    const auto t_inv = plane_tf_.inverse();

    new_points.reserve(point_ids.size());
    for (auto &point_id : point_ids) {
        if (points_reindex_map.at(point_id) == -1) {
            points_reindex_map.at(point_id) = new_points.size();

            Eigen::Vector3d point_2d(get_x(point_id), get_y(point_id), 0);
            Eigen::Vector3d point_3d = t_inv * point_2d;
            new_points.push_back({point_3d[0], point_3d[1], point_3d[2]});
        }
        point_id = static_cast<uint32_t>(points_reindex_map.at(point_id));
    }

    return std::move(new_points);
}

#endif //SURFACE_FILTERS_TRIANGLE_H
