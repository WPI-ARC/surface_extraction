//
// Created by will on 7/13/16.
//

#ifndef PROJECT_SURFACE_HPP
#define PROJECT_SURFACE_HPP

#include "SurfaceData.hpp"
#include <surface_utils/color_generator.hpp>
#include <boost/format.hpp>

class Surface {
public:
    enum ProvideLevel { ALWAYS = 0, IF_AVAILABLE = 1, NEVER = 2 };

private:
    // Every explicitly-constructed surface object always gets a new id and color
    static uint32_t next_id() {
        static uint32_t id_gen = 0;

        return id_gen++;
    }

    static std_msgs::ColorRGBA next_color() {
        static random_colors::color_generator gen(0.5, 0.99);

        std_msgs::ColorRGBA color;
        color.a = 1;
        std::tie(color.r, color.g, color.b) = gen.rgb();
        return color;
    }

public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    ////////////////////////////// Constructor(s) //////////////////////////////
    Surface(PointCloud inliers, pcl::ModelCoefficients model, Eigen::Affine3d pose)
        : data_(next_id(), next_color(), std::move(inliers), std::move(model), std::move(pose)),
          plane_upto_(data_.inliers.size()) {}

    ////////////////////////////// Getters //////////////////////////////
    const uint32_t &id() const { return data_.id; }

    const std_msgs::ColorRGBA &color() const { return data_.color; }

    const PointCloud &inliers() const { return data_.inliers; }

    bool has_plane_approx() const { return plane_upto_ > 0; }
    bool has_plane() const { return plane_upto_ == size(); }

    const pcl::ModelCoefficients &model_approx() const {
        assert(has_plane_approx() && "Tried to get model, but it has never been provided");
        return data_.model;
    }
    const pcl::ModelCoefficients &model() const {
        assert(has_plane() && "Tried to get out-of-date model (use model_approx if that's OK)");
        return data_.model;
    }

    const Eigen::Affine3d &pose_approx() const {
        assert(has_plane_approx() && "Tried to get pose, but it has never been provided");
        return data_.pose;
    }
    const Eigen::Affine3d &pose() const {
        assert(has_plane() && "Tried to get out-of-date pose (use pose_approx if that's OK)");
        return data_.pose;
    }

    const Eigen::Affine3f pose_float_approx() const {
        assert(has_plane_approx() && "Tried to get pose, but it has never been provided");
        return data_.pose.cast<float>();
    }
    const Eigen::Affine3f pose_float() const {
        assert(has_plane() && "Tried to get out-of-date pose (use pose_float_approx if that's OK)");
        return data_.pose.cast<float>();
    }

    bool has_shape_approx() const { return shape_upto_ > 0; }
    bool has_shape() const { return shape_upto_ == size(); }

    const PointCloud &boundary_approx() const {
        assert(has_shape_approx() && "Tried to get boundary, but it has never been provided");
        return data_.boundary;
    }
    const PointCloud &boundary() const {
        assert(has_shape() && "Tried to get out-of-date boundary (use boundary_approx if that's OK)");
        return data_.boundary;
    }

    const PointCloud &boundary_or_inliers() const { return has_shape() ? boundary() : inliers(); }

    const std::vector<pcl::Vertices> &polygons_approx() const {
        assert(has_shape_approx() && "Tried to get polygons, but it has never been provided");
        return data_.polygons;
    }
    const std::vector<pcl::Vertices> &polygons() const {
        assert(has_shape() && "Tried to get out-of-date polygons (use polygons_approx if that's OK)");
        return data_.polygons;
    }

    bool has_mesh_approx() const { return mesh_upto_ > 0; }
    bool has_mesh() const { return mesh_upto_ == size(); }

    const shape_msgs::Mesh &mesh_approx() const {
        assert(has_mesh_approx() && "Tried to get mesh, but it has never been provided");
        return data_.mesh;
    }
    const shape_msgs::Mesh &mesh() const {
        assert(has_mesh() && "Tried to get out-of-date mesh (use mesh_approx if that's OK)");
        return data_.mesh;
    }

    surface_types::SurfaceData get_data(const bool get_inliers, const bool get_shape, const bool get_mesh) const {
        // Note: the caller is responsible for ensuring the requested data is actually available
        if (get_inliers && get_shape && get_mesh) {
            // If they want everything, it's easy
            assert(has_plane() && "Tried to get out-of-date plane in get_data");
            assert(has_shape() && "Tried to get out-of-date shape in get_data");
            assert(has_mesh() && "Tried to get out-of-date mesh in get_data");
            return data_;
        } else {
            // Otherwise, it's better to copy only what's needed
            surface_types::SurfaceData result;
            result.id = id();
            result.color = color();
            // No option to not provide the plane info
            result.model = model();
            result.pose = pose();
            // The rest is optoinal
            if (get_inliers) {
                result.inliers = inliers();
            }
            if (get_shape) {
                result.boundary = boundary();
                result.polygons = polygons();
            }
            if (get_mesh) {
                result.mesh = mesh();
            }

            return result;
        }
    }

    bool needs_cleanup() const { return cleanup_upto_ != size(); }

    const pcl::PointCloud<pcl::PointXYZL> &tiling() const {
        assert(!needs_cleanup() && "Tried to get out-of-date tiling");
        return tiling_;
    }

    ////////////////////////////// Mutators //////////////////////////////
    void add_inliers(const PointCloud &cloud) { data_.inliers += cloud; }

    void add_inliers(const PointCloud &cloud, const std::vector<int> &indices) {
        const auto prev_size = data_.inliers.size();
        // Due to implementation details, resize + assign is faster than reserve + push_back
        data_.inliers.resize(prev_size + indices.size());
        for (std::size_t i = 0; i < indices.size(); i++) {
            data_.inliers[prev_size + i] = cloud[indices[i]];
        }
    }

    void update_plane(pcl::ModelCoefficients model, Eigen::Affine3d pose) {
        data_.model = std::move(model);
        data_.pose = std::move(pose);
        plane_upto_ = size();
    }

    void update_plane(std::tuple<pcl::ModelCoefficients, Eigen::Affine3d> tup) {
        data_.model = std::move(std::get<0>(tup));
        data_.pose = std::move(std::get<1>(tup));
        plane_upto_ = size();
    }

    void update_shape(PointCloud boundary, std::vector<pcl::Vertices> polygons) {
        data_.boundary = std::move(boundary);
        data_.polygons = std::move(polygons);
        shape_upto_ = size();
    }

    void update_shape(std::tuple<PointCloud, std::vector<pcl::Vertices>> tup) {
        data_.boundary = std::move(std::get<0>(tup));
        data_.polygons = std::move(std::get<1>(tup));
        shape_upto_ = size();
    }

    void update_mesh(shape_msgs::Mesh mesh) {
        data_.mesh = std::move(mesh);
        mesh_upto_ = size();
    }

    void update(std::tuple<pcl::ModelCoefficients, Eigen::Affine3d, PointCloud, std::vector<pcl::Vertices>,
                           shape_msgs::Mesh> tup) {
        // i hope you like angle brackets
        update_plane(std::move(std::get<0>(tup)), std::move(std::get<1>(tup)));
        update_shape(std::move(std::get<2>(tup)), std::move(std::get<3>(tup)));
        update_mesh(std::move(std::get<4>(tup)));
    }

    void update_tiling(pcl::PointCloud<pcl::PointXYZL> tiling) {
        tiling_ = std::move(tiling);
        cleanup_upto_ = size();
    }

    ////////////////////////////// Other //////////////////////////////
    std::size_t size() const { return data_.inliers.size(); }

    boost::format print_color() const {
        return boost::format("\x1b[38;2;%d;%d;%dm") % static_cast<int>(data_.color.r * 256) %
               static_cast<int>(data_.color.g * 256) % static_cast<int>(data_.color.b * 256);
    }

private:
    surface_types::SurfaceData data_;
    pcl::PointCloud<pcl::PointXYZL> tiling_;
    // <data_type>_upto_ indicates how big the inliers were when the <data_type> was last computed
    // (or for cleanup, when the cleanup was last run on this surface)
    // Introduces a requirement that the inliers must never be reordered unless this is updated accordingly
    std::size_t plane_upto_ = 0;
    std::size_t shape_upto_ = 0;
    std::size_t mesh_upto_ = 0;
    std::size_t cleanup_upto_ = 0;
};

#endif // PROJECT_SURFACE_HPP
