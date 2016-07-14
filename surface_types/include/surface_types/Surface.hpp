//
// Created by will on 7/13/16.
//

#ifndef PROJECT_SURFACE_HPP
#define PROJECT_SURFACE_HPP

#include "SurfaceData.hpp"
#include <surface_utils/color_generator.hpp>
#include <boost/format.hpp>

class Surface {
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

    Surface(PointCloud inliers, pcl::ModelCoefficients model, Eigen::Affine3d pose)
        : data_(next_id(), next_color(), std::move(inliers), std::move(model), std::move(pose)),
          model_pose_upto_(data_.inliers.size()) {}

    std::size_t size() const { return data_.inliers.size(); }

    uint32_t id() const { return data_.id; }

    const std_msgs::ColorRGBA color() const { return data_.color; }

    const PointCloud inliers() const { return data_.inliers; }

    const pcl::ModelCoefficients model() const {
        assert(model_pose_upto_ == size() && "Tried to get out-of-date model (use model_approx if that's OK)");
        return data_.model;
    }

    const pcl::ModelCoefficients model_approx() const { return data_.model; }

    const Eigen::Affine3d pose() const {
        assert(model_pose_upto_ == size() && "Tried to get out-of-date pose (use pose_approx if that's OK)");
        return data_.pose;
    }

    const Eigen::Affine3d pose_approx() const { return data_.pose; }

    const Eigen::Affine3f pose_float() const {
        assert(model_pose_upto_ == size() && "Tried to get out-of-date pose (use pose_float_approx if that's OK)");
        return data_.pose.cast<float>();
    }

    const Eigen::Affine3f pose_float_approx() const { return data_.pose.cast<float>(); }

    const PointCloud boundary() const {
        assert(boundary_polygons_upto_ == size() &&
               "Tried to get out-of-date boundary (use boundary_approx if that's OK)");
        return data_.boundary;
    }

    const PointCloud boundary_approx() const { return data_.boundary; }

    const PointCloud boundary_or_inliers() const {
        return boundary_polygons_upto_ == size() ? data_.boundary : data_.inliers;
    }

    const std::vector<pcl::Vertices> polygons() const {
        assert(boundary_polygons_upto_ == size() &&
               "Tried to get out-of-date polygons (use polygons_approx if that's OK)");
        return data_.polygons;
    }

    const std::vector<pcl::Vertices> polygons_approx() const { return data_.polygons; }

    const shape_msgs::Mesh mesh() const {
        assert(mesh_upto_ == size() && "Tried to get out-of-date mesh (use mesh_approx if that's OK)");
        return data_.mesh;
    }

    const shape_msgs::Mesh mesh_approx() const { return data_.mesh; }

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
        model_pose_upto_ = size();
    }

    void update_shape(PointCloud boundary, std::vector<pcl::Vertices> polygons) {
        data_.boundary = std::move(boundary);
        data_.polygons = std::move(polygons);
        boundary_polygons_upto_ = size();
    }

    void update_shape(std::tuple<PointCloud, std::vector<pcl::Vertices>> tup) {
        data_.boundary = std::move(std::get<0>(tup));
        data_.polygons = std::move(std::get<1>(tup));
        boundary_polygons_upto_ = size();
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

    boost::format print_color() const {
        return boost::format("\x1b[38;2;%d;%d;%dm") % static_cast<int>(data_.color.r * 256) %
               static_cast<int>(data_.color.g * 256) % static_cast<int>(data_.color.b * 256);
    }

private:
    surface_types::SurfaceData data_;
    // <data_type>_upto_ indicates how big the inliers were when the <data_type> was last computed
    // Introduces a requirement that the inliers must never be reordered unless this is updated accordingly
    std::size_t model_pose_upto_ = 0;
    std::size_t boundary_polygons_upto_ = 0;
    std::size_t mesh_upto_ = 0;
};

#endif // PROJECT_SURFACE_HPP
