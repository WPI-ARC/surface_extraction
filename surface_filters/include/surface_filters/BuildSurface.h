/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#ifndef SURFACE_FILTERS_CONCAVE_HULL_H_
#define SURFACE_FILTERS_CONCAVE_HULL_H_

// System / Boost
#include <mutex>
#include <unordered_map>
#include <boost/range/adaptors.hpp>

// PCL
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/distances.h>

// Messages
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <surface_filters/BuildSurfaceConfig.h>

// Surfaces
#include <surfaces/utils.hpp>
#include <surfaces/Segment.hpp>
#include <surfaces/SurfaceStamped.hpp>
#include <surfaces/SurfaceMeshStamped.hpp>
#include "surface_filters/Triangulate.h"

namespace vis = visualization_msgs;

namespace surface_filters {
    namespace sync_policies = message_filters::sync_policies;

    /** \brief @b MovingLeastSquares represents a nodelet using the MovingLeastSquares implementation.
    * The type of the output is the same as the input, it only smooths the XYZ coordinates according to the parameters.
    * Normals are estimated at each point as well and published on a separate topic.
    */
    class BuildSurface : public pcl_ros::PCLNodelet {
        // Point types
        typedef pcl::PointXYZ PointIn;
        typedef pcl::PointXYZ PointOut;

        // Point cloud types
        typedef pcl::PointCloud<PointIn> PointCloudIn;

        typedef pcl::PolygonMesh PolygonMesh;

        typedef surfaces::Segment<PointIn> Segment;
        typedef surfaces::SurfaceStamped<PointIn> SurfaceStamped;
        typedef surfaces::SurfaceMeshStamped SurfaceMeshStamped;

        // Message synchronizer types
        template<typename ...SubscribedTypes>
        using ExactTimeSynchronizer = message_filters::Synchronizer<sync_policies::ExactTime<SubscribedTypes...> >;
        template<typename ...SubscribedTypes>
        using ApproximateTimeSynchronizer = message_filters::Synchronizer<sync_policies::ApproximateTime<SubscribedTypes...> >;

    protected:
        pcl::SacModel model_type_ = pcl::SACMODEL_PLANE;

        /** \brief positive, non-zero value, defining the maximum length from a vertex to the facet center. */
        double alpha_ = 0.025;

        /** \brief Dimension of the hull, either 2 or 3. */
        int dimension_ = 2;

        /** \brief Pointer to a dynamic reconfigure service. */
        boost::shared_ptr<dynamic_reconfigure::Server<BuildSurfaceConfig>> srv_;

        /** \brief Dynamic reconfigure callback
          * \param config the config object
          * \param level the dynamic reconfigure level
          */
        void config_callback(BuildSurfaceConfig &config, uint32_t level);

    private:
        /** \brief Nodelet initialization routine. */
        virtual void onInit();

        /** \brief Synchronized input callback.
          * \param cloud the pointer to the input point cloud
          * \param indices the pointer to the input point cloud indices
          */
        void synchronized_input_callback(const Segment::ConstPtr &segment, bool is_new);

        SurfaceStamped::Ptr publish_surface(const Segment::ConstPtr &segment, bool is_new);

        SurfaceMeshStamped::Ptr publish_surface_mesh(const SurfaceStamped::ConstPtr &surface, bool is_new);

        PolygonMesh get_concave_hull(const pcl::ModelCoefficients &model, const PointCloudIn &cloud);

        void get_projected_cloud(const Segment::ConstPtr &segment, PointCloudIn &proj_output);

        void get_3d_mesh(const SurfaceStamped::ConstPtr &surface, shape_msgs::Mesh &output_trimesh);

        void get_triangulation_vertices(const pcl::ModelCoefficients &model, const PointCloudIn &hull_cloud,
                                        shape_msgs::Mesh &output_trimesh, std::vector<double> &points_2d) const;

        void get_triangulation_segments(const PolygonMesh &hull, const std::vector<double> &points_2d,
                                        std::vector<int> &segment_list, std::vector<double> &holes) const;

        void get_triangluation_trimesh(std::vector<double> &points_2d, std::vector<int> &segment_list,
                                       std::vector<double> &holes, const PolygonMesh &hull,
                                       shape_msgs::Mesh &output_trimesh) const;

        bool verify_hull(const PolygonMesh &hull, const pcl::ModelCoefficients &model) const;
        bool verify_hull_segment_lengths(const PolygonMesh &hull) const;
        bool verify_hull_unique_vertices(const PolygonMesh &hull) const;
        bool verify_hull_no_self_intersection(const PolygonMesh &hull, const pcl::ModelCoefficients &model) const;
        bool verify_hull_perimeter_holes(const PolygonMesh &hull) const;

    private:
        /** \brief The PCL implementation used. */
        pcl::ProjectInliers<PointIn> proj_;
        pcl::ConcaveHull<PointIn> hull_;

        /** \brief The output publishers */
        ros::Publisher pub_new_surface;
        ros::Publisher pub_new_surface_mesh;
        ros::Publisher pub_updated_surface;
        ros::Publisher pub_updated_surface_mesh;

        ros::Publisher pub_markers;

        /** \brief The input PointCloud subscriber (used when 'input' is the only required topic) */
        ros::Subscriber sub_create_surface_;
        ros::Subscriber sub_update_surface_;

        /** \brief Mutex for use with dynamic reconfigure */
        std::mutex hull_mutex_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif  //#ifndef SURFACE_FILTERS_CONCAVE_HULL_H_
