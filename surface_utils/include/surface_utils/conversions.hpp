//
// Created by will on 6/20/16.
//

#ifndef PROJECT_CONVERSIONS_HPP
#define PROJECT_CONVERSIONS_HPP

#include <pcl/point_types.h>

namespace surface_utils {


    pcl::PointXYZL toLabeledPoint(const pcl::PointXYZ &pt) {
        pcl::PointXYZL lpt;
        lpt.x = pt.x;
        lpt.y = pt.y;
        lpt.z = pt.z;
        return lpt;
    }

//    template <typename PointT>
//    surface_msgs::Surface surfaceToRos(const surfaces::Surface<PointT> &s) {
//            surface_msgs::Surface s_ros;
//
//            // ID, color, model
//            s_ros.id = s.id;
//            s_ros.color = s.color;
//            pcl_conversions::fromPCL(s.model, s_ros.model);
//
//            // Concave hull
//            pcl_conversions::fromPCL(s.concave_hull.header, s_ros.concave_hull.header);
//            pcl_conversions::fromPCL(s.concave_hull.cloud, s_ros.concave_hull.cloud);
//            s_ros.concave_hull.polygons.resize(s.concave_hull.polygons.size());
//            for (std::size_t i = 0; i < s.concave_hull.polygons.size(); i++) {
//                    pcl_conversions::fromPCL(s.concave_hull.polygons[i], s_ros.concave_hull.polygons[i]);
//            }
//
//            // Inliers
//            pcl::PCLPointCloud2 inliers;
//            pcl::toPCLPointCloud2<PointT>(s.inliers, inliers);
//            pcl_conversions::fromPCL(inliers, s_ros.inliers);
//
//            return s_ros;
//    }
//
//    surface_msgs::SurfaceMesh surfaceMeshToRos(const surfaces::SurfaceMesh &m) {
//            surface_msgs::SurfaceMesh m_ros;
//
//            m_ros.id = m.id;
//            m_ros.mesh = m.surface_mesh;
//
//            return m_ros;
//    }
}

#endif //PROJECT_CONVERSIONS_HPP
