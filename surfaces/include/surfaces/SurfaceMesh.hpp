//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACE_HPP
#define SURFACE_MANAGER_SURFACE_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <surfaces/Polygons.hpp>

namespace surfaces {

// Defined similarly to pcl::PointIndices
    template <typename PointType>
    struct SurfaceMesh
    {
        SurfaceMesh() : cloud()
        {}

        unsigned int id;
        pcl::PointCloud<PointType> cloud;

    public:
        typedef boost::shared_ptr< ::surfaces::SurfaceMesh<PointType> > Ptr;
        typedef boost::shared_ptr< ::surfaces::SurfaceMesh<PointType> const> ConstPtr;
    }; // struct Surface

    template <typename PointType>
    inline std::ostream& operator << (std::ostream& s, const ::surfaces::SurfaceMesh<PointType> &v)
    {
        s << "id: " << std::endl;
        s << "  " << v.id;
        s << "cloud: " << std::endl;
        s << cloud;
        return (s);
    }
}

#endif //SURFACE_MANAGER_SURFACE_HPP
