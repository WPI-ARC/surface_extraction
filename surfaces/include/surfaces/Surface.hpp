//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACE_HPP
#define SURFACE_MANAGER_SURFACE_HPP

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <surfaces/Polygons.hpp>

namespace surfaces {

// Defined similarly to pcl::PointIndices
    template <typename PointType>
    struct Surface
    {
        Surface() : id(), color(), concave_hull(), model()
        {}

        unsigned int id;
        std_msgs::ColorRGBA color;
        surfaces::Polygons concave_hull;
        
        pcl::ModelCoefficients model;

    public:
        typedef boost::shared_ptr< ::surfaces::Surface<PointType> > Ptr;
        typedef boost::shared_ptr< ::surfaces::Surface<PointType> const> ConstPtr;
    }; // struct Surface

    template <typename PointType>
    inline std::ostream& operator << (std::ostream& s, const ::surfaces::Surface<PointType> &v)
    {
        s << "id: " << std::endl;
        s << "  " << v.id;
        s << "color: " << std::endl;
        s << "  " << v.color.r << std::endl;
        s << "  " << v.color.g << std::endl;
        s << "  " << v.color.b << std::endl;
        s << "  " << v.color.a << std::endl;
        s << "concave_hull[]" << std::endl;
        s << "  " << v.concave_hull;
        s << "model[]" << std::endl;
        for (size_t i = 0; i < v.model.values.size(); ++i)
        {
            s << "  model[" << i << "]: ";
            s << "  " << v.model.values.at(i) << std::endl;
        }
        return (s);
    }
}

#endif //SURFACE_MANAGER_SURFACE_HPP
