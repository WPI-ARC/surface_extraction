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

}

#endif //PROJECT_CONVERSIONS_HPP
