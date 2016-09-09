//
// Created by will on 9/7/16.
//

#ifndef PROJECT_EIGEN_HELPERS_CONVERSIONS_HPP_H
#define PROJECT_EIGEN_HELPERS_CONVERSIONS_HPP_H

namespace EigenHelpersConversions {


    inline Eigen::Affine3f GeometryPoseToEigenAffine3f(const geometry_msgs::Pose& pose)
    {
        Eigen::Translation3f trans(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaternionf quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        Eigen::Affine3f eigen_pose = trans * quat;
        return eigen_pose;
    }

    inline Eigen::Vector3f GeometryVector3ToEigenVector3f(const geometry_msgs::Vector3& vector)
    {
        Eigen::Vector3f eigen_vector(vector.x, vector.y, vector.z);
        return eigen_vector;
    }
}

#endif //PROJECT_EIGEN_HELPERS_CONVERSIONS_HPP_H
