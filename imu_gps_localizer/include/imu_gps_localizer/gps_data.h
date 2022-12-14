//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_GPS_DATA_H
#define GPS_IMU_FUSION_GPS_DATA_H

#include "GeographicLib/LocalCartesian.hpp"
#include <eigen3/Eigen/Core>

namespace ImuGpsLocalization {
class GPSData{
public:
    GPSData() = default;

    Eigen::Vector3d position_lla = Eigen::Vector3d::Zero();//LLA
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();//NED
    Eigen::Vector3d position_ned = Eigen::Vector3d::Zero();

    Eigen::Matrix<double, 15, 15> cov = Eigen::Matrix<double, 15, 15>::Zero();

    // Eigen::Vector3d true_velocity = Eigen::Vector3d::Zero();
    // Eigen::Vector3d true_position_lla = Eigen::Vector3d::Zero();
    double time = 0.0;
};
}
#endif //GPS_IMU_FUSION_GPS_DATA_H
