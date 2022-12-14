#pragma once

#include <fstream>
#include <memory>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include "imu_gps_localizer/imu_gps_localizer.h"
#include "imu_gps_localizer/imu_gps_localizer_normal.h"

// #include "imu_data.h"
// #include "gps_data.h"

class LocalizationWrapper {
public:
    LocalizationWrapper(ros::NodeHandle& nh);
    ~LocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);

    // TODO
    // void EskfNormalRun();
    // void ValidGPSAndIMUData();

private:
    void LogState(const ImuGpsLocalization::State& state);
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data);

    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state);
    void ConvertStateToRosTopic1(const ImuGpsLocalization::State& state);

    ros::Subscriber imu_sub_;
    ros::Subscriber gps_position_sub_;
    ros::Publisher state_pub_1;
    ros::Publisher state_pub_2;
    // load rate map
    ros::Publisher publisher_;

    std::ofstream file_state_;
    std::ofstream file_gps_;

    nav_msgs::Path ros_path_1;
    nav_msgs::Path ros_path_2;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;

    // TODO 
    std::string localizationMode, initMethod;
    std::unique_ptr<ImuGpsLocalization::ESKF> imu_gps_localizer_ptr_normal;
    bool gnss_inited;
    // std::deque<IMUData> imu_data_buff_;
    // std::deque<GPSData> gps_data_buff_;
    // IMUData curr_imu_data_;
    // GPSData curr_gps_data_;
};