#include <memory>

#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "localization_wrapper.h"

int main (int argc, char** argv) {
    // Set glog.
    FLAGS_colorlogtostderr = true;
    ROS_INFO("\033[1;32m----> ESKF has Started.\033[0m");
    // Initialize ros.
    ros::init(argc, argv, "imu_gps_localization");
    ros::NodeHandle nh;
    
    // Initialize localizer.
    LocalizationWrapper localizer(nh);

    ros::spin();
    return 0;
}