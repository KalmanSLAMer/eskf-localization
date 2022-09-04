#include "localization_wrapper.h"

#include <iomanip>

#include <glog/logging.h>

#include "imu_gps_localizer/base_type.h"

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);

    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    const Eigen::Vector3d I_p_Gps(x, y, z);

    // TODO
    // add localization mode and initmethod
    nh.param("Localization_mode", localizationMode, "ESKF_NORMAL");
    nh.param("InitMethod", initMethod, "OPENVINS");

    std::string log_folder = "/home";
    ros::param::get("log_folder", log_folder);

    // Log.
    file_state_.open(log_folder + "/state.csv");
    file_gps_.open(log_folder +"/gps.csv");

    // TODO
    // add param
    std::string work_space_path = WORK_SPACE_PATH;
    std::string config_file_path = work_space_path_ + "/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    imu_data_buff_.clear();
    gps_data_buff_.clear();

    // Initialization imu gps localizer.(ESKF_NORMAL)
    if(localizationMode == "ESKF_NORMAL"){
        imu_gps_localizer_ptr_ = 
            std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                                acc_bias_noise, gyro_bias_noise,
                                                                I_p_Gps);
    }
    else if(localizationMode == "ESKF_QK"){
    // Initialization imu gps localizer.(ESKF_QK)
        imu_gps_localizer_ptr_normal = 
            std::make_unique<ImuGpsLocalization::ESKF>(config_node);
    }

    // Subscribe topics.
    imu_sub_ = nh.subscribe("/imu/data", 10,  &LocalizationWrapper::ImuCallback, this);
    gps_position_sub_ = nh.subscribe("/fix", 10,  &LocalizationWrapper::GpsPositionCallback, this);

    state_pub_ = nh.advertise<nav_msgs::Path>("fused_path", 10);
}

LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    file_gps_.close();
}

void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;
    if(localizationMode == "ESKF_QK"){
        ImuGpsLocalization::State fused_state;
        const bool ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
        if (!ok) {
            return;
        }

        // Publish fused state.
        ConvertStateToRosTopic(fused_state);
        state_pub_.publish(ros_path_);

        // Log fused state.
        LogState(fused_state);
    }
    else if(localizationMode == "ESKF_NORMAL")  // TODO
    {
        IMUData imu_data;
        imu_data.time = imu_msg_ptr->header.stamp.toSec();
        imu_data.linear_accel.x() = imu_msg_ptr->linear_acceleration.x;
        imu_data.linear_accel.y() = imu_msg_ptr->linear_acceleration.y;
        imu_data.linear_accel.z() = imu_msg_ptr->linear_acceleration.z;

        imu_data.angle_velocity.x() = imu_msg_ptr->angular_velocity.x;
        imu_data.angle_velocity.y() = imu_msg_ptr->angular_velocity.y;
        imu_data.angle_velocity.z() = imu_msg_ptr->angular_velocity.z;
        imu_data_buff_.emplace_back(imu_data);
    }
}

void LocalizationWrapper::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
    // Check the gps_status.
    if (gps_msg_ptr->status.status != 2) {
        LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
        return;
    }

    if(localizationMode == "ESKF_QK"){
        ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
        gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
        gps_data_ptr->lla << gps_msg_ptr->latitude,
                            gps_msg_ptr->longitude,
                            gps_msg_ptr->altitude;
        gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

        imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);

        LogGps(gps_data_ptr);
    }
    else if(localizationMode == "ESKF_NORMAL"){
        GPSData gps_data;
        gps_data.time = gps_msg_ptr->header.stamp.toSec();
        gps_data.position_lla.x() = gps_msg_ptr->latitude;
        gps_data.position_lla.y() = gps_msg_ptr->longitude;
        gps_data.position_lla.z() = gps_msg_ptr->altitude;
        gps_data.cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());
        gps_data_buff_.emplace(gps_data)

        if(!InitFlag){
            if(ValidGPSAndIMUData()){ 
                imu_gps_localizer_ptr_normal->Init(imu_data_buff_.front(), gps_data_buff_.front());
                imu_data_buff_.pop_front();
                gps_data_buff_.pop_front();
                InitFlag = true;
            }
        }
        EskfNormalRun();
    }
}

// TODO
void LocalizationWrapper::EskfNormalRun()
{
    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()){
        curr_imu_data_ = imu_data_buff_.front();
        curr_gps_data_ = gps_data_buff_.front();
        if (curr_imu_data_.time < curr_gps_data_.time){
            eskf_ptr_->Predict(curr_imu_data_);
            imu_data_buff_.pop_front();
        } else{
            eskf_ptr_->Predict(curr_imu_data_);
            imu_data_buff_.pop_front();

            eskf_ptr_->Correct(curr_gps_data_);
            gps_data_buff_.pop_front();
        }
    }
}

bool LocalizationWrapper::ValidGPSAndIMUData() {
    IMUData curr_imu_data_ = imu_data_buff_.front();
    GPSData curr_gps_data_ = gps_data_buff_.front();

    double delta_time = curr_imu_data_.time - curr_gps_data_.time;

    if (delta_time > 0.01){
        gps_data_buff_.pop_front();
        return false;
    }

    if (delta_time < -0.01){
        imu_data_buff_.pop_front();
        return false;
    }

    return true;
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << G_q_I.x() << "," << G_q_I.y() << "," << G_q_I.z() << "," << G_q_I.w() << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data) {
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
}

void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = state.G_p_I[0];
    pose.pose.position.y = state.G_p_I[1];
    pose.pose.position.z = state.G_p_I[2];

    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    ros_path_.poses.push_back(pose);
}