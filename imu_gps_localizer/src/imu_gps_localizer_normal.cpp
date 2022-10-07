//
// Created by meng on 2021/2/19.
//
#include "imu_gps_localizer/imu_gps_localizer_normal.h"
#include "imu_gps_localizer/utils.h"
#include "../3rd/sophus/se3.hpp"

namespace ImuGpsLocalization {
constexpr double kDegree2Radian = M_PI / 180.0;

Eigen::Matrix3d BuildSkewMatrix(const Eigen::Vector3d& vec){
    Eigen::Matrix3d matrix;
    matrix << 0.0,     -vec[2],   vec[1],
              vec[2],    0.0,     -vec[0],
              -vec[1],   vec[0],    0.0;

    return matrix;
}

ESKF::ESKF(Eigen::Vector3d I_p_Gps_) : initialized_(false) {
    // double gravity = node["earth"]["gravity"].as<double>();
    double gravity = 9.81007;
    // double earth_rotation_speed = node["earth"]["rotation_speed"].as<double>();
    double earth_rotation_speed = 7.272205216e-05;
    // double cov_prior_posi = node["covariance"]["prior"]["posi"].as<double>();
    // double cov_prior_vel = node["covariance"]["prior"]["vel"].as<double>();
    // double cov_prior_ori = node["covariance"]["prior"]["ori"].as<double>();
    // double cov_prior_epsilon = node["covariance"]["prior"]["epsilon"].as<double>();
    // double cov_prior_delta = node["covariance"]["prior"]["delta"].as<double>();
    // double cov_measurement_posi = node["covariance"]["measurement"]["posi"].as<double>();
    // double cov_process_gyro = node["covariance"]["process"]["gyro"].as<double>();
    // double cov_process_accel = node["covariance"]["process"]["accel"].as<double>();
    // L_ = node["earth"]["latitude"].as<double>();
    cov_prior_posi = 1e-4;
    cov_prior_vel = 1e-4;
    cov_prior_ori = 1e-6;
    cov_prior_epsilon = 1e-6;
    cov_prior_delta = 1e-6;

    cov_process_accel = 1e-1;
    cov_process_gyro = 1e-1;
    cov_process_accel_bias = 1e-3;
    cov_process_gyro_bias = 1e-3;

    L_ = 47;
    I_p_Gps = I_p_Gps_;
    g_ = Eigen::Vector3d(0.0, 0.0, -gravity);
    w_ = Eigen::Vector3d(0.0, earth_rotation_speed * cos(L_ * kDegree2Radian),
                         earth_rotation_speed * sin(L_ * kDegree2Radian));

    // SetCovarianceP(cov_prior_posi, cov_prior_vel, cov_prior_ori,
    //                cov_prior_epsilon, cov_prior_delta);
    // SetCovarianceR(cov_measurement_posi);
    // SetCovarianceQ(cov_process_gyro, cov_process_accel);

    initializer_ = std::make_unique<Initializer>(I_p_Gps);

    X_.setZero();
    F_.setZero();
    C_.setIdentity();
    G_.block<3,3>(INDEX_MEASUREMENT_POSI,INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();

    F_.block<3,3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity();
    F_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = BuildSkewMatrix(-w_);

    // 考虑bias的随机游走
    B_.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();
}

// TODO 
// handle the imu date
bool ESKF::ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state)
{
    if (!initialized_)
    {
        initializer_->AddImuData(imu_data_ptr);
        return false;
    }
    Predict(state_.imu_data_ptr, imu_data_ptr, &state_);
    // std::cout << "state2 imu: " << state_.G_p_I.x() << " " << state_.G_p_I.y() << " " << state_.G_p_I.z() << std::endl;
    ConvertENUToLLA(init_lla, state_.G_p_I, &(state_.lla));
    state_.timestamp = imu_data_ptr->timestamp;
    state_.imu_data_ptr = imu_data_ptr;
    *fused_state = state_;
    return true;
}

// handle the gps data
bool ESKF::ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr)
{
    if(!initialized_)
    {
        if(!initializer_->AddGpsPositionData(gps_data_ptr, &state_))
            return false;
        init_lla = gps_data_ptr->lla;
        state_.lla = gps_data_ptr->lla;
        initialized_ = true;
    }
    Correct(gps_data_ptr, &state_);
}

void ESKF::SetCovarianceQ(double gyro_noise, double accel_noise) {
    Q_.setZero();
    Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * cov_process_accel;
    Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * cov_process_gyro;
    Q_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * cov_process_accel_bias;
    Q_.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * cov_process_gyro_bias;
}

void ESKF::SetCovarianceR(double posi_noise) {
    R_.setZero();
    R_ = Eigen::Matrix3d::Identity() * posi_noise * posi_noise;
}

void ESKF::SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
                          double gyro_noise, double accel_noise) {
    state_.cov.block<3, 3>(0, 0) = posi_noise * Eigen::Matrix3d::Identity(); // position std: 10 m
    state_.cov.block<3, 3>(3, 3) = velo_noise * Eigen::Matrix3d::Identity(); // velocity std: 10 m/s
    // roll pitch std 10 degree.
    state_.cov.block<3, 3>(6, 6) = ori_noise * Eigen::Matrix3d::Identity();
    // state_.cov(8, 8)             = 100. * kDegreeToRadian * 100. * kDegreeToRadian; // yaw std: 100 degree.
    // Acc bias.
    state_.cov.block<3, 3>(9, 9) = accel_noise * Eigen::Matrix3d::Identity();
    // Gyro bias.
    state_.cov.block<3, 3>(12, 12) = gyro_noise * Eigen::Matrix3d::Identity();
}

// bool ESKF::Init(const GPSData &curr_gps_data, const IMUData &curr_imu_data) {
//     // 用gps的测得的速度初始化原始的速度
//     init_velocity_ = curr_gps_data.true_velocity;
//     velocity_ = init_velocity_;

//     // 初始位姿，已知，可能是自己定的
//     Eigen::Quaterniond Q = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
//                            Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
//                            Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());
//     init_pose_.block<3,3>(0,0) = Q.toRotationMatrix();
//     pose_ = init_pose_;

//     imu_data_buff_.clear();
//     imu_data_buff_.push_back(curr_imu_data);

//     curr_gps_data_ = curr_gps_data;

//     return true;
// }

// void ESKF::GetFGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y) {
//     F = Ft_;
//     G = G_;
//     Y = Y_;
// }

bool ESKF::Correct(const GpsPositionDataPtr gps_data_ptr, State* state) {
    const Eigen::Vector3d &G_p_I = state->G_p_I;
    const Eigen::Matrix3d &G_R_I = state->G_R_I;

    Eigen::Vector3d G_p_Gps;
    state->lla = gps_data_ptr->lla;
    ConvertLLAToENU(init_lla, gps_data_ptr->lla, &G_p_Gps);

    // curr_gps_data_ = curr_gps_data;

    // Y_ = pose_.block<3,1>(0,3) - curr_gps_data.position_ned;
    // std::cout << "state2 gps: " << G_p_Gps.x() << " " << G_p_Gps.y() << " " << G_p_Gps.z() << std::endl;
    Y_ = G_p_Gps - (G_p_I + G_R_I * I_p_Gps);

    const Eigen::Matrix3d V = gps_data_ptr->cov * 1e-4;
    
    const Eigen::MatrixXd P = state->cov;

    G_.block<3, 3>(INDEX_MEASUREMENT_POSI, INDEX_STATE_ORI) = -G_R_I * BuildSkewMatrix(I_p_Gps);

    // K_ = P_ * G_.transpose() * (G_ * P_ * G_.transpose() + C_ * R_ * C_.transpose()).inverse();

    K_ = P * G_.transpose() * (G_ * P * G_.transpose() + C_ * V * C_.transpose()).inverse();

    // state->cov = (TypeMatrixP::Identity() - K_ * G_) * P_;
    X_ = X_ + K_ * (Y_ - G_ * X_);

    const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K_ * G_;

    // * I_KH.transpose() + K_ * V * K_.transpose();
    state->cov = I_KH * P;

    EliminateError();

    ResetState();

    return true;
}

bool ESKF::Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state) {
    // imu_data_buff_.push_back(curr_imu_data);

    const double delta_t = cur_imu->timestamp - last_imu->timestamp;

    UpdateOdomEstimation(last_imu, cur_imu, state);

    // Eigen::Vector3d curr_accel = state->G_R_I
    //                              * cur_imu->acc;

    UpdateErrorState(delta_t, cur_imu->acc, state);
    // imu_data_buff_.pop_front();
}

// 这个未考虑bias的随机游走，也就是说Δbias(导数)=0
bool ESKF::UpdateErrorState(const double t, const Eigen::Vector3d &accel, State* state) {
    Eigen::Matrix3d F_23 = BuildSkewMatrix(state->G_R_I * (accel - state->acc_bias));

    F_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_ORI) = F_23;
    F_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = state->G_R_I;
    F_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_GYRO_BIAS) = -state->G_R_I;
    B_.block<3,3>(INDEX_STATE_VEL, 0) = state->G_R_I;
    B_.block<3,3>(INDEX_STATE_ORI, 3) = -state->G_R_I;

    TypeMatrixB Bk = B_ * t;
    TypeMatrixF Fk = TypeMatrixF::Identity() + F_ * t;
    // 这里的B阵考虑bias的随机游走了
    // TypeMatrixB Bk = B_ * t;

    Q_.setZero();
    Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * cov_process_accel;
    Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * cov_process_gyro;
    Q_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * cov_process_accel_bias;
    Q_.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * cov_process_gyro_bias;

    // Ft_ = F_ * t;

    // X_ = Fk * X_;
    // P_ = Fk * P_ * Fk.transpose() + Bk * Q_ * Bk.transpose();
    const Eigen::MatrixXd P = state->cov;
    state->cov = Fk * P * Fk.transpose() + Bk * Q_ * Bk.transpose();
    return true;
}

bool ESKF::UpdateOdomEstimation(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state) {

    const double delta_t = cur_imu->timestamp - last_imu->timestamp;
    // std::cout << "state2 imu predict delta_t: " << delta_t << std::endl;
    State last_state = *state;
    Eigen::Vector3d angular_delta;
    // std::cout << "state2 last_imu gyro data: " << last_imu->gyro[0] << " " << last_imu->gyro[1] << " " << last_imu->gyro[2] << std::endl;
    // std::cout << "state2 cur_imu gyro data: " << cur_imu->gyro[0] << " " << cur_imu->gyro[1] << " " << cur_imu->gyro[2] << std::endl;
    // std::cout << "state2 last_imu acc data: " << last_imu->acc[0] << " " << last_imu->acc[1] << " " << last_imu->acc[2] << std::endl;
    // std::cout << "state2 cur_imu acc data: " << cur_imu->acc[0] << " " << cur_imu->acc[1] << " " << cur_imu->acc[2] << std::endl;
    ComputeAngularDelta(last_imu, cur_imu, angular_delta, state);

    // std::cout << "state2 imu predict delta_angle_axis: " << angular_delta[0] << " " << angular_delta[1] << " " << angular_delta[2] << std::endl;
    Eigen::Matrix3d R_nm_nm_1 = Eigen::Matrix3d::Identity();
    // ComputeEarthTranform(delta_t, state, R_nm_nm_1);

    // Eigen::Matrix3d curr_R, last_R;
    ComputeOrientation(angular_delta, R_nm_nm_1, state);

    Eigen::Vector3d curr_vel, last_vel;
    ComputeVelocity(curr_vel, last_vel, last_imu, cur_imu, state, last_state);
    // std::cout << "state2 imu predict G_v_I: " << curr_vel[0] << " " << curr_vel[1] << " " << curr_vel[2] << std::endl;
    ComputePosition(curr_vel, last_vel, delta_t, state);
    // std::cout << "state2 imu predict G_p_I: " << state->G_p_I[0] << " " << state->G_p_I[1] << " " << state->G_p_I[2] << std::endl;
    return true;
}

bool ESKF::ComputeAngularDelta(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, Eigen::Vector3d &angular_delta, const State* last_state) {
    // IMUData curr_imu_data = imu_data_buff_.at(1);
    // IMUData last_imu_data = imu_data_buff_.at(0);

    const double delta_t = cur_imu->timestamp - last_imu->timestamp;

    if (delta_t <= 0){
        return false;
    }

    Eigen::Vector3d curr_angular_vel = cur_imu->gyro;

    Eigen::Vector3d last_angular_vel = last_imu->gyro;

    // Eigen::Vector3d curr_unbias_angular_vel = curr_angular_vel;
    // Eigen::Vector3d last_unbias_angular_vel = last_angular_vel;

    Eigen::Vector3d gyro = 0.5 * (curr_angular_vel + last_angular_vel) - last_state->gyro_bias;
    // std::cout << "state2 imu gyro: " << gyro[0] << " " << gyro[1] << " " << gyro[2] << std::endl;
    // std::cout << "state2 imu gyro bias: " << last_state->gyro_bias[0] << " " << last_state->gyro_bias[1] << " " << last_state->gyro_bias[2] << std::endl;
    angular_delta = (0.5 * (curr_angular_vel + last_angular_vel) - last_state->acc_bias) * delta_t;

    return true;
}

bool ESKF::ComputeEarthTranform(const double delta_t, State* last_state, Eigen::Matrix3d &R_nm_nm_1) {
    // IMUData curr_imu_data = imu_data_buff_.at(1);
    // IMUData last_imu_data = imu_data_buff_.at(0);

    // const double delta_t = cur_imu->timestamp - last_imu->timestamp;
    // 物体运动产生的角速度
    // 有点问题，这里需要GPS数据，怎么传进来？？？
    Eigen::Vector3d w_in_n;
    if (last_state->G_v_I.x() > 0 || last_state->G_v_I.y() > 0 || last_state->G_v_I.z() > 0)
    {
        constexpr double rm = 6353346.18315;
        constexpr double rn = 6384140.52699;
        Eigen::Vector3d w_en_n(-last_state->G_v_I[1] / (rm + last_state->lla[2]),
                            last_state->G_v_I[0] / (rn + last_state->lla[2]),
                            last_state->G_v_I[0] / (rn + last_state->lla[2])
                            * std::tan(last_state->lla[0] * kDegree2Radian));

        w_in_n = w_en_n + w_;
    }
    else
        w_in_n = w_;
    
    auto angular = delta_t * w_in_n;

    Eigen::AngleAxisd angle_axisd(angular.norm(), angular.normalized());

    R_nm_nm_1 = angle_axisd.toRotationMatrix().transpose();
}

bool ESKF::ComputeOrientation(const Eigen::Vector3d &angular_delta,
                              const Eigen::Matrix3d R_nm_nm_1,
                              State* state) {
    Eigen::AngleAxisd angle_axisd(angular_delta.norm(), angular_delta.normalized());
    // F_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::AngleAxisd(angular_delta.norm(), angular_delta.normalized()).toRotationMatrix().transpose();
    // last_R = pose_.block<3, 3>(0, 0);
    // // n系相对于i系有一个位姿的变换（旋转，因为地球自转），我们求得的是相对于n系的位姿
    // curr_R = R_nm_nm_1 * pose_.block<3, 3>(0, 0) * angle_axisd.toRotationMatrix();

    // pose_.block<3, 3>(0, 0) = curr_R;
    Eigen::Matrix3d last_R = state->G_R_I;
    state->G_R_I = R_nm_nm_1 * last_R * angle_axisd.toRotationMatrix();
    return true;
}

bool ESKF::ComputeVelocity(Eigen::Vector3d &curr_vel, Eigen::Vector3d &last_vel,
                         const ImuDataPtr last_imu, const ImuDataPtr cur_imu,
                         State* state, const State last_state) {
    // IMUData curr_imu_data = imu_data_buff_.at(1);
    // IMUData last_imu_data = imu_data_buff_.at(0);
    const double delta_t = cur_imu->timestamp - last_imu->timestamp;
    if (delta_t <=0 ){
        return false;
    }
    
        // 把IMU测到的加速度转到世界坐标系下
        // Eigen::Vector3d curr_accel = curr_imu_data.linear_accel;
        // Eigen::Vector3d curr_unbias_accel = GetUnbiasAccel(curr_R * curr_accel);

        // Eigen::Vector3d last_accel = last_imu_data.linear_accel;
        // Eigen::Vector3d last_unbias_accel = GetUnbiasAccel(last_R * last_accel);

        // last_vel = velocity_;

        // velocity_ += delta_t * 0.5 * (curr_unbias_accel + last_unbias_accel);
        // curr_vel = velocity_;

    Eigen::Vector3d curr_unbias_accel = state->G_R_I * (cur_imu->acc - state->acc_bias);
    Eigen::Vector3d last_unbias_accel = last_state.G_R_I * (last_imu->acc - last_state.acc_bias);

    last_vel = state->G_v_I;
    Eigen::Vector3d acc = 0.5 * (curr_unbias_accel + last_unbias_accel) + g_;
    // std::cout << "state2 imu acc: " << acc[0] << " " << acc[1] << " " << acc[2] << std::endl;
    // std::cout << "state2 imu acc bias: " << last_state.acc_bias[0] << " " << last_state.acc_bias[1] << " " << last_state.acc_bias[2] << std::endl;
    state->G_v_I = last_vel + (0.5 * (curr_unbias_accel + last_unbias_accel) + g_) * delta_t;
    curr_vel = state->G_v_I;

    return true;
}

Eigen::Vector3d ESKF::GetUnbiasAccel(const Eigen::Vector3d &accel) {
//    return accel - accel_bias_ + g_;
    return accel + g_;
}

bool ESKF::ComputePosition(const Eigen::Vector3d& curr_vel, const Eigen::Vector3d& last_vel, const double delta_t, State* state){
    // double delta_t = imu_data_buff_.at(1).time - imu_data_buff_.at(0).time;

    state->G_p_I += 0.5 * delta_t * (curr_vel + last_vel);

    return true;
}

void ESKF::ResetState() {
    X_.setZero();
}

void ESKF::EliminateError() {
    // pose_.block<3,1>(0,3) = pose_.block<3,1>(0,3) - X_.block<3,1>(INDEX_STATE_POSI, 0);

    // velocity_ = velocity_ - X_.block<3,1>(INDEX_STATE_VEL, 0);
    // Eigen::Matrix3d C_nn = Sophus::SO3d::exp(X_.block<3,1>(INDEX_STATE_ORI, 0)).matrix();
    // pose_.block<3,3>(0,0) = C_nn * pose_.block<3,3>(0,0);

    // gyro_bias_ = gyro_bias_ - X_.block<3,1>(INDEX_STATE_GYRO_BIAS, 0);
    // accel_bias_ = accel_bias_ - X_.block<3,1>(INDEX_STATE_ACC_BIAS, 0);

    state_.G_p_I     += X_.block<3,1>(INDEX_STATE_POSI, 0);
    state_.G_v_I     += X_.block<3,1>(INDEX_STATE_VEL, 0);
    state_.acc_bias  -= X_.block<3,1>(INDEX_STATE_ACC_BIAS, 0);
    state_.gyro_bias -= X_.block<3,1>(INDEX_STATE_GYRO_BIAS, 0);

    if(X_.block<3,1>(INDEX_STATE_ORI, 0).norm() > 1e-12)
    {
        state_.G_R_I *= Eigen::AngleAxisd(X_.block<3, 1>(INDEX_STATE_ORI, 0).norm(), X_.block<3, 1>(INDEX_STATE_ORI, 0).normalized()).toRotationMatrix();
    }
}

Eigen::Matrix4d ESKF::GetPose() const {
    return pose_;
}
}