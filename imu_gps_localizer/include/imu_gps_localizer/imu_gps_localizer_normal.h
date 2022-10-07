//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_ESKF_H
#define GPS_IMU_FUSION_ESKF_H

#include <eigen3/Eigen/Dense>

#include "imu_gps_localizer/base_type.h"
#include "imu_gps_localizer/initializer.h"

namespace ImuGpsLocalization {
class ESKF {
public:
    ESKF(Eigen::Vector3d I_p_Gps_);

    /*!
     * 用于ESKF滤波器的初始化，设置初始位姿，初始速度
     * @param curr_gps_data 与imu时间同步的gps数据
     * @param curr_imu_data 与gps时间同步的imu数据
     * @return
     */
    // bool Init(const GPSData &curr_gps_data, const IMUData &curr_imu_data);

    /*!
     * 滤波器的预测，对应卡尔曼滤波器的前两个公式
     * @param curr_imu_data
     * @return
     */
    bool Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state);

    /*!
     * 滤波器的矫正，对应卡尔曼滤波器的后三个公式
     * @param curr_gps_data
     * @return
     */
    bool Correct(const GpsPositionDataPtr gps_data_ptr, State* state);

    Eigen::Matrix4d GetPose() const;

    Eigen::Vector3d GetVelocity(){
        return velocity_;
    }

    // TODO
    // handle the imu data
    bool ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state);

    // handle the rtk data
    bool ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr);

private:
    void SetCovarianceQ(double gyro_noise_cov, double accel_noise_cov);

    void SetCovarianceR(double posi_noise_cov);

    void SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
                        double gyro_noise, double accel_noise);

    /*!
     * 通过IMU计算位姿和速度
     * @return
     */
    bool UpdateOdomEstimation(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state);

    bool UpdateErrorState(const double t, const Eigen::Vector3d &accel, State* state);

    bool ComputeAngularDelta(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, Eigen::Vector3d &angular_delta, const State* last_state);

    /*!
     * 计算地球转动给导航系带来的变换
     * @param R_nm_nm_1
     * @return
     */
    bool ComputeEarthTranform(const double delta_t, State* last_state, Eigen::Matrix3d &R_nm_nm_1);

    /*!
     * 通过IMU计算当前姿态
     * @param angular_delta
     * @param R_nm_nm_1
     * @param curr_R
     * @param last_R
     * @return
     */
    bool ComputeOrientation(const Eigen::Vector3d &angular_delta,
                            const Eigen::Matrix3d R_nm_nm_1,
                            State* state);

    bool ComputeVelocity(Eigen::Vector3d &curr_vel,
                         Eigen::Vector3d &last_vel,
                         const ImuDataPtr last_imu, 
                         const ImuDataPtr cur_imu,
                         State* state,
                         const State last_state);

    Eigen::Vector3d GetUnbiasAccel(const Eigen::Vector3d &accel);

    /*!
     * 通过imu计算当前位移
     * @param curr_vel
     * @param last_vel
     * @return
     */
    bool ComputePosition(const Eigen::Vector3d& curr_vel, const Eigen::Vector3d& last_vel, const double delta_t, State* state);

    /*!
     * 对误差进行滤波之后，需要在实际算出来的轨迹中，消除这部分误差
     */
    void EliminateError();

    /*!
     * 每次矫正之后，需要重置状态变量X
     */
    void ResetState();

private:
    static const unsigned int DIM_STATE = 15;
    static const unsigned int DIM_STATE_NOISE = 12;
    static const unsigned int DIM_MEASUREMENT = 3;
    static const unsigned int DIM_MEASUREMENT_NOISE = 3;

    static const unsigned int INDEX_STATE_POSI = 0;
    static const unsigned int INDEX_STATE_VEL = 3;
    static const unsigned int INDEX_STATE_ORI = 6;
    static const unsigned int INDEX_STATE_ACC_BIAS = 9;
    static const unsigned int INDEX_STATE_GYRO_BIAS = 12;
    static const unsigned int INDEX_MEASUREMENT_POSI = 0;

    typedef typename Eigen::Matrix<double, DIM_STATE, 1> TypeVectorX;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, 1> TypeVectorY;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixF;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> TypeMatrixB;
    typedef typename Eigen::Matrix<double, DIM_STATE_NOISE, DIM_STATE_NOISE> TypeMatrixQ;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixP;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT> TypeMatrixK;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_MEASUREMENT_NOISE> TypeMatrixC;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE> TypeMatrixG;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT> TypeMatrixR;

    TypeVectorX X_;
    TypeVectorY Y_;
    TypeMatrixF F_;
    TypeMatrixB B_;
    TypeMatrixQ Q_;
    TypeMatrixP P_;
    TypeMatrixK K_;
    TypeMatrixC C_;
    TypeMatrixG G_;
    TypeMatrixC R_;

    TypeMatrixF Ft_;

    Eigen::Vector3d init_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();

    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d g_;//重力加速度
    Eigen::Vector3d w_;//地球自传角速度

    // GPSData curr_gps_data_;

    double L_ = 0.0;//纬度

    // std::deque<IMUData> imu_data_buff_;

    // TODO 
    // add state
    State state_;
    bool initialized_;

    // initializer
    std::unique_ptr<Initializer> initializer_;

    // outside param
    Eigen::Vector3d I_p_Gps;

    // initial gpsdata
    Eigen::Vector3d init_lla;

    double cov_process_gyro;
    double cov_process_accel;
    double cov_process_accel_bias;
    double cov_process_gyro_bias;
    double cov_prior_posi;
    double cov_prior_vel;
    double cov_prior_ori;
    double cov_prior_epsilon;
    double cov_prior_delta;

public:
    void GetFGY(TypeMatrixF& F,TypeMatrixG& G, TypeVectorY & Y);
};
}

#endif //GPS_IMU_FUSION_ESKF_H