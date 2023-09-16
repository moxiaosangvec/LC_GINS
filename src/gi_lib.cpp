/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-07-21 21:50:29
 * @LastEditors: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @LastEditTime: 2023-09-14 01:06:54
 * @FilePath: /LC_GINS/src/lcgi_lib.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "gi_lib.h"
#include "gilib_interface.h"
#include <Eigen/Core>

/**
 * @description: 默认构造函数
 * @return {*}
 */
GILib::GILib()
{
    /* 完成数据初始化 */
    time_stamp_         = 0.0;

    imu_pre_.time_stamp = 0.0;
    imu_pre_.acc.setZero();
    imu_pre_.gyro.setZero();
    imu_cur_.time_stamp = 0.0;
    imu_cur_.acc.setZero();
    imu_cur_.gyro.setZero();

    gpos_.time_stamp = 0.0;
    gpos_.blh.setZero();
    gpos_.std.setZero();
    gpos_.sat_num = 0;
    gpos_.sol_type = SOLQ_NONE;

    memset(&nav_log_, 0, sizeof(navlog_t));

    fusion_nav_.time_stamp = 0;
    fusion_nav_.blh.setZero();
    fusion_nav_.pos.setZero();
    fusion_nav_.att.setZero();
    fusion_nav_.qbn.setIdentity();
    fusion_nav_.Tbn.setIdentity();
    fusion_nav_.vel.setZero();

    dr_nav_.blh.setZero();
    dr_nav_.pos.setZero();
    dr_nav_.vel.setZero();
    dr_nav_.att.setZero();
    dr_nav_.qbn.setIdentity();
    dr_nav_.Tbn.setIdentity();
    dr_nav_.time_stamp = 0.0;

    P_.resize(STATE_DIM_, STATE_DIM_);
    X_.resize(STATE_DIM_, 1);
}


status_t GILib::add_imu_data(void *imu_buf, int imu_size)
{
    gins_imu_t imu = { 0 };
    if (imu_size != sizeof(gins_imu_t) || imu_buf == nullptr) return GILIB_STATUS_FAIL;
    memcpy(&imu, imu_buf, sizeof(gins_imu_t));

    imu_.time_stamp = imu.time_stamp;
    for (size_t i = 0; i < 3; i++)
    {
        imu_.gyro[i] = imu.gyro[i] * DEG2RAD;
        imu_.acc[i] = imu.acc[i];
    }

    if (deque_imu_.size() >= IMU_DEQUE_NUM_)
    {
        deque_imu_.pop_front();
    }
    deque_imu_.push_back(imu_);

    return GILIB_STATUS_OK;
}

status_t GILib::add_gpos_data(void *gpos_buf, int gpos_size)
{
    gins_pos_t pos = { 0 };
    if (gpos_size != sizeof(gins_pos_t) || gpos_buf == nullptr) return GILIB_STATUS_FAIL;
    memcpy(&pos, gpos_buf, sizeof(gins_pos_t));
    
    /* 外部接口数据定位转换至内部的定义 */
    gpos_.time_stamp = pos.time_stamp;
    gpos_.blh[0] = pos.blh[0] * DEG2RAD;
    gpos_.blh[1] = pos.blh[1] * DEG2RAD;
    gpos_.blh[2] = pos.blh[2];
    gpos_.sol_type = (solqtype_t)pos.postype;    /* 目前内外定义是一致的，可以直接转 */
    gpos_.std[0] = pos.std[0];
    gpos_.std[1] = pos.std[1];
    gpos_.std[2] = pos.std[2];
    
    if (deque_gpos_.size() >= GPOS_DEQUE_NUM_)
    {
        deque_gpos_.pop_front();
    }
    deque_gpos_.push_back(gpos_);

    return GILIB_STATUS_OK;
}

status_t GILib::imu_compensate()
{
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    imu_cur_.gyro -= imu_err_.gyro_bias;
    imu_cur_.acc  -= imu_err_.acc_bias;
    
    Eigen::Vector3d gyrscale, accscale;
    gyrscale   = Eigen::Vector3d::Ones() + imu_err_.gyro_scale;
    accscale   = Eigen::Vector3d::Ones() + imu_err_.acc_scale;
    imu_cur_.gyro   = imu_cur_.gyro.cwiseProduct(gyrscale.cwiseInverse());
    imu_cur_.acc    = imu_cur_.acc.cwiseProduct(accscale.cwiseInverse());

    return GILIB_STATUS_OK;
}

status_t GILib::earth_data_compute(const double lat, const double height)
{
    double l = lat;     /* {rad} */
    double h = height;
    double sqr_sinlat = 0;
    
    sqr_sinlat  = sin(l) * sin(l);

    nav_log_.Rm = WGS84_RA * (1 - WGS84_SQR_E1) / (pow(1 - WGS84_SQR_E1 * sin(lat), 1.5));
    nav_log_.Rn = WGS84_RA / pow(1 - WGS84_SQR_E1 * sin(lat), 0.5);
    nav_log_.g  = 9.7803267715 * (1 + 0.0052790414 * sqr_sinlat + 
                  0.0000232718 * sqr_sinlat * sqr_sinlat) +
                  h * (0.0000000043977311 * sqr_sinlat - 0.0000030876910891) + 
                  0.0000000000007211 * h * h;
    nav_log_.wien   = { WGS84_WIE * cos(lat), 0, -WGS84_WIE * sin(lat) };

    return GILIB_STATUS_OK;
}


/**
 * @description: IMU机械编排
 * @return {*}
 */
status_t GILib::imu_mech()
{
    status_t ret = GILIB_STATUS_NONE;
    double dt = 0.0;
    double Rm = 0.0;
    double Rn = 0.0;
    double H = 0.0;
    double g = 0.0;
    Eigen::Vector3d pos_ned         = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel         = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_mid     = Eigen::Vector3d::Zero();
    Eigen::Vector3d blh             = Eigen::Vector3d::Zero();
    Eigen::Vector3d blh_mid         = Eigen::Vector3d::Zero();
    Eigen::Matrix4d Tbn             = Eigen::Matrix4d::Zero();
    Eigen::Quaterniond qbn          = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond dqbb         = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond dqnn         = Eigen::Quaterniond::Identity();
    Eigen::Vector3d phib            = Eigen::Vector3d::Zero();          /* b系下的等效旋转矢量 */
    Eigen::Vector3d phin            = Eigen::Vector3d::Zero();          /* n系下的等效旋转矢量 */
    Eigen::Vector3d dtheta_pre      = Eigen::Vector3d::Zero();
    Eigen::Vector3d dtheta_cur      = Eigen::Vector3d::Zero();
    Eigen::Matrix3d I3              = Eigen::Matrix3d::Identity();

    Eigen::Vector3d dv_gn = Eigen::Vector3d::Zero();
    Eigen::Vector3d dv_fn = Eigen::Vector3d::Zero();
    Eigen::Vector3d dv_fb = Eigen::Vector3d::Zero();
    Eigen::Vector3d dvcur = Eigen::Vector3d::Zero();
    Eigen::Vector3d dvpre = Eigen::Vector3d::Zero();
    Eigen::Vector3d g_pn  = Eigen::Vector3d::Zero();                    /* 当地重力加速度 */
    Eigen::Vector3d w_ien = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_enn = Eigen::Vector3d::Zero();


    /* 0. 数据准备 */
    blh = fusion_nav_.blh;
    vel = fusion_nav_.vel;
    qbn = fusion_nav_.qbn;
    Tbn = fusion_nav_.Tbn;
    Rm = nav_log_.Rm;
    Rn = nav_log_.Rn;

    w_enn(0) = vel(1) / (Rn + H);
    w_enn(1) = -vel(0) / (Rm + H);
    w_enn(2) = -vel(1) * tan(blh(0)) / (Rn + H);

    g_pn = { 0, 0, nav_log_.g };

    /* 1. IMU姿态递推 */
    qbn = fusion_nav_.qbn;
    dt = imu_cur_.time_stamp - imu_pre_.time_stamp; 
    dtheta_cur = imu_cur_.gyro * dt;
    dtheta_pre = imu_pre_.gyro * dt;
    dvcur      = imu_cur_.acc * dt;
    dvpre      = imu_pre_.acc * dt;
    phib = dtheta_cur + 1 / 12 * dtheta_pre.cross(dtheta_cur);
    // dqbb = rot_.rotvec2quaternion(phib);
    dqbb.w() = cos(0.5 * phib.norm());
    dqbb.vec() = sin(0.5 * phib.norm()) / (0.5 * phib.norm()) * (0.5 * phib);
    phin = (w_ien + w_enn) * dt;
    // dqnn = rot_.rotvec2quaternion(phin);
    dqnn.w() = cos(0.5 * phin.norm());
    dqnn.vec() = -sin(0.5 * phin.norm()) / (0.5 * phin.norm()) * (0.5 * phin);
    qbn = dqnn * qbn * dqbb; 
    // qbn.normalized();

    /* 2. IMU速度递推 */
    dv_gn = (g_pn - (2 * w_ien + w_enn).cross(vel)) * dt;
    dv_fb = dvcur + 0.5 * dtheta_cur.cross(dvcur) + 
            1 / 12 * (dtheta_pre.cross(dvcur) + dvcur.cross(dtheta_cur));
    dv_fn = (I3 - 0.5 * rot_.vec2skewsmt(phin)) * Tbn.block<3, 3>(0, 0) * dv_fb;
    vel += dv_fn + dv_gn;

    /* 3. IMU位置递推 */
    vel_mid = 0.5 * (vel + fusion_nav_.vel);
    blh(2) += -vel_mid(2) * dt;
    blh_mid(2) = 0.5 * (blh(2) + fusion_nav_.blh(2));
    blh(0) = blh(0) + vel_mid(0) / (Rm + blh(2)) * dt;
    blh_mid(0) = 0.5 * (blh(0) + fusion_nav_.blh(0));
    blh(1) = blh(1) + vel_mid(1) / ((Rn + blh(2)) * cos(blh(0))) * dt;

    /* 4. 将更新结果写入到状态变量 */
    fusion_nav_.time_stamp = imu_cur_.time_stamp;
    fusion_nav_.qbn = qbn;
    fusion_nav_.Tbn.block<3, 3>(0, 0) = qbn.toRotationMatrix();
    fusion_nav_.att = rot_.quaternion2euler(qbn);
    fusion_nav_.vel = vel;
    fusion_nav_.blh = blh;
    return ret;
}


status_t GILib::process_imu(void *inbuf, int insize)
{
    /* 变量定义 */
    Eigen::MatrixXd F;      /* 连续方程误差状态矩阵 */
    Eigen::MatrixXd Phi;    /* 离散化的状态转移矩阵 */
    Eigen::MatrixXd G;      /* 噪声驱动矩阵 */
    Eigen::MatrixXd Q;      /* 输入噪声阵 */
    Eigen::MatrixXd Qk;     /* 系统噪声矩阵 */
    Eigen::Vector3d fb      = Eigen::Vector3d::Zero();
    Eigen::Vector3d wibb    = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_ien   = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_enn   = Eigen::Vector3d::Zero();
    Eigen::Matrix3d temp    = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Cbn     = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d I       = Eigen::Matrix3d::Zero();
    double Rm = 0, Rn = 0;
    double g = 0;
    double lat = 0, lon = 0, h = 0;
    double vn = 0, ve = 0, vd = 0;
    double we = 0;
    double Tgb = 0, Tab = 0, Tgs = 0, Tas = 0;
    double vrw = 0, arw = 0;
    double sigma_gb = 0, sigma_ab = 0, sigma_gs = 0, sigma_as = 0;
    double dt = 0;

    /* 增加IMU数据 */
    add_imu_data(inbuf, insize);
    imu_pre_ = imu_cur_;
    imu_cur_ = imu_;

    /* 矩阵分配内存 */
    F.resize(STATE_DIM_, STATE_DIM_);
    Phi.resize(STATE_DIM_, STATE_DIM_);
    G.resize(STATE_DIM_, NOISE_DIM_);    
    Q.resize(NOISE_DIM_, NOISE_DIM_);
    Qk.resize(STATE_DIM_, STATE_DIM_);

    /* 变量赋值 */
    lat     = fusion_nav_.blh(0);
    lon     = fusion_nav_.blh(1);
    h       = fusion_nav_.blh(2);
    vn      = fusion_nav_.vel(0);
    ve      = fusion_nav_.vel(1);
    vd      = fusion_nav_.vel(2);
    Cbn     = fusion_nav_.Tbn.block<3, 3>(0, 0);
    fb      = imu_cur_.acc;
    wibb    = imu_cur_.gyro;
    Tgb     = cfg_.Tgb;
    Tab     = cfg_.Tab;
    Tgs     = cfg_.Tgs;
    Tas     = cfg_.Tas;
    I       = Eigen::Matrix3d::Identity();
    dt      = imu_cur_.time_stamp - imu_pre_.time_stamp;

    /* 地球数据计算 */
    earth_data_compute(lat, h);
    Rm       = nav_log_.Rm; 
    Rn       = nav_log_.Rn; 
    g        = nav_log_.g;
    w_ien    = nav_log_.wien;

    /* IMU数据补偿 */
    imu_compensate();

    /* IMU机械编排,递推后即更新了fuison_nav_ */
    imu_mech();

    w_enn(0) = ve / (Rn + h);
    w_enn(1) = -vn / (Rm + h);
    w_enn(2) = -ve * tan(lat) / (Rn + h);

    /* 矩阵计算 */
    /* 1. F阵 */
    temp(0, 0) = -vn / (Rm + h);
    temp(0, 1) = 0;
    temp(0, 2) = vn / (Rm + h);
    temp(1, 0) = ve * tan(lat) / (Rn + h);
    temp(1, 1) = -(vd + vn * tan(lat)) / (Rn + h);
    temp(1, 2) = ve / (Rn + h);
    temp(2, 0) = 0;
    temp(2, 1) = 0;
    temp(2, 2) = 0;
    F.block<3, 3>(0, 0) = temp;

    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    temp(0, 0) = -2 * ve * WGS84_WIE * cos(lat) / (Rm + h) - ve * ve / (cos(lat) * cos(lat)) / ((Rm + h) * (Rn + h));
    temp(0, 1) = 0;
    temp(0, 2) = vn * vd / ((Rm + h) * (Rm * h)) - ve * ve * tan(lat) / ((Rn + h) * (Rn + h));
    temp(1, 0) = 2 * WGS84_WIE * (vn * cos(lat) - vd * sin(lat)) / (Rm + h) + vn * ve / (cos(lat) * cos(lat)) / ((Rm + h) * (Rn + h));
    temp(1, 1) = 0;
    temp(1, 2) = (ve * vd + vn * ve * tan(lat)) / ((Rn + h) * (Rn + h));
    temp(2, 0) = 2 * WGS84_WIE * ve * sin(lat) / (Rm + h);
    temp(2, 1) = 0;
    temp(2, 2) = -ve * ve / ((Rn + h) * (Rn + h)) - vn * vn / ((Rm + h) * (Rm + h)) + 2 * g / (sqrt(Rm * Rn) + h);
    F.block<3, 3>(3, 0) = temp;

    temp(0, 0) = vd / (Rm + h);
    temp(0, 1) = -2 * (WGS84_WIE * sin(lat) + ve * tan(lat) / (Rn + h));
    temp(0, 2) = vn / (Rm + h);
    temp(1, 0) = 2 * WGS84_WIE * sin(lat) + ve * tan(lat) / (Rn + h);
    temp(1, 1) = (vd + vn * tan(lat)) / (Rn + h);
    temp(1, 2) = 2 * WGS84_WIE * cos(lat) + ve / (Rn + h);
    temp(2, 0) = -2 * vn / (Rm + h);
    temp(2, 1) = -2 * (WGS84_WIE * cos(lat) + ve / (Rn + h));
    temp(2, 2) = 0;
    F.block<3, 3>(3, 3) = temp;
    
    F.block<3, 3>(3, 6) = rot_.vec2skewsmt(Cbn * fb);
    F.block<3, 3>(3, 12) = Cbn;
    F.block<3, 3>(3, 18) = -Cbn * fb.asDiagonal();

    temp(0, 0) = -WGS84_WIE * sin(lat) / (Rm + h);
    temp(0, 1) = 0;
    temp(0, 2) = ve / ((Rn + h) * (Rn + h));
    temp(1, 0) = 0;
    temp(1, 1) = 0;
    temp(1, 2) = -vn / ((Rm + h) * (Rm + h));
    temp(2, 0) = -WGS84_WIE * cos(lat) / (Rm + h) - ve / (cos(lat) * cos(lat)) / ((Rm + h) * (Rn + h));
    temp(2, 1) = 0;
    temp(2, 2) = -ve * tan(lat) / ((Rn + h) * (Rn + h));
    F.block<3, 3>(6, 0) = temp;

    temp(0, 0) = 0;
    temp(0, 1) = 1 / (Rn + h);
    temp(0, 2) = 0;
    temp(1, 0) = -1 / (Rm + h);
    temp(1, 1) = 0;
    temp(1, 2) = 0;
    temp(2, 0) = 0;
    temp(2, 1) = -tan(lat) / (Rn + h);
    temp(2, 2) = 0;
    F.block<3, 3>(6, 3) = temp;

    F.block<3, 3>(6, 6) = -rot_.vec2skewsmt(w_ien + w_enn);
    F.block<3, 3>(6, 9) = -Cbn;
    F.block<3, 3>(6, 15) = -Cbn * wibb.asDiagonal();
    F.block<3, 3>(9, 9) = -1 / Tgb * Eigen::Matrix3d::Identity();
    F.block<3, 3>(12, 12) = -1 / Tab * Eigen::Matrix3d::Identity();
    F.block<3, 3>(15, 15) = -1 / Tgs * Eigen::Matrix3d::Identity();
    F.block<3, 3>(18, 18) = -1 / Tas * Eigen::Matrix3d::Identity();
    
    /* 2. Q阵 */
    Q.block<3, 3>(0, 0) = vrw * vrw * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(3, 3) = arw * arw * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(6, 6) = 2 * sigma_gb / (Tgb * Tgb) * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(9, 9) = 2 * sigma_ab / (Tab * Tab) * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(12, 12) = 2 * sigma_gs / (Tgs * Tgs) * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(15, 15) = 2 * sigma_as / (Tas * Tas) * Eigen::Matrix3d::Identity();

    /* 3. G阵 */
    G.block<3, 3>(3, 0) = Cbn;
    G.block<3, 3>(6, 3) = Cbn;
    G.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    G.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();
    G.block<3, 3>(15, 12) = Eigen::Matrix3d::Identity();
    G.block<3, 3>(18, 15) = Eigen::Matrix3d::Identity();
    
    /* 离散化状态转移矩阵 */
    Phi = Eigen::Matrix<double, 21, 21>::Identity() + F * dt;

    /* 离散化系统噪声阵计算 */
    Qk  = 0.5 * (Phi * G * Q * G.transpose() * Phi.transpose() + G * Q * G.transpose()) * dt;

    /* 卡尔曼滤波：状态预测和协方差预测 */
    X_  = Phi * X_;
    P_  = Phi * P_ * Phi.transpose() + Qk;

    return GILIB_STATUS_OK;
}


status_t GILib::process_gnss(void *gpos_buf, int gpos_size)
{
    status_t        ret     = GILIB_STATUS_NONE;
    Eigen::Vector3d blh     = Eigen::Vector3d::Zero();
    Eigen::Vector3d std     = Eigen::Vector3d::Zero();
    Eigen::Vector3d larm    = Eigen::Vector3d::Zero();    
    Eigen::Matrix3d Cbn     = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Dr      = Eigen::Matrix3d::Zero();
    Eigen::Vector3d tmp     = Eigen::Vector3d::Zero();
    Eigen::Vector3d Z       = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 21, 21> I = Eigen::Matrix<double, 21, 21>::Identity();
    Eigen::Matrix<double, 3 , 21> H = Eigen::Matrix<double, 3 , 21>::Zero(); 
    Eigen::Matrix<double, 3 ,  3> R = Eigen::Matrix<double, 3 ,  3>::Zero();
    Eigen::Matrix<double, 21,  3> K = Eigen::Matrix<double, 21,  3>::Zero();
    double          Rm      = 0;
    double          Rn      = 0;
    double          lat     = 0;
    double          lon     = 0;
    double          h       = 0;

    /* 增加gnss数据 */
    add_gpos_data(gpos_buf, gpos_size);
    gpos_cur_ = gpos_;

    /* 变量赋值 */
    blh     = fusion_nav_.blh;
    std     = gpos_cur_.std;
    larm    = cfg_.vec_gnss_in_veh;
    Cbn     = fusion_nav_.Tbn.block<3, 3>(0, 0);
    lat     = fusion_nav_.blh(0);
    lon     = fusion_nav_.blh(1);
    h       = fusion_nav_.blh(2);
    Rm      = nav_log_.Rm;
    Rn      = nav_log_.Rn;

    if (nav_log_.align_flag != TRUE)
    {
        set_initstate();
        printf("set_init\n");
        return GILIB_STATUS_OK;
    }
    
    /* 杆臂补偿 */
    tmp     = { 1 / (Rm + h), 1 / ((Rn + h) * cos(lat)), -1};
    Dr      = tmp.asDiagonal();
    blh     = blh + Dr.inverse() * Cbn * larm;  /* 把IMU补偿到RTK天线处 */

    /* 观测值，RTK处的位置差 */
    Z       = Dr * (blh - gpos_cur_.blh);

    /* 观测矩阵 */
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(0, 6) = rot_.vec2skewsmt(Cbn * larm);

    /* 观测噪声阵 */
    R       = std.asDiagonal(); 

    /* 滤波更新 */
    K       = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
    X_      = X_ + K * (Z - H * X_);
    P_      = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();

    return ret;
}

status_t GILib::state_feedback()
{
    Eigen::Matrix3d Dr_inv  = Eigen::Matrix3d::Zero();
    Eigen::Vector3d tmp     = Eigen::Vector3d::Zero();
    Eigen::Quaterniond dqbn = Eigen::Quaterniond::Identity();
    double          Rm      = 0;
    double          Rn      = 0;
    double          h       = 0;
    double          lat     = 0;
    Rm      = nav_log_.Rm;
    Rn      = nav_log_.Rn; 
    lat     = fusion_nav_.blh(0);
    h       = fusion_nav_.blh(2);
    /* 得到经纬高和NED转换的矩阵 */
    tmp     = { 1 / (Rm + h), 1 / ((Rn + h) * cos(lat)), -1};
    Dr_inv  = tmp.asDiagonal();

    /* 误差状态补偿 */
    fusion_nav_.blh = fusion_nav_.blh - Dr_inv  *   X_.block<3, 1>( 0, 0);
    fusion_nav_.vel = fusion_nav_.vel -             X_.block<3, 1>( 3, 0);
    fusion_nav_.qbn = rot_.rotvec2quaternion(X_.block<3, 1>(6, 0)) * fusion_nav_.qbn;
    fusion_nav_.att = rot_.quaternion2euler(fusion_nav_.qbn);
    imu_err_.gyro_bias  = imu_err_.gyro_bias    +   X_.block<3, 1>( 9, 0);
    imu_err_.acc_bias   = imu_err_.acc_bias     +   X_.block<3, 1>(12, 0);
    imu_err_.gyro_scale = imu_err_.gyro_scale   +   X_.block<3, 1>(15, 0);
    imu_err_.acc_scale  = imu_err_.acc_scale    +   X_.block<3, 1>(18, 0);

    return GILIB_STATUS_OK;
}

/**
 * @description: 外部接口，根据type来调用对应的IMU或者是GNSS线程
 * @param {prctype_t} type: 算法处理类型
 * @param {void*} inbuf: 输入内存的头指针，将该指针对应的内存数据复制到私有变量中处理 
 * @param {int} insize: inbuf指向的内存大小
 * @param {void*} outbuf：线程输出指针
 * @param {int} outsize：outbuf指向的内存大小
 * @return {*}
 */
status_t GILib::process(const prctype_t type, void* inbuf, int insize, void* outbuf, int outsize)
{
    status_t ret = GILIB_STATUS_NONE;
    if (type == prctype_imu)
    {
        ret = process_imu(inbuf, insize);
    }
    if (type == prctype_gpos)
    {
        ret = process_gnss(inbuf, insize);
    }

    return ret;
}

status_t GILib::load_conf(const cfgtype_t type, void *inbuf, int insize)
{
    status_t ret = GILIB_STATUS_NONE;
    return ret;
}

status_t GILib::set_initstate()
{
    if (fabs(gpos_.time_stamp - imu_cur_.time_stamp) < 0.01)
    {
        fusion_nav_.time_stamp = gpos_.time_stamp;
        fusion_nav_.blh = gpos_.blh;
        fusion_nav_.pos.setZero();
        fusion_nav_.att.setZero();
        fusion_nav_.vel.setZero();
        fusion_nav_.Tbn.setIdentity();
        fusion_nav_.qbn.setIdentity();
        nav_log_.align_flag = TRUE;
    }

    return GILIB_STATUS_OK;
}
status_t GILib::gilib_output(void *outbuf, int outsize)
{
    gins_navout_t nav_out = { 0 };
    
    if (outsize < sizeof(gins_navout_t)) return GILIB_STATUS_FAIL;

    /* 输出结果 */
    memcpy(outbuf, &nav_out, outsize);
    return GILIB_STATUS_OK;
}