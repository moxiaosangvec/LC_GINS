/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-07-21 21:50:29
 * @LastEditors: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @LastEditTime: 2023-09-20 02:12:56
 * @FilePath: /LC_GINS/src/lcgi_lib.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "earth.h"
#include "gi_lib.h"
#include "gilib_interface.h"
#include <Eigen/Core>

/**
 * @description: 加入IMU数据
 * @param {void} *imu_buf IMU数据起始地址
 * @param {int} imu_size IMU数据内存大小
 * @return {*}
 */
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
/**
 * @description: 加入gnss pos数据
 * @param {void} *gpos_buf gpos数据起始地址
 * @param {int} gpos_size gpos数据内存大小
 * @return {*}
 */
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

/**
 * @description: 完成状态的初始对准，初始化基站配置，初始化中间数据
 * @return {*}
 */
status_t GILib::initialize()
{
    Eigen::Vector3d blh = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    /* 导航状态初值 */
    fusion_nav_.time_stamp = gpos_.time_stamp;
    fusion_nav_.pos.setZero();  /* 位置初值为0 */
    fusion_nav_.vel.setZero();  /* TODO: 速度初值不应该为0 */
    fusion_nav_.att.setZero();  /* TODO: 姿态角初值不应该为0 */
    vel = fusion_nav_.vel;
    /* 内部配置基站参数 */
    cfg_.gnss_base = gpos_.blh;     /* 初始位置作为内部的基站 */
    cfg_.gnss_base_flg = TRUE;
    
    /* 内部日志参数 */
    nav_log_.align_flag = TRUE;
    nav_log_.blh_for_earth = gpos_.blh; /* 初始化位置用于地球计算 */
    blh = nav_log_.blh_for_earth;
    nav_log_.blh_valid_flg = TRUE;
    nav_log_.Rm     = earth_.get_Rm(blh(0));
    nav_log_.Rn     = earth_.get_Rn(blh(0));
    nav_log_.g      = earth_.gravity(blh(0), blh(2));
    nav_log_.wien   = earth_.wien(blh(0));
    nav_log_.wenn   = earth_.wenn(nav_log_.Rm, nav_log_.Rn, blh, vel);
    nav_log_.ba.setZero(); 
    nav_log_.bg.setZero(); 
    nav_log_.sa.setZero(); 
    nav_log_.sg.setZero(); 

    return GILIB_STATUS_OK;
}
/**
 * @description: imu数据转换至车体系
 * @param {Matrix4d} T      -I      imu到车体的外参，T * imu_origin = imu_veh
 * @param {Vector3d} att    -I      当前的姿态角，在转换至车体时需要用到
 * @param {imu_t} &imu      -I/O    转换的IMU
 * @return {*}
 */
status_t GILib::imu_trans_to_veh(const Eigen::Matrix4d T, 
                                 const Eigen::Vector3d att, 
                                 imu_t &imu)
{
    imu.acc  = T.block<3, 3>(0, 0) * imu.acc;
    imu.gyro = T.block<3, 3>(0, 0) * imu.gyro;

    /* 暂时不考虑姿态 */
    return GILIB_STATUS_OK;
}

/**
 * @description: imu数据补偿偏置和比例因子
 * @param {imu_t} &imu  -I/O    转换前后的imu数据
 * @return {*}
 */
status_t GILib::imu_compensate(imu_t &imu)
{
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    imu.acc  -= nav_log_.ba; 
    imu.gyro -= nav_log_.bg;
    
    Eigen::Vector3d gyrscale, accscale;
    gyrscale   = Eigen::Vector3d::Ones() + nav_log_.sg; 
    accscale   = Eigen::Vector3d::Ones() + nav_log_.sa; 
    imu.gyro   = imu.gyro.cwiseProduct(gyrscale.cwiseInverse());
    imu.acc    = imu.acc.cwiseProduct(accscale.cwiseInverse());

    return GILIB_STATUS_OK;
}

/**
 * @description: IMU机械编排算法
 * @param {int} simplify_flag   -I  简化的IMU机械编排算法标志
 * @param {imu_t} &imu_pre      -I  上一帧IMU
 * @param {imu_t} &imu_cur      -I  当前帧IMU
 * @param {navstate_t} &nav_cur -I  当前帧导航状态
 * @param {navstate_t} &nav_nxt -O  名义状态预测
 * @return {*}
 */
status_t GILib::imu_mech(const int simplify_flag, 
                         const imu_t &imu_pre, const imu_t &imu_cur, 
                         const navstate_t &nav_cur, navstate_t &nav_nxt)
{
    status_t ret = GILIB_STATUS_NONE;

    Eigen::Vector3d pos             = Eigen::Vector3d::Zero();
    Eigen::Vector3d att             = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel             = Eigen::Vector3d::Zero();
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

    double dt = 0;

    /* 0. 数据准备 */
    pos = nav_cur.pos;
    vel = nav_cur.vel;
    att = nav_cur.att;

    w_ien = nav_log_.wien;
    w_enn = nav_log_.wenn;

    dt = nav_log_.imu_cur_.time_stamp - nav_log_.imu_pre_.time_stamp;
    g_pn = { 0, 0, nav_log_.g };

    /* 1. IMU姿态递推 */
    qbn = rot_.euler2quaternion(fusion_nav_.att);
    dtheta_cur = nav_log_.imu_cur_.gyro * dt;
    dtheta_pre = nav_log_.imu_pre_.gyro * dt;
    dvcur      = nav_log_.imu_cur_.acc * dt;
    dvpre      = nav_log_.imu_pre_.acc * dt;
    phib = dtheta_cur + 1 / 12 * dtheta_pre.cross(dtheta_cur);
    
    dqbb.w() = cos(0.5 * phib.norm());
    dqbb.vec() = sin(0.5 * phib.norm()) / (0.5 * phib.norm()) * (0.5 * phib);
    phin = (w_ien + w_enn) * dt;
    
    dqnn.w() = cos(0.5 * phin.norm());
    dqnn.vec() = -sin(0.5 * phin.norm()) / (0.5 * phin.norm()) * (0.5 * phin);

    if (simplify_flag) qbn = qbn * dqbb;
    else qbn = dqnn * qbn * dqbb; 

    /* 2. IMU速度递推 */
    if (simplify_flag) dv_gn = g_pn * dt;
    else dv_gn = (g_pn - (2 * w_ien + w_enn).cross(vel)) * dt;
    dv_fb = dvcur + 0.5 * dtheta_cur.cross(dvcur) + 
            1 / 12 * (dtheta_pre.cross(dvcur) + dvcur.cross(dtheta_cur));
    if (simplify_flag) dv_fn = rot_.euler2rotmat(att) * dv_fb;
    else dv_fn = (I3 - 0.5 * rot_.vec2skewsmt(phin)) * rot_.euler2rotmat(att) * dv_fb;
    vel += dv_fn + dv_gn;

    /* 3. IMU位置递推 */
    pos = pos + 0.5 * (vel + fusion_nav_.vel) * dt; 
    
    /* 4. 将更新结果写入到状态变量 */
    fusion_nav_.time_stamp = imu_cur_.time_stamp;
    fusion_nav_.qbn = qbn;
    fusion_nav_.Tbn.block<3, 3>(0, 0) = qbn.toRotationMatrix();
    fusion_nav_.att = rot_.quaternion2euler(qbn);
    fusion_nav_.vel = vel;
    fusion_nav_.blh = blh;
    return ret;
}

/**
 * @description: 状态的协方差递推预测，直接修改类的私有变量{X_ P_}
 * @param {int} simplify_flag 简化导航方程标志，简化后不考虑地球自传
 * @param {navstate_t} nav 当前名义导航状态，构建矩阵需要
 * @return {X_, P_}
 */    
status_t GILib::state_cov_predict(const int simplify_flag, navstate_t nav)
{
    /* 变量定义 */
    Eigen::MatrixXd F;      /* 连续方程误差状态矩阵 */
    Eigen::MatrixXd Phi;    /* 离散化的状态转移矩阵 */
    Eigen::MatrixXd G;      /* 噪声驱动矩阵 */
    Eigen::MatrixXd Q;      /* 输入噪声阵 */
    Eigen::MatrixXd Qk;     /* 系统噪声矩阵 */

    Eigen::Matrix3d temp = Eigen::Matrix3d::Zero();

    double vn = 0, ve = 0, vd = 0;
    double Rm = 0, Rn = 0, h = 0, g = 0;
    
    /* 变量赋值 */

    /* 矩阵分配内存 */
    F.resize(STATE_DIM_, STATE_DIM_);
    Phi.resize(STATE_DIM_, STATE_DIM_);
    G.resize(STATE_DIM_, NOISE_DIM_);    
    Q.resize(NOISE_DIM_, NOISE_DIM_);
    Qk.resize(STATE_DIM_, STATE_DIM_);

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


}
status_t GILib::process_imu(void *inbuf, int insize)
{
    /* 变量定义 */
    navstate_t    nav_result;
    
    /* 增加IMU数据，写到私有变量imu_中 */
    add_imu_data(inbuf, insize);
    
    /* IMU外参转换 */
    imu_trans_to_veh(cfg_.T_imu_to_veh, fusion_nav_.att, imu_);

    /* imu数据补偿，补偿偏置和比例因子 */
    imu_compensate(imu_);
    
    /* imu数据写入到nav_log中 */
    nav_log_.imu_pre_ = nav_log_.imu_cur_;
    nav_log_.imu_cur_ = imu_;

    /* imu名义状态递推 */
    imu_mech(cfg_.simplify_flg, 
                    nav_log_.imu_pre_, nav_log_.imu_cur_, 
                    fusion_nav_, nav_result);

    /* 系统状态和协方差预测 */
    state_cov_predict(cfg_.simplify_flg, fusion_nav_);
 
    /* 结果保存 */
    fusion_nav_ = nav_result;
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
        initialize();
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
    gins_cfg_t cfg = { 0 };
    if (type == cfgtype_custom)
    {
        memcpy(&cfg, inbuf, insize);
        if (cfg.gnss_base_flg)
        {
            cfg_.gnss_base(0) = cfg.gnss_base[0] * DEG2RAD;
            cfg_.gnss_base(1) = cfg.gnss_base[1] * DEG2RAD;
            cfg_.gnss_base(2) = cfg.gnss_base[2];
            for (size_t i = 0; i < 4; i++)
            {
                for (size_t j = 0; j < 4; j++)
                {
                    cfg_.T_imu_to_veh(i, j) = cfg.T_imu_to_veh[i][j];
                }
            }
            for (size_t i = 0; i < 3; i++)
            {
                cfg_.vec_gnss_in_veh(i) = cfg.vec_gnss_to_veh[i];
            }      
        }
        cfg_.Tab = cfg.Tab;
        cfg_.Tas = cfg.Tas;
        cfg_.Tgb = cfg.Tgb;
        cfg_.Tgs = cfg.Tgs;
    }
    return ret;
}

status_t GILib::gilib_output(void *outbuf, int outsize)
{
    gins_navout_t nav_out = { 0 };
    
    if (outsize < sizeof(gins_navout_t)) return GILIB_STATUS_FAIL;

    /* 输出结果 */
    memcpy(outbuf, &nav_out, outsize);
    return GILIB_STATUS_OK;
}