/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-07-21 21:50:29
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2023-07-27 01:33:35
 * @FilePath: /LC_GINS/src/lcgi_lib.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "gi_lib.h"
#include "mathcal.h"
#include <Eigen/Core>

/**
 * @description: 默认构造函数
 * @return {*}
 */
GILib::GILib()
{
    /* 完成数据初始化 */
    time_stamp_ = 0.0;
    imu_pre_.time = 0.0;
    imu_pre_.acc.setZero();
    imu_pre_.gyro_deg.setZero();
    imu_cur_.time = 0.0;
    imu_cur_.acc.setZero();
    imu_cur_.gyro_deg.setZero();
    memset(&nav_flag_, 0, sizeof(navflag_t));
    err_.ba.setZero();
    err_.bg.setZero();
    err_.sa.setZero();
    err_.sg.setZero();
    err_.da.setZero();
    err_.dr.setZero();
    err_.dv.setZero();
    fusion_nav_.blh_deg.setZero();
    fusion_nav_.flag = 0;
    fusion_nav_.pos_ned.setZero();
    fusion_nav_.rpy_deg.setZero();
    fusion_nav_.qbn.setIdentity();
    fusion_nav_.Tbn.setIdentity();
    fusion_nav_.vel_ned.setZero();
}

double GILib::getgravity()
{
    double lat_rad = 0;
    double h = 0;
    double sin2 = 0;
    double gravity = 0;
    lat_rad = fusion_nav_.blh_deg[0] * DEG2RAD;
    h = fusion_nav_.blh_deg[2];
    sin2 = sin(lat_rad);
    sin2 *= sin2;

    gravity = 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) +
               h * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * h * h;
    return gravity;
}

status_t GILib::process_imu()
{
    status_t ret = GILIB_STATUS_NONE;
    double dt = 0.0;
    double Rm = 0.0;
    double Rn = 0.0;
    double H = 0.0;
    Eigen::Vector3d pos_ned         = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_ned         = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_ned_mid     = Eigen::Vector3d::Zero();
    Eigen::Vector3d blh_deg         = Eigen::Vector3d::Zero();
    Eigen::Vector3d blh_deg_mid     = Eigen::Vector3d::Zero();
    Eigen::Matrix4d Tbn             = Eigen::Matrix4d::Zero();
    Eigen::Quaterniond qbn          = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond dqbb         = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond dqnn         = Eigen::Quaterniond::Identity();
    Eigen::Vector3d phib_rad        = Eigen::Vector3d::Zero();     /* b系下的等效旋转矢量 */
    Eigen::Vector3d phin_rad        = Eigen::Vector3d::Zero();     /* n系下的等效旋转矢量 */
    Eigen::Vector3d dtheta_rad_pre  = Eigen::Vector3d::Zero();
    Eigen::Vector3d dtheta_rad_cur  = Eigen::Vector3d::Zero();
    Eigen::Matrix3d I3              = Eigen::Matrix3d::Identity();

    Eigen::Vector3d dv_gn = Eigen::Vector3d::Zero();
    Eigen::Vector3d dv_fn = Eigen::Vector3d::Zero();
    Eigen::Vector3d dv_fb = Eigen::Vector3d::Zero();
    Eigen::Vector3d dvcur = Eigen::Vector3d::Zero();
    Eigen::Vector3d dvpre = Eigen::Vector3d::Zero();
    Eigen::Vector3d g_pn  = Eigen::Vector3d::Zero();    /* 当地重力加速度 */
    Eigen::Vector3d w_ien = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_enn = Eigen::Vector3d::Zero();


    /* */
    blh_deg = fusion_nav_.blh_deg;
    vel_ned = fusion_nav_.vel_ned;
    Tbn = fusion_nav_.Tbn;

    Rm = WGS84_RA * (1 - WGS84_SQR_E1) / (pow(1 - WGS84_SQR_E1 * sin(blh_deg(0) * DEG2RAD), 1.5));
    Rn = WGS84_RA / pow(1 - WGS84_SQR_E1 * sin(blh_deg(0) * DEG2RAD), 0.5);
    H = blh_deg(2);
    w_ien = { WGS84_WIE_RAD * cos(blh_deg(0) * DEG2RAD), 0, -WGS84_WIE_RAD * sin(blh_deg(0) * DEG2RAD)};
    w_enn(0) = vel_ned(1) / (Rn + H);
    w_enn(1) = -vel_ned(0) / (Rm + H);
    w_enn(2) = -vel_ned(1) * tan(blh_deg(0) * DEG2RAD) / (Rn + H);
    g_pn = { 0, 0, getgravity() };

    /* 1. IMU姿态递推 */
    qbn = fusion_nav_.qbn;
    dt = imu_cur_.time - imu_pre_.time;
    dtheta_rad_cur = imu_cur_.gyro_deg;
    dtheta_rad_pre = imu_pre_.gyro_deg;
    dvcur          = imu_cur_.acc;
    dvpre          = imu_pre_.acc;
    phib_rad = dtheta_rad_cur + 1 / 12 * dtheta_rad_pre.cross(dtheta_rad_cur);
    dqbb.w() = cos(0.5 * phib_rad.norm());
    dqbb.vec() = sin(0.5 * phib_rad.norm()) / (0.5 * phib_rad.norm()) * (0.5 * phib_rad);
    phin_rad = (w_ien + w_enn) * dt;
    dqnn.w() = cos(0.5 * phin_rad.norm());
    dqnn.vec() = -sin(0.5 * phin_rad.norm()) / (0.5 * phin_rad.norm()) * (0.5 * phin_rad);
    qbn = dqnn * qbn * dqbb; 
    qbn.normalized();

    /* 2. IMU速度递推 */
    dv_gn = (g_pn - (2 * w_ien + w_enn).cross(vel_ned)) * dt;
    dv_fb = dvcur + 0.5 * dtheta_rad_cur.cross(dvcur) + 
            1 / 12 * (dtheta_rad_pre.cross(dvcur) + dvcur.cross(dtheta_rad_cur));
    dv_fn = (I3 - 0.5 * vec2mat_skewsmt(phin_rad)) * Tbn.block<3, 3>(0, 0) * dv_fb;
    vel_ned += dv_fn + dv_gn;

    /* 3. IMU位置递推 */
    vel_ned_mid = 0.5 * (vel_ned + fusion_nav_.vel_ned);
    blh_deg(2) += -0.5 * (vel_ned_mid(2)) * dt;
    blh_deg_mid(2) = 0.5 * (blh_deg(2) + fusion_nav_.blh_deg(2));
    blh_deg(0) = (blh_deg(0) * DEG2RAD + 0.5 * vel_ned_mid(0) / (Rm + blh_deg_mid(2))) * RAD2DEG;
    blh_deg_mid(0) = 0.5 * (blh_deg(0) + fusion_nav_.blh_deg(0));
    blh_deg(1) = (blh_deg(1) * DEG2RAD + 0.5 * vel_ned_mid(1) / 
                 ((Rn + blh_deg_mid(2)) * cos(blh_deg_mid(0) * DEG2RAD)) * dt) * RAD2DEG;

    /* 4. 将更新结果写入到状态变量 */
    fusion_nav_.flag = true;
    fusion_nav_.time_stamp = imu_cur_.time;
    fusion_nav_.qbn = qbn;
    fusion_nav_.Tbn.block<3, 3>(0, 0) = fusion_nav_.qbn.toRotationMatrix();
    fusion_nav_.vel_ned = vel_ned;
    fusion_nav_.blh_deg = blh_deg;
    return ret;
}

status_t GILib::return_nav(navstate_t &navstate)
{
    status_t ret = GILIB_STATUS_NONE;
    navstate = fusion_nav_;
    return ret; 
}
status_t GILib::process_gnss()
{
    status_t ret = GILIB_STATUS_NONE;
    return ret;
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
        memcpy(&imu_pre_, &imu_cur_, sizeof(imu_t)); 
        memcpy(&imu_cur_, inbuf, insize);
        if (imu_pre_.time > 0)
        {
            ret = process_imu();
        }
        else
        {
            ret = set_initstate();
        }
    }
    if (type == prctype_gnss)
    {
        memcpy(&gnss_, inbuf, insize);
        ret = process_gnss();
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
    fusion_nav_.blh_deg = { 30.4447858054, 114.4718661162, 21.095 };
    fusion_nav_.pos_ned = { 0, 0, 0 };
    fusion_nav_.vel_ned = { 0, 0, 0 };
    fusion_nav_.rpy_deg = { 0.85421502, -2.03480295, 185.70235133 };
    fusion_nav_.qbn     = fusion_nav_.rpy_deg.
    return GILIB_STATUS_OK;
}