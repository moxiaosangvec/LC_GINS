/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-07-22 10:56:05
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2023-07-25 21:40:44
 * @FilePath: /LC_GINS/include/common.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _COMMON_H_
#define _COMMON_H_
#include <Eigen/Core>
#include <Eigen/Geometry>

const double DEG2RAD        = 0.01745329252f;
const double RAD2DEG        = 57.2957795131f;

const double WGS84_WIE_RAD  = 7.2921151467E-5;       /* 地球自转角速度*/
const double WGS84_F        = 0.0033528106647474805; /* 扁率 */
const double WGS84_RA       = 6378137.0000000000;    /* 长半轴a */
const double WGS84_RB       = 6356752.3142451793;    /* 短半轴b */
const double WGS84_GM0      = 398600441800000.00;    /* 地球引力常数 */
const double WGS84_SQR_E1   = 0.0066943799901413156; /* 第一偏心率的平方 */
const double WGS84_SQR_E2   = 0.0067394967422764341; /* 第二偏心率的平方 */

const int    STATE_DIM      = 21;

/* 错误码 */
#define GILIB_STATUS_OK         (1)
#define GILIB_STATUS_NONE       (0)
#define GILIB_STATUS_FAIL       (-1)

typedef int status_t;

typedef enum cfgtype_t
{
    cfgtype_default = 1,
    cfgtype_custom = 2,
    cfgtype_end = 0xFFFFFFFF,
}cfgtype_t;

typedef enum prctype_t
{
    prctype_none = 0,
    prctype_imu  = 1,
    prctype_gnss = 2,
    prctype_end  = 0xFFFFFFFF,
} prctype_t;

typedef struct cfg_t
{
    Eigen::Matrix4d Tbn_imu_to_veh; 
    Eigen::Vector3d vec_gnss_in_veh;
} cfg_t;

typedef struct imu_t
{
    double time;
    Eigen::Vector3d gyro_deg;
    Eigen::Vector3d acc;
}imu_t;

typedef struct gnss_t
{
    double time;
    int postype;
    int veltype;
    int headtype;
    Eigen::Vector3d llh_deg;
    Eigen::Vector3d ned;
    Eigen::Vector3d ned_std;
    Eigen::Vector3d vel_ned; 
    double head_deg;
} gnss_t;


/* 状态数据 */
typedef struct errstate_t
{
    Eigen::Vector3d dr;     /* 位置误差向量 */
    Eigen::Vector3d dv;     /* 速度误差向量 */
    Eigen::Vector3d da;     /* 姿态误差向量（失准角） */
    Eigen::Vector3d bg;     /* 陀螺仪偏置 */
    Eigen::Vector3d ba;     /* 加速度计偏置 */
    Eigen::Vector3d sg;     /* 陀螺仪比例因子 */
    Eigen::Vector3d sa;     /* 加速度计比例因子 */
}errstate_t;

typedef struct errcov_t
{
    /* data */
    int dim;
    Eigen::MatrixXd cov;
    
} errcov_t;


/* 导航状态 */
typedef struct navstate_t
{
    int flag;
    double time_stamp;
    Eigen::Vector3d pos_ned;
    Eigen::Vector3d blh_deg;
    Eigen::Vector3d vel_ned;
    Eigen::Quaterniond qbn;
    Eigen::Vector3d rpy_deg;
    Eigen::Matrix4d Tbn;
} navstate_t;

typedef struct navflag_t
{
    /* data */
    int align_flag;
} navflag_t;

#endif