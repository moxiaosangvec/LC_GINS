/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-07-22 10:56:05
 * @LastEditors: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @LastEditTime: 2023-09-03 22:25:01
 * @FilePath: /LC_GINS/include/common.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _COMMON_H_
#define _COMMON_H_
#include <Eigen/Core>
#include <Eigen/Geometry>

const double DEG2RAD        = 0.01745329252f;
const double RAD2DEG        = 57.2957795131f;

const double WGS84_WIE      = 7.2921151467E-5;       /* 地球自转角速度{rad/s} */
const double WGS84_F        = 0.0033528106647474805; /* 扁率 */
const double WGS84_RA       = 6378137.0000000000;    /* 长半轴a */
const double WGS84_RB       = 6356752.3142451793;    /* 短半轴b */
const double WGS84_GM0      = 398600441800000.00;    /* 地球引力常数 */
const double WGS84_SQR_E1   = 0.0066943799901413156; /* 第一偏心率的平方 */
const double WGS84_SQR_E2   = 0.0067394967422764341; /* 第二偏心率的平方 */

const int    STATE_DIM      = 21;                    /* 状态维数 */

const int    TRUE           = 1;
const int    FALSE          = 0;

/* 错误码 */
#define GILIB_STATUS_OK         (1)
#define GILIB_STATUS_NONE       (0)
#define GILIB_STATUS_FAIL       (-1)

typedef int status_t;




typedef enum solqtype_t
{
    SOLQ_NONE       = 0,            /* 无解 */
    SOLQ_FIX        = 1,            /* 固定解 */
    SOLQ_FLOAT      = 4,            /* 浮点解 */
    SOLQ_SINGLE     = 5,            /* 单点解 */
    SOLQ_UNKNOWN    = 10,           /* 未知解类型 */
    SOLQ_ENUM_END   = 0xFFFFFFFF,   /* 枚举结束位{4byte} */
}solqtype_t;

typedef enum cfgtype_t
{
    cfgtype_default = 1,
    cfgtype_custom  = 2,
    cfgtype_end = 0xFFFFFFFF,
}cfgtype_t;

typedef enum prctype_t
{
    prctype_none = 0,
    prctype_imu  = 1,
    prctype_gpos = 2,
    prctype_gvel = 3,
    prctype_end  = 0xFFFFFFFF,
} prctype_t;

typedef struct cfg_t
{
    Eigen::Matrix4d Tbn_imu_to_veh;     /* imu到车体的坐标转换矩阵 */ 
    Eigen::Vector3d vec_gnss_in_veh;    /* GNSS天线相对于车体的杆臂 */
    double Tgb;                         /* 时间常数{s} */
    double Tab;                         /* 时间常数{s} */
    double Tgs;                         /* 时间常数{s} */
    double Tas;                         /* 时间常数{s} */
} cfg_t;

typedef struct imu_t
{
    double          time;           /* 时间戳{s} */
    Eigen::Vector3d gyro;           /* 陀螺仪角速度{rad/s} */
    Eigen::Vector3d acc;            /* 比力{m/s/s} */
} imu_t;

typedef struct gnss_pos_t
{
    double          time_stamp;     /* 时间戳{s} */
    Eigen::Vector3d blh;            /* 纬经高{rad,rad,m} */
    Eigen::Vector3d std;            /* 纬经高标准差{m,m,m} */
    solqtype_t      sol_type;       /* 定位类型{} */
    unsigned int    sat_num;        /* 解算卫星数 */
} gnss_pos_t;


typedef struct imu_err_t
{
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_scale;
    Eigen::Vector3d acc_scale;
} imu_err_t;


/* 状态数据 */
typedef struct errstate_t
{
    Eigen::Vector3d dr;         /* 位置误差向量(n系{m}) */
    Eigen::Vector3d dv;         /* 速度误差向量 */
    Eigen::Vector3d da;         /* 姿态误差向量(失准角{rad}) */
    Eigen::Vector3d bg;         /* 陀螺仪偏置误差 {rad/s} */
    Eigen::Vector3d ba;         /* 加速度计偏置误差 */
    Eigen::Vector3d sg;         /* 陀螺仪比例因子误差 */
    Eigen::Vector3d sa;         /* 加速度计比例因子误差 */
}errstate_t;

/* 导航状态 */
typedef struct navstate_t
{
    int             flag;           /* 定位标志 */
    double          time_stamp;     /* 时间戳{s} */
    Eigen::Vector3d blh;            /* 纬度、经度、高度{rad,m} */
    Eigen::Vector3d pos;            /* 位置NED{m} */
    Eigen::Vector3d vel;            /* 速度NED{m/s}*/
    Eigen::Vector3d att;            /* roll, pitch, yaw{rad} */
    Eigen::Matrix4d Tbn;            /* 位姿矩阵 */
    Eigen::Quaterniond qbn;         /* 四元数 */
} navstate_t;

typedef struct syslog_t
{
    double      sys_time;
    prctype_t   prctype;               
} syslog_t;


typedef struct navflag_t
{
    /* data */
    int align_flag;
} navflag_t;

#endif