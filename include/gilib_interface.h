/*
 * @Author: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @Date: 2023-08-12 21:13:51
 * @LastEditors: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @LastEditTime: 2023-09-19 21:54:16
 * @FilePath: /LC_GINS/include/gilib_interface.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */


#ifndef _GILIB_INTERFACE_H_
#define _GILIB_INTERFACE_H_

/* 算法库处理类型定义 */
typedef enum gins_prctype_t
{
    GINS_PRCTYPE_NONE       = 0x00,         /* 无类型 */
    GINS_PRCTYPE_IMU        = 0xA1,         /* IMU处理类型 */
    GINS_PRCTYPE_GNSSPOS    = 0xB1,         /* GNSS POS融合类型 */
    GINS_PRCTYPE_GNSSVELB   = 0xB2,         /* GNSS VEL融合类型 */
    GINS_PRCTYPE_GNSSVELN   = 0xB3,         /* GNSS VEL融合类型 */
    GINS_PRCTYPE_ENUM_END   = 0xFFFFFFFF,   /* 枚举结束，用于字节对齐 */
}gins_prctype_t;

/* GNSS解类型定义 */
typedef enum gins_solq_t
{
    GINS_SOLQ_NONE          = 0,            /* 无解 */
    GINS_SOLQ_FIX           = 1,            /* 固定解 */
    GINS_SOLQ_FLOAT         = 4,            /* 浮点解 */
    GINS_SOLQ_SINGLE        = 5,            /* 单点解 */
    GINS_SOLQ_UNKNOWN       = 10,           /* 未知解类型 */
    GINS_SOLQ_ENUM_END      = 0xFFFFFFFF,   /* 枚举结束位{4byte} */
} gins_solq_t;


typedef struct gins_cfg_t
{
    int         gnss_base_flg;      /* gnss基站配置标志 */
    double      gnss_base[3];       /* gnss基站 lat,lon,height {deg,deg,m} */
    double      T_imu_to_veh[4][4]; /* imu相当于车辆的外参, T * imu_b = imu_n */
    double      vec_gnss_to_veh[3]; /* gnss相对于车辆的杆臂，*/
} gins_cfg_t;


/* GILIB IMU数据输入 */
typedef struct gins_imu_t
{
    double      time_stamp;        /* imu time{s} */
    double      gyro[3];           /* 陀螺仪角速度{deg/s} */
    double      acc[3];            /* 加速度{m/s/s} */
} gins_imu_t;

/* GILIB 位置融合数据输入 */
typedef struct gins_gpos_t
{
    double      time_stamp;         /* time{s} */
    double      blh[3];             /* 纬经高{deg,m} */
    double      std[3];             /* 纬经高置信度{m} */
    gins_solq_t postype;            /* 定位类型 SOLQ_XXX */
} gins_pos_t;

/* GILIB b系速度融合数据 */
typedef struct gins_gvelb_t
{
    double      time_stamp;         /* time {s} */
    double      velb[3];            /* 速度 (FRD) {m/s} */
    double      std[3];             /* 速度置信度 (FRD）{m/s} */
    gins_solq_t veltype;            /* 定位类型 SOLQ_XXX */
} gins_gvelb_t;


/* GILIB n系速度融合数据 */
typedef struct gins_gveln_t
{
    double      time_stamp;         /* time {s} */
    double      veln[3];            /* 速度 (NED) {m/s} */
    double      std[3];             /* 速度置信度 (NED) {m/s} */
    gins_solq_t veltype;            /* 定位类型 SOLQ_XXX */
} gins_gveln_t;

/* GILIB输出 */
typedef struct gins_navout_t
{
    double      time_stamp;         /* time {s} */
    int         converge_flg;       /* 收敛状态 (0:no, 1:yes) */
    int         align_flg;          /* 对准状态 (0:no, 1:yes) */
    int         blh_valid_flg;      /* 纬经高有效标志，无效时输出的blh无效 (0:no, 1:yes) */
    double      blh[3];             /* 纬经高 {deg,m} */
    double      pos[3];             /* 位置 (NED) {m} */
    double      vel[3];             /* 速度 (NED) {m/s} */
    double      att[3];             /* 欧拉角 (rpy) {deg} */
    double      gyro_bias[3];       /* 陀螺仪偏置 (rpy) {deg/s} */
    double      acc_bias[3];        /* 加速度计偏置 (rpy) {m/s/s} */
    double      gyro_scale[3];      /* 加速度计偏置 (rpy) {m/s/s} */
    double      acc_scale[3];       /* 加速度计偏置 (rpy) {m/s/s} */
    double      qbn[4];             /* b到n的四元数 (xyzw)，b为前右下FRD，n为北东地NED */
    double      Tbn[4][4];          /* b到n的位姿转换矩阵，按行存储，b为前右下FRD，n为北东地NED */
    double      pos_std[3];         /* 位置协方差 {m} */
    double      vel_std[3];         /* 速度协方差 {m/s} */
    double      att_std[3];         /* 姿态协方差 {deg} */
} gins_navout_t;





#endif
