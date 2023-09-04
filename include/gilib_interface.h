/*
 * @Author: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @Date: 2023-08-12 21:13:51
 * @LastEditors: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @LastEditTime: 2023-09-03 23:03:23
 * @FilePath: /LC_GINS/include/gilib_interface.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */


#ifndef _GILIB_INTERFACE_H_
#define _GILIB_INTERFACE_H_

/* 算法库处理类型定义 */
typedef enum gins_prctype_t
{
    GINS_PRCTYPE_NONE       = 0,
    GINS_PRCTYPE_IMU        = 1,
    GINS_PRCTYPE_GNSSPOS    = 2,
    GINS_PRCTYPE_GNSSVEL    = 3,
    GINS_PRCTYPE_ENUM_END   = 0xFFFFFFFF,
}gins_prctype_t;

/* GNSS解类型定义 */
typedef enum gins_solq_t
{
    GINS_SOLQ_NONE       = 0,            /* 无解 */
    GINS_SOLQ_FIX        = 1,            /* 固定解 */
    GINS_SOLQ_FLOAT      = 4,            /* 浮点解 */
    GINS_SOLQ_SINGLE     = 5,            /* 单点解 */
    GINS_SOLQ_UNKNOWN    = 10,           /* 未知解类型 */
    GINS_SOLQ_ENUM_END   = 0xFFFFFFFF,   /* 枚举结束位{4byte} */
} gins_solq_t;

/* GILIB IMU数据输入 */
typedef struct gins_imu_t
{
    int    flg;                     /* 数据有效位 */
    double time_stamp;              /* imu time{s} */
    double gyro[3];                 /* 陀螺仪角速度{deg/s} */
    double acc[3];                  /* 加速度{m/s/s} */
} gins_imu_t;

/* GILIB 位置融合数据输入 */
typedef struct gins_gpos_t
{
    int    flg;                     /* 数据有效位 */
    gins_solq_t postype;            /* 定位类型 SOLQ_XXX */
    double time_stamp;              /* time{s} */
    double blh[3];                  /* 纬经高{deg,m} */
    double std[3];                  /* 纬经高置信度{m} */
} gins_pos_t;

/* GILIB b系速度融合数据 */
typedef struct gins_gvelb_t
{
    int    flg;                     /* 数据有效位 */
    gins_solq_t veltype;            /* 定位类型 SOLQ_XXX */
    double time_stamp;              /* time{s} */
    double velb[3];                 /* 前右下{m/s} */
    double std[3];                  /* 前右下置信度{m/s} */
} gins_gvelb_t;


/* GILIB n系速度融合数据 */
typedef struct gins_gveln_t
{
    int    flg;                     /* 数据有效位 */
    gins_solq_t veltype;            /* 定位类型 SOLQ_XXX */
    double time_stamp;              /* time{s} */
    double veln[3];                 /* 北东地{m/s} */
    double std[3];                  /* 北东地置信度{m/s} */
} gins_gveln_t;


/* GILIB输出 */
typedef struct gins_navout_t
{
    double time_stamp;              /* 时间戳{s} */
    int    converg_flag;            /* 收敛状态 */
    double blh[3];                  /* 纬度、经度、高度{deg,m} */
    double pos[3];                  /* 位置（北东地{m}） */
    double vel[3];                  /* 速度（北东地{m/s}） */
    double att[3];                  /* 欧拉角 (roll, pitch, yaw{deg}) */
    double qbn[4];                  /* b到n的四元数{w,xyz}，b为前右下FRD，n为北东地NED */
    double Tbn[4][4];               /* b到n的位姿转换矩阵，b为前右下FRD，n为北东地NED */
    double pos_std[3];              /* 位置协方差{m}*/
    double vel_std[3];              /* 速度协方差{m/s} */
    double att_std[3];              /* 姿态协方差{deg} */
} gins_navout_t;


#endif
