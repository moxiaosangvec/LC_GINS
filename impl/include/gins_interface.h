
/*
 * @Author: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @Date: 2023-08-12 21:13:51
 * @LastEditors: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @LastEditTime: 2023-11-02 21:55:53
 * @FilePath: /LC_GINSLIB/include/gilib_interface.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */


#ifndef _GINSLIB_INTERFACE_H_
#define _GINSLIB_INTERFACE_H_


/* 错误码 */


/* 算法库处理类型定义 */
typedef enum ginslib_prctype_t
{
    GINSLIB_PRCTYPE_NONE                = 0x00,             /* 无类型 */
    GINSLIB_PRCTYPE_IMU                 = 0xA1,             /* IMU处理类型 */
    GINSLIB_PRCTYPE_GNSSPOS             = 0xB1,             /* GNSS POS融合类型 */
    GINSLIB_PRCTYPE_GNSSVELB            = 0xB2,             /* GNSS VEL融合类型 */
    GINSLIB_PRCTYPE_GNSSVELN            = 0xB3,             /* GNSS VEL融合类型 */
    GINSLIB_PRCTYPE_ENUM_END            = 0xFFFFFFFF,       /* 枚举结束，用于字节对齐 */
}ginslib_prctype_t;

/* GNSS解类型定义 */
typedef enum ginslib_gnss_soltype_t
{
    GINSLIB_SOLQ_NONE                   = 0,                /* 无解 */
    GINSLIB_SOLQ_FIX                    = 1,                /* 固定解 */
    GINSLIB_SOLQ_FLOAT                  = 4,                /* 浮点解 */
    GINSLIB_SOLQ_SINGLE                 = 5,                /* 单点解 */
    GINSLIB_SOLQ_UNKNOWN                = 10,               /* 未知解类型 */
    GINSLIB_SOLQ_ENUM_END               = 0xFFFFFFFF,       /* 枚举结束位{4byte} */
} ginslib_gnss_soltype_t;

/* 算法输出定位标志位类型 */
typedef enum ginslib_sol_status_t
{
    GINSLIB_SOL_STATUS_INVALID          = 0,                /* 无效定位，尚未初始化 */ 
    GINSLIB_SOL_STATUS_ALIGN_COARSE     = 1,                /* 粗对准状态，同时也是收敛中 */
    GINSLIB_SOL_STATUS_ALIGN_PRECISE    = 2,                /* 精对准状态，即已经收敛 */
    GINSLIB_SOL_STATUS_PROPAGATE        = 3,                /* 绝对定位丢失，进入递推状态 */ 
    GINSLIB_SOL_STATUS_DIVERGING        = 4,                /* 正在发散 */
    GINSLIB_SOL_STATUS_DIVERGED         = 5,                /* 已经完全发散 */
    GINSLIB_SOL_STATUS_END              = 0xFFFFFFFF,       /* 枚举结束位 */
} ginslib_sol_status_t;

/* 算法库配置参数类型 */
typedef enum ginslib_cfg_type_t
{
    GINSLIB_CFG_TYPE_DEFAULT            = 1,                /* 默认配置参数类型 */
    GINSLIB_CFG_TYPE_CUSTOM             = 2,                /* 输入自定义参数 */
    GINSLIB_CFG_TYPE_END                = 0xFFFFFFFF,       /* 枚举结束位 */
}ginslib_cfg_type_t;

/* 算法配置参数 */
typedef struct ginslib_cfg_t
{
    int                                 gnss_base_flg;      /* gnss基站配置标志 */
    double                              gnss_base[3];       /* gnss基站 lat,lon,height {deg,deg,m} */
    double                              T_imu_to_veh[4][4]; /* imu相当于车辆的外参, T * imu_b = imu_n */
    double                              vec_gnss_to_veh[3]; /* gnss相对于车辆的杆臂，*/
} ginslib_cfg_t;


/* GINSLIB IMU数据输入 */
typedef struct ginslib_imu_t
{
    double                              time_stamp;        /* imu time{s} */
    double                              gyro[3];           /* 陀螺仪角速度{deg/s} */
    double                              acc[3];            /* 加速度{m/s/s} */
} ginslib_imu_t;


/* GINSLIB 位置融合数据输入 */
typedef struct ginslib_gpos_t
{
    double                              time_stamp;         /* time{s} */
    double                              blh[3];             /* 纬经高{deg,m} */
    double                              std[3];             /* 纬经高置信度{m} */
    ginslib_gnss_soltype_t              postype;            /* 定位类型 SOLQ_XXX */
} ginslib_pos_t;


/* GINSLIB 速度融合数据 */
typedef struct ginslib_gvel_t
{
    double                              time_stamp;         /* time {s} */
    double                              hori_vel;           /* 水平速度 */
    double                              trk_gnd;            /* 水平速度方向{deg} */
    double                              vert_vel;           /* 垂直速度 */
    ginslib_gnss_soltype_t              veltype;            /* 定位类型 SOLQ_XXX */
} ginslib_gvel_t;


/* GINSLIB输出 */
typedef struct ginslib_navout_t
{
    double                              time_stamp;         /* time {s} */
    ginslib_sol_status_t                sol_status;         /* 解状态 (0:no, 1:yes) */
    bool                                blh_valid_flg;      /* 纬经高有效标志 */
    double                              blh[3];             /* 纬经高 {deg,m} */
    double                              pos[3];             /* 位置 (NED) {m} */
    double                              vel[3];             /* 速度 (NED) {m/s} */
    double                              att[3];             /* 欧拉角 (rpy) {deg} */
    double                              qbn[4];             /* b到n的四元数 (wxyz)，b为前右下FRD，n为北东地NED */
    double                              Tbn[4][4];          /* b到n的位姿转换矩阵，按行存储，b为前右下FRD，n为北东地NED */
    double                              pos_std[3];         /* 位置协方差 {m} */
    double                              vel_std[3];         /* 速度协方差 {m/s} */
    double                              att_std[3];         /* 姿态协方差 {deg} */
} ginslib_navout_t;

#endif
