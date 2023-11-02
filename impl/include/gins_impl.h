/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-07-22 11:27:41
 * @LastEditors: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @LastEditTime: 2023-11-02 22:06:55
 * @FilePath: /LC_GINS/include/gi_lib.h
 * @Description: gilib库的接口文件
 */


#ifndef _GINS_IMPL_H_
#define _GINS_IMPL_H_

#include "../tool/earth.h"
#include "../tool/rotation.h"


#include <deque>

const double        DEG2RAD             = 0.01745329252f;   /* 角度转弧度 */
const double        RAD2DEG             = 57.2957795131f;   /* 弧度转角度 */


/* 错误码 */
#define             GINS_STATUS_NONE    (0x00)              /* 未知返回值 */
#define             GINS_STATUS_OK      (0x01)              /* 函数正常返回 */
#define             GINS_STATUS_FAIL    (0xFF)              /* 函数异常返回 */

typedef             int                 status_t;           /* 状态码 */

/******************************************************************************/
/*****************************  枚   举   类   型  *****************************/
/******************************************************************************/


/* 算法处理类型定义 */
typedef enum gins_prctype_t
{
    GINS_PRCTYPE_IMU                    = 1,
    GINS_PRCTYPE_GPOS                   = 2, 
    GINS_PRCTYPE_GVEL                   = 3,
    GINS_PRCTYPE_END                    = 0xFFFFFFFF,
} gins_prctype_t;

/* 算法输出定位状态类型定义 */
typedef enum gins_solstatus_t
{
    GINS_SOLSTATUS_WAIT_ALIGN           = 1,
    GINS_SOLSTATUS_ALIGN_COARSE         = 2,
    GINS_SOLSTATUS_ALIGN_REFINED        = 3,
    GINS_SOLSTATUS_DIVERGING            = 4,
    GINS_SOLSTATUS_DIVERGED             = 5,
    GINS_SOLSTATUS_END                  = 0xFFFFFFFF,
} gins_solstatus_t;

/* gnss定位类型定义 */
typedef enum gnss_soltype_t
{
    GNSS_SOLTYPE_NONE                   = 0,                /* 无解 */
    GNSS_SOLTYPE_FIX                    = 1,                /* 固定解 */
    GNSS_SOLTYPE_FLOAT                  = 4,                /* 浮点解 */
    GNSS_SOLTYPE_SINGLE                 = 5,                /* 单点解 */
    GNSS_SOLTYPE_UNKNOWN                = 10,               /* 未知解类型 */
    GNSS_SOLTYPE_END                    = 0xFFFFFFFF,       /* 枚举结束位{4byte} */
} gnss_soltype_t;

/* 车辆运动状态类型定义 */
typedef enum gins_vms_t 
{
    GINS_VMS_STAT                       = 1,                /* 车辆静止 */ 
    GINS_VMS_MOVEMENT                   = 2,                /* 车辆运动 */ 
    GINS_VMS_END                        = 0xFFFFFFFF,       /* 枚举结束位{4byte} */
} gins_vms_t;

/******************************************************************************/
/*****************************  结  构  体  定  义  *****************************/
/******************************************************************************/

/* 算法参数 */
typedef struct gins_param_t
{
    bool                                simplify_flg;       /* 是否简化计算（忽略地球自传和椭球性） */
    double                              Tba;                /* 加计偏置时间常数{s} */
    double                              Tbg;                /* 陀螺仪偏置时间常数{s} */
    double                              Tsa;                /* 加计比例因子时间常数{s} */
    double                              Tsg;                /* 陀螺仪比例因子时间常数{s} */
    double                              vrw;                /* 速率随机游走 */
    double                              arw;                /* 角度随机游走 */
    double                              sigma_ba;
    double                              sigma_bg;
    double                              sigma_sa;
    double                              sigma_sg;
    int                                 state_dim;          /* 状态维数 */
    int                                 noise_dim;          /* 噪声维数 */
    int                                 imu_deq_len;        /* imu队列长度 */
    int                                 imu_stat_deq_len;   /* 静态IMU数据队列长度 */
    int                                 gpos_deq_len;       /* gpos队列长度 */
    bool                                gnss_base_isvalid;  /* 基站有效标志 */
    Eigen::Vector3d                     gnss_base;          /* 基站配置blh{rad,m} */
    Eigen::Vector3d                     gnss_larm;          /* GNSS 杆臂{m} */
} gins_param_t;

/* 外部配置参数 */
typedef struct gins_cfg_t
{
    int                                 imu_sample_rate;    /* IMU采样率 */
    int                                 gnss_sample_rate;   /* GNSS采样率 */
    bool                                gnss_base_flg;      /* gnss基站配置成功标志 */
    Eigen::Vector3d                     gnss_base;          /* gnss基站 lat,lon,h{deg,deg,h} */
    Eigen::Matrix4d                     T_imu_to_veh;       /* imu到车体的坐标转换矩阵 T * imu_b = imu_n */ 
    Eigen::Vector3d                     vec_gnss_in_veh;    /* GNSS天线相对于车体的杆臂 */
} gins_cfg_t;

/* IMU 传感器数据 */
typedef struct gins_imu_t
{
    double                              time_stamp;         /* 时间戳 {s} */
    bool                                flg;                /* 有效标志位 */
    Eigen::Vector3d                     gyro;               /* 角速度 {rad/s} */
    Eigen::Vector3d                     acc;                /* 比力 {m/s/s} */
} gins_imu_t;

/* GNSS POS 传感器数据 */
typedef struct gins_gpos_t
{
    double                              time_stamp;         /* 时间戳 {s} */
    bool                                flg;                /* 有效标志位 */
    Eigen::Vector3d                     blh;                /* 纬经高 {rad,rad,m} */
    Eigen::Vector3d                     std;                /* 纬经高标准差 {m,m,m} */
    gnss_soltype_t                      pos_type;           /* 位置类型 */
    unsigned int                        sat_num;            /* 解算卫星数 */
} gins_gpos_t;

/* GNSS POS 传感器数据 */
typedef struct gins_gvel_t
{
    double                              time_stamp;         /* 时间戳 {s} */
    bool                                flg;                /* 有效标志位 */
    double                              hori_vel;           /* 水平速度 */
    double                              vert_vel;           /* 垂直速度 */
    double                              trk_gnd;            /* 水平速度对地角度，北向顺时针为正{rad} */
    gnss_soltype_t                      vel_type;           /* 速度类型 */
} gins_gvel_t;


/* 导航名义状态 */
typedef struct gins_nav_t
{
    double                              time_stamp;         /* 时间戳{s} */
    gins_solstatus_t                    sol_status;         /* 解状态 */
    Eigen::Vector3d                     pos;                /* 位置 ned {m} */
    Eigen::Vector3d                     vel;                /* 速度NED{m/s}*/
    Eigen::Vector3d                     att;                /* roll, pitch, yaw{rad} */
} gins_nav_t;

/* 误差状态，实际上没有必要，用X_就可以 */
typedef struct gins_err_t 
{
    double                              time_stamp;         /* 时间戳{s} */
    Eigen::Vector3d                     dpos;               /* 位置 ned {m} */
    Eigen::Vector3d                     dvel;               /* 速度NED{m/s}*/
    Eigen::Vector3d                     phi;                /* roll, pitch, yaw{rad} */
    Eigen::Vector3d                     dba;
    Eigen::Vector3d                     dbg;
    Eigen::Vector3d                     dsa;
    Eigen::Vector3d                     dsg;
} gins_err_t;

/* 算法执行日志 trace */
typedef struct gins_trace_t
{
    double                              sys_time;           /* 系统时间{s} */
    gins_prctype_t                      prctype;            /* 处理类型 */ 
} gins_trace_t;

/* 算法过程数据记录 */
typedef struct gins_log_t
{
    /* 传感器数据 */
    gins_imu_t                          imu2;               /* 连续三帧imu用于递推(editable) */
    gins_imu_t                          imu1;               /* 连续三帧imu用于递推(editable) */
    gins_imu_t                          imu0;               /* 连续三帧imu用于递推(editable) */
     
    /* 中间解算数据 */
    double                              Rm;                 /* 子午圈半径 {m} */
    double                              Rn;                 /* 卯酉圈半径 {m} */
    double                              g;                  /* 重力加速度{m/s/s} */
    Eigen::Vector3d                     wien;               /* wie在n系 {rad/s} */
    Eigen::Vector3d                     wenn;               /* wen在n系 {rad/s} */
    Eigen::Vector3d                     ba;
    Eigen::Vector3d                     bg;
    Eigen::Vector3d                     sa;
    Eigen::Vector3d                     sg;
} gins_log_t;

/******************************************************************************/
/*****************************  类      定     义  *****************************/
/******************************************************************************/

class Gins
{
private:
    /* dpos, dvel, datt, dba, dbg, dsa, dsg */
    enum StateID 
    { 
        DP_ID = 0, 
        DV_ID = 3, 
        DA_ID = 6, 
        DBA_ID = 9, 
        DBG_ID = 12, 
        DSA_ID = 15, 
        DSG_ID = 18,
        DG_ID = 19,
        DWS_ID = 20 
    };
    enum NoiseID
    { 
        VRW_ID = 0, 
        ARW_ID = 3, 
        DBASTD_ID = 6, 
        DBGSTD_ID = 9, 
        DSASTD_ID = 12, 
        DSGSTD_ID = 15, 
        DGSTD_ID = 16, 
        DWSSTD_ID = 17
    };

    /* 输入数据 */      
    gins_imu_t                          imupre_;            /* 上一帧时刻Imu测量 */
    gins_imu_t                          imucur_;            /* 当前帧时刻Imu测量 */
    gins_gpos_t                         gpos_;              /* 当前帧GNSS pos 观测 */
    gins_gvel_t                         gvel_;              /* 当前帧IMU数据 */
    std::deque<gins_imu_t>              imu_deque_;         /* Imu队列 */
    std::deque<gins_imu_t>              imu_stat_deque_;    /* 静态Imu队列 */
    std::deque<gins_gpos_t>             gpos_deque_;        /* gnss pos队列 */
    gins_cfg_t                          cfg_;               /* 算法配置参数 */

    /* 输出数据 */
    gins_nav_t                          nav_;               /* 组合导航结果 */

    /* 过程数据 */
    gins_param_t                        param_;             /* 算法相关参数 */
    gins_log_t                          log_;               /* 中间过程数据 */
    gins_trace_t                        trace_;             /* 算法中间过程记录，用来输出日志 */ 

    /* 计算工具句柄 */
    Rotation                            rot_;               /* 旋转计算类 */
    Earth                               earth_;             /* 地球计算类 */

    /* 滤波更新相关 */
    Eigen::VectorXd                     X_;                 /* 状态(dr{m},dv{m/s},da{rad},bg,ba,sg,sa) */
    Eigen::MatrixXd                     P_;                 /* 状态协方差 */
    

private:
    /* 内部算法处理 */

    status_t    add_imu_to_deque(const gins_imu_t &imu, const int &max_deque_len, std::deque<gins_imu_t> &imu_deque);
    /**
     * @description: 完成初始化，使定位进入到粗对准状态
     * @return {*}
     */    
    status_t    coarse_align();

    /**
     * @description: IMU数据转换至车体系
     * @param {const Matrix4d}  T       外参T_imu^veh
     * @param {const Vector3d}  att     车辆姿态(rpy{rad})
     * @param {imu_t}           &imu    IMU
     * @return {*}
     */
    status_t    imu_trans_to_veh(const Eigen::Matrix4d T, const Eigen::Vector3d att, gins_imu_t &imu);

    /**
     * @description: imu偏置比例因子补偿
     * @param {imu_t} &imu
     * @param {Vecotr3d} &ba
     * @param {Vecotr3d} &bg
     * @param {Vecotr3d} &sa
     * @param {Vecotr3d} &sg
     * @return {*}
     */    
    status_t    imu_compensate(gins_imu_t &imu, 
                               const Eigen::Vector3d &ba, 
                               const Eigen::Vector3d &bg, 
                               const Eigen::Vector3d &sa, 
                               const Eigen::Vector3d &sg);

    status_t    imu_mech(const gins_imu_t &imupre, 
                         const gins_imu_t &imucur, 
                         const gins_nav_t &navpre,
                         gins_nav_t &navcur);
    
    status_t    state_cov_predict(const gins_imu_t &imupre, const gins_imu_t &imucur, const gins_nav_t &nav);
    status_t    state_feedback();

private:
    /* 内部和外部的数据交换接口 */
    status_t    set_param();
    status_t    init_log_trace();
    status_t    process_imu(void *imu_buf, int imu_size);
    status_t    process_gnss(void *gpos_buf, int gpos_size);
    status_t    add_imu_data(const void *imu_buf, const int imu_size, gins_imu_t &imu);
    status_t    add_gpos_data(void *gpos_buf, int gpos_size);
    status_t    gins_return_output(void *outbuf, int outsize);

public:
    Gins();
    /**
     * @description: 外部接口函数，载入配置参数
     * @param {cfgtype_t} type
     * @param {void} *inbuf 配置结构体指针头
     * @param {int} insize 配置结构体大小
     * @return {*} 状态码
     */    
    status_t    ginslib_set_config(const ginslib_cfg_type_t cfg_type, void *inbuf, int insize);
    /**
     * @description: 外部接口函数
     * @param {prctype_t} type 算法处理类型
     * @param {void} *inbuf 输入数据
     * @param {int} insize 输入数据内存大小
     * @param {void} *outbuf 输出数据
     * @param {int} outsize 输出数据内存大小
     * @return {*}
     */    
    status_t    ginslib_process(const ginslib_prctype_t type, void *inbuf, int insize, void *outbuf, int outsize);
};

#endif
