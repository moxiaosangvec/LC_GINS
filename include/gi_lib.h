/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-07-22 11:27:41
 * @LastEditors: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @LastEditTime: 2023-09-20 01:59:24
 * @FilePath: /LC_GINS/include/gi_lib.h
 * @Description: gilib库的接口文件
 */


#ifndef _GI_LIB_H_
#define _GI_LIB_H_

#include "earth.h"
#include "rotation.h"

#include "gilib_interface.h"

#include <deque>

const double    DEG2RAD         =   0.01745329252f;         /* 角度转弧度 */
const double    RAD2DEG         =   57.2957795131f;         /* 弧度转角度 */

// const double    WGS84_WIE       =   7.2921151467E-5;        /* 自转角速度 {rad/s} */
// const double    WGS84_F         =   0.0033528106647474805;  /* 扁率 */
// const double    WGS84_RA        =   6378137.0000000000;     /* 长半轴a {m} */
// const double    WGS84_RB        =   6356752.3142451793;     /* 短半轴b {m} */
// const double    WGS84_GM0       =   398600441800000.00;     /* 地球引力常数 */
// const double    WGS84_SQR_E1    =   0.0066943799901413156;  /* 第一偏心率的平方 */
// const double    WGS84_SQR_E2    =   0.0067394967422764341;  /* 第二偏心率的平方 */

const int       STATE_DIM       =   21;                     /* 状态维数 */
const int       NOISE_DIM       =   6;                      /* 噪声维数 */
const int       GPOS_OBS_DIM    =   3;                      /* GNSS POS 观测维度 */
const int       IMU_DEQUE_TIME  =   3;                      /* Imu 窗口时长 */
const int       GPOS_DEQUE_TIME =   3;                      /* Gnss 窗口时长 */
const int       IMU_SAMPLE_FRQ  =   100;                    /* Imu采样率 */
const int       GPOS_SAMPLE_FRQ =   5;                      /* GNSS pos采样率 */
const int       TRUE            =   1;                      /* 布尔值 TRUE */
const int       FALSE           =   0;                      /* 布尔值 FALSE */

/* 类型定义 */
typedef         char                bool_t;                 /* 布尔类型 */ 

/* 错误码 */
#define         GILIB_STATUS_NONE   (0x00)                  /* 未知返回值 */
#define         GILIB_STATUS_OK     (0x01)                  /* 函数正常返回 */
#define         GILIB_STATUS_FAIL   (0xFF)                  /* 函数异常返回 */

typedef         int                 status_t;               /* 状态码 */


typedef enum solqtype_t
{
    SOLQ_NONE           =           0,                      /* 无解 */
    SOLQ_FIX            =           1,                      /* 固定解 */
    SOLQ_FLOAT          =           4,                      /* 浮点解 */
    SOLQ_SINGLE         =           5,                      /* 单点解 */
    SOLQ_UNKNOWN        =           10,                     /* 未知解类型 */
    SOLQ_ENUM_END       =           0xFFFFFFFF,             /* 枚举结束位{4byte} */
}solqtype_t;

typedef enum cfgtype_t
{
    cfgtype_default     =           1,                      /* 默认类型 */
    cfgtype_custom      =           2,                      /* 自定义输入参数 */
    cfgtype_end         =           0xFFFFFFFF,             /* 枚举结束位{4byte} */
}cfgtype_t;

typedef enum prctype_t
{
    prctype_none        =           0,                      /* 无类型，不做处理 */
    prctype_imu         =           1,                      /* IMU处理类型 */
    prctype_gpos        =           2,                      /* GNSS POS处理类型 */
    prctype_gvel        =           3,                      /* GNSS VEL处理类型 */
    prctype_end         =           0xFFFFFFFF,             /* 枚举结束位 */
} prctype_t;

typedef enum enum_union_pos_t
{
    NED_TYPE            =           1,                      /* NED位置类型 */
    BLH_TYPE            =           2,                      /* BLH位置类型 */
    TYPE_END            =           0xFFFFFFFF,             /* 枚举结束位 */
}enum_uni_pos_t;
typedef struct cfg_t
{
    /* 来源于外部调用配置，和车型、需求有关 */
    bool_t              gnss_base_flg;          /* gnss基站配置成功标志 */
    Eigen::Vector3d     gnss_base;              /* gnss基站 lat,lon,h{deg,deg,h} */
    Eigen::Matrix4d     T_imu_to_veh;           /* imu到车体的坐标转换矩阵 T * imu_b = imu_n */ 
    Eigen::Vector3d     vec_gnss_in_veh;        /* GNSS天线相对于车体的杆臂 */
    
    /* 内部算法配置使用 */
    bool_t              simplify_flg;           /* 是否简化计算（忽略地球自传和椭球性） */
    double              Tba;                    /* 加计偏置时间常数{s} */
    double              Tbg;                    /* 陀螺仪偏置时间常数{s} */
    double              Tsa;                    /* 加计比例因子时间常数{s} */
    double              Tsg;                    /* 陀螺仪比例因子时间常数{s} */
    int                 state_dim;              /* 状态维数 */
} cfg_t;

typedef struct imu_t
{
    double              time_stamp;             /* 时间戳 {s} */
    Eigen::Vector3d     gyro;                   /* 角速度 {rad/s} */
    Eigen::Vector3d     acc;                    /* 比力 {m/s/s} */
} imu_t;

typedef struct gpos_t
{
    double              time_stamp;             /* 时间戳 {s} */
    Eigen::Vector3d     blh;                    /* 纬经高 {rad,rad,m} */
    Eigen::Vector3d     std;                    /* 纬经高标准差 {m,m,m} */
    solqtype_t          sol_type;               /* 定位类型 */
    unsigned int        sat_num;                /* 解算卫星数 */
} gpos_t;


typedef struct imu_err_t
{
    Eigen::Vector3d     gyro_bias;              /* 陀螺仪偏置 {rad/s} */
    Eigen::Vector3d     acc_bias;               /* 加计偏置 {m/s/s} */
    Eigen::Vector3d     gyro_scale;             /* 陀螺仪偏置 {rad/s} */
    Eigen::Vector3d     acc_scale;              /* 加计偏置 {m/s/s} */
} imu_err_t;

/* 导航状态 */
typedef struct navstate_t
{
    double              time_stamp;             /* 时间戳{s} */
    Eigen::Vector3d     pos;                    /* 位置 ned {m} */
    Eigen::Vector3d     vel;                    /* 速度NED{m/s}*/
    Eigen::Vector3d     att;                    /* roll, pitch, yaw{rad} */
} navstate_t;

typedef struct syslog_t
{
    double              sys_time;               /* 系统时间 */
    prctype_t           prctype;                /* 处理类型 */ 
} syslog_t;


typedef struct navlog_t
{
    /* 传感器数据 */
    imu_t               imu_pre_;               /* 上一帧imu(editable) */
    imu_t               imu_cur_;               /* 当前imu(editable) */
    gpos_t              gpos_cur_;              /* 当前gnss pos(editable) */
    
    /* 标志位数据 */
    bool_t              align_flag;             /* 对准标志位 */
    bool_t              blh_valid_flg;          /* 纬经高可用标志 */
    
    /* 中间解算数据 */
    double              Rm;                     /* 子午圈半径 {m} */
    double              Rn;                     /* 卯酉圈半径 {m} */
    double              g;                      /* 重力加速度{m/s/s} */
    Eigen::Vector3d     wien;                   /* wie在n系 {rad/s} */
    Eigen::Vector3d     wenn;                   /* wen在n系 {rad/s} */
    Eigen::Vector3d     blh_for_earth;          /* blh用来地球解算 {rad,m}*/
    Eigen::Vector3d     ba;                     /* 加计偏置 {m/s/s} */
    Eigen::Vector3d     bg;                     /* 陀螺仪偏置 {rad/s} */
    Eigen::Vector3d     sa;                     /* 加计比例因子 */
    Eigen::Vector3d     sg;                     /* 陀螺仪比例因子 */
} navlog_t;


class GILib
{
private:
    double              time_stamp_     = 0;    /* 时间戳 {s}*/
    const int           STATE_DIM_      = 21;   /* 状态维数 */
    const int           NOISE_DIM_      = 12;   /* 噪声维数 */
    /* Imu队列最大长度 */
    const int           IMU_DEQUE_NUM_  = 300;  /* imu队列长度 */
    /* GPOS队列最大长度 */
    const int           GPOS_DEQUE_NUM_ = 15;   /* GPOS队列长度 */
    
    /* dpos, dvel, datt, dba, dbg, dsa, dsg */
    enum StateID 
    { 
        P_ID = 0, 
        V_ID = 3, 
        PHI_ID = 6, 
        BA_ID = 9, 
        BG_ID = 12, 
        SA_ID = 15, 
        SG_ID = 18 
    };
    
    /* vrw(acc noise), arw(gyro noise), bastd, bgstd, sastd, sgstd */
    enum NoiseID 
    { 
        VRW_ID = 0, 
        ARW_ID = 3, 
        BASTD_ID = 6, 
        BGSTD_ID = 9, 
        SASTD_ID = 12, 
        SGSTD_ID = 15 
    };

    /* 输入数据，算法只做访问不做修改 */      
    imu_t               imu_;                   /* 当前时刻Imu测量(readonly) */
    gpos_t              gpos_;                  /* GNSS pos 观测(readonly) */
    std::deque<imu_t>   deque_imu_;             /* Imu队列(readonly) */
    std::deque<gpos_t>  deque_gpos_;            /* gnss pos队列(readonly) */
    cfg_t               cfg_;                   /* 算法内部配置参数(readonly) */

    /* 输出数据 */
    navstate_t          dr_nav_;                /* 航位推算结果 */
    navstate_t          fusion_nav_;            /* 组合导航结果 */

    /* 过程数据 */
    navlog_t            nav_log_;               /* 中间过程数据 */
    
    /* 计算句柄 */
    Rotation            rot_;                   /* 旋转计算类 */
    Earth               earth_;                 /* 地球数据计算类 */

    Eigen::MatrixXd     P_;                     /* 状态协方差 */
    Eigen::MatrixXd     X_;     /* 状态(dr{m},dv{m/s},da{rad},bg,ba,sg,sa) */
    

private:
    /* 内部算法处理 */
    status_t    initialize();
    status_t    earth_data_compute(const double lat, const double height);
    status_t    imu_trans_to_veh(const Eigen::Matrix4d T, const Eigen::Vector3d att, imu_t &imu);
    status_t    imu_compensate(imu_t &imu);
    /**
     * @description: IMU机械编排，递推下一时刻的位姿
     * @param {int} simplify_flag  简化导航方程标志，简化后不考虑地球自传
     * @param {navstate_t} &nav_cur  当前输入导航名义状态
     * @param {navstate_t} &nav_nxt  IMU递推出下一时刻的导航名义状态
     * @return {*}
     */    
    status_t    imu_mech(const int simplify_flag, 
                         const imu_t &imu_pre, const imu_t &imu_cur,
                         const navstate_t &nav_cur, navstate_t &nav_nxt);
    
    /**
     * @description: 状态的协方差递推预测，直接修改类的私有变量{X_ P_}
     * @param {int} simplify_flag 简化导航方程标志，简化后不考虑地球自传
     * @param {navstate_t} nav 当前名义导航状态，构建矩阵需要
     * @return {X_, P_}
     */    
    status_t    state_cov_predict(const int simplify_flag, navstate_t nav);
    status_t    state_feedback();

private:
    /* 内部和外部的数据交换接口 */
    status_t    process_imu(void *imu_buf, int imu_size);
    status_t    process_gnss(void *gpos_buf, int gpos_size);
    status_t    add_imu_data(void *imu_buf, int imu_size);
    status_t    add_gpos_data(void *gpos_buf, int gpos_size);
    status_t    gilib_output(void *outbuf, int outsize);

public:
    /**
     * @description: 外部接口函数，载入配置参数
     * @param {cfgtype_t} type
     * @param {void} *inbuf 配置结构体指针头
     * @param {int} insize 配置结构体大小
     * @return {*} 状态码
     */    
    status_t    load_conf(const cfgtype_t type, void *inbuf, int insize);
    /**
     * @description: 外部接口函数
     * @param {prctype_t} type 算法处理类型
     * @param {void} *inbuf 输入数据
     * @param {int} insize 输入数据内存大小
     * @param {void} *outbuf 输出数据
     * @param {int} outsize 输出数据内存大小
     * @return {*}
     */    
    status_t    process(const prctype_t type, void *inbuf, int insize, void *outbuf, int outsize);
};

#endif
