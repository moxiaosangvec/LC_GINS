/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-07-21 21:50:29
 * @LastEditors: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @LastEditTime: 2023-11-02 22:18:05
 * @FilePath: /LC_GINS/src/lcgi_lib.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "../tool/earth.h"
#include "../tool/rotation.h"
#include "../include/gins_impl.h"
#include "../include/gins_interface.h"

#include <Eigen/Core>

Gins::Gins()
{
    
}

status_t Gins::set_param()
{
    double imu_duration = 0;
    imu_duration = 3.0;

    param_.simplify_flg = false;       /* 是否简化计算（忽略地球自传和椭球性） */
    param_.Tba = 3600;                /* 加计偏置时间常数{s} */
    param_.Tbg = 3600;                /* 陀螺仪偏置时间常数{s} */
    param_.Tsa = 3600;                /* 加计比例因子时间常数{s} */
    param_.Tsg = 3600;                /* 陀螺仪比例因子时间常数{s} */
    param_.vrw = 0;                /* 速率随机游走 */
    param_.arw = 0;                /* 角度随机游走 */
    param_.sigma_ba = 0;
    param_.sigma_bg = 0;
    param_.sigma_sa = 0;
    param_.sigma_sg = 0;
    param_.state_dim = 0;          /* 状态维数 */
    param_.noise_dim = 0;          /* 噪声维数 */
    param_.imu_deq_len = 0;        /* imu队列长度 */
    param_.imu_stat_deq_len = 0;   /* 静态IMU数据队列长度 */
    param_.gpos_deq_len = 0;       /* gpos队列长度 */
    param_.gnss_base_isvalid = false;  /* 基站有效标志 */
    param_.gnss_base.setZero();          /* 基站配置blh{rad,m} */
    param_.gnss_larm.setZero();          /* GNSS 杆臂{m} */
}

status_t Gins::init_log_trace()
{
    log_.ba.setZero();
    log_.bg.setZero();
    log_.sa.setZero();
    log_.sg.setZero();
    log_.g = 0;
    log_.Rm = 0;
    log_.Rn = 0;
    log_.wenn.setZero();
    log_.wien.setZero();
}
/**
 * @description: 加入IMU数据
 * @param {void} *imu_buf IMU数据起始地址
 * @param {int} imu_size IMU数据内存大小
 * @return {*}
 */
status_t Gins::add_imu_data(const void *imu_buf, const int imu_size, gins_imu_t &imu)
{
    ginslib_imu_t lib_imu = { 0 };
    if (imu_size != sizeof(ginslib_imu_t) || imu_buf == nullptr) return GINS_STATUS_FAIL;

    memcpy(&lib_imu, imu_buf, sizeof(ginslib_imu_t));

    /* 转换到内部定义，并写到私有变量 readonly */
    imu.time_stamp = lib_imu.time_stamp;
    for (size_t i = 0; i < 3; i++)
    {
        imu.gyro[i] = lib_imu.gyro[i] * DEG2RAD;
        imu.acc[i] = lib_imu.acc[i];
    }

    return GINS_STATUS_OK;
}

status_t Gins::add_imu_to_deque(const gins_imu_t &imu, const int &max_deque_len, std::deque<gins_imu_t> &imu_deque)
{
    imu_deque.push_back(imu);
    if (imu_deque.size() > max_deque_len)
    {
        imu_deque.pop_front();
    }
    return GINS_STATUS_OK;
}

/**
 * @description: 加入gnss pos数据
 * @param {void} *gpos_buf gpos数据起始地址
 * @param {int} gpos_size gpos数据内存大小
 * @return {*}
 */
status_t Gins::add_gpos_data(void *gpos_buf, int gpos_size)
{
    ginslib_pos_t pos = { 0 };
    if (gpos_size != sizeof(ginslib_pos_t) || gpos_buf == nullptr) return GINS_STATUS_FAIL;
    memcpy(&pos, gpos_buf, sizeof(ginslib_pos_t));
    
    /* 外部接口数据定位转换至内部的定义 */
    gpos_.time_stamp = pos.time_stamp;
    gpos_.blh[0] = pos.blh[0] * DEG2RAD;
    gpos_.blh[1] = pos.blh[1] * DEG2RAD;
    gpos_.blh[2] = pos.blh[2];
    gpos_.pos_type = (gnss_soltype_t)pos.postype;    /* 目前内外定义是一致的，可以直接转 */
    gpos_.std[0] = pos.std[0];
    gpos_.std[1] = pos.std[1];
    gpos_.std[2] = pos.std[2];
    
    if (gpos_deque_.size() == param_.gpos_deq_len)
    {
        gpos_deque_.pop_front();
    }
    gpos_deque_.push_back(gpos_);

    return GINS_STATUS_OK;
}

/**
 * @description: 完成状态的初始对准，初始化基站配置，初始化中间数据
 * @return {*}
 */
status_t Gins::coarse_align()
{
    /* 用当前gpos状态设置组合导航状态初值 */
    nav_.time_stamp = gpos_.time_stamp;
    nav_.pos.setZero();  /* 位置初值为0 */
    nav_.vel.setZero();  /* TODO: 速度初值不应该为0 */
    nav_.att.setZero();  /* TODO: 姿态角初值不应该为0 */
    nav_.sol_status = GINS_SOLSTATUS_ALIGN_COARSE;   /* 解状态设置为粗对准 */

    /* 内部基站配置 */
    param_.gnss_base = gpos_.blh;

    /* 内部数据记录 */
    log_.ba.setZero();
    log_.bg.setZero();
    log_.sa.setZero();
    log_.sg.setZero();
    log_.Rm     = earth_.get_Rm(gpos_.blh(0));
    log_.Rn     = earth_.get_Rn(gpos_.blh(0));
    log_.g      = earth_.gravity(gpos_.blh(0), gpos_.blh(2));
    log_.wien   = earth_.wien(gpos_.blh(0));

    /* wenn和速度有关，初始化时速度为0，wenn实际上也为0 */
    log_.wenn   = earth_.wenn(log_.Rm, log_.Rn, gpos_.blh, nav_.vel);
    return GINS_STATUS_OK;
}


/**
 * @description: imu数据转换至车体系
 * @param {Matrix4d} T      -I      imu到车体的外参，T * imu_origin = imu_veh
 * @param {Vector3d} att    -I      当前的姿态角，在转换至车体时需要用到
 * @param {imu_t} &imu      -I/O    转换的IMU
 * @return {*}
 */
status_t Gins::imu_trans_to_veh(const Eigen::Matrix4d T, 
                                 const Eigen::Vector3d att, 
                                 gins_imu_t &imu)
{
    imu.acc  = T.block<3, 3>(0, 0) * imu.acc;
    imu.gyro = T.block<3, 3>(0, 0) * imu.gyro;

    /* 暂时不考虑姿态 */
    return GINS_STATUS_OK;
}

/**
 * @description: imu数据补偿偏置和比例因子
 * @param {imu_t} &imu  -I/O    转换前后的imu数据
 * @return {*}
 */
status_t Gins::imu_compensate(gins_imu_t &imu, 
                               const Eigen::Vector3d &ba, 
                               const Eigen::Vector3d &bg, 
                               const Eigen::Vector3d &sa, 
                               const Eigen::Vector3d &sg)
{
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    imu.acc  -= ba; 
    imu.gyro -= bg;
    
    Eigen::Vector3d gyrscale, accscale;
    gyrscale   = Eigen::Vector3d::Ones() + sg; 
    accscale   = Eigen::Vector3d::Ones() + sa; 
    imu.gyro   = imu.gyro.cwiseProduct(gyrscale.cwiseInverse());
    imu.acc    = imu.acc.cwiseProduct(accscale.cwiseInverse());

    return GINS_STATUS_OK;
}

status_t Gins::imu_mech(const gins_imu_t &imu1, 
                        const gins_imu_t &imu0, 
                        const gins_nav_t &navpre,
                        gins_nav_t &navcur)
{
    status_t ret = GINS_STATUS_NONE;
    bool simplify_flag = false;
    bool single_precycle_flg = false;

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
    Eigen::Vector3d dvel_cur = Eigen::Vector3d::Zero();
    Eigen::Vector3d dvel_pre = Eigen::Vector3d::Zero();
    Eigen::Vector3d g_pn  = Eigen::Vector3d::Zero();                    /* 当地重力加速度 */
    Eigen::Vector3d w_ien = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_enn = Eigen::Vector3d::Zero();

    double dt = 0;

    /* 0. 数据准备 */
    pos = navpre.pos;
    vel = navpre.vel;

    w_ien = log_.wien;
    w_enn = log_.wenn;
    g_pn = { 0, 0, log_.g };

    dt = imu0.time_stamp - imu1.time_stamp;
    dtheta_cur = 0.5 * (imu0.gyro + imu1.gyro) * dt;
    dvel_cur   = 0.5 * (imu0.acc  + imu1.acc ) * dt;

    dt = imu0.time_stamp - imu1.time_stamp;

    /* 1. IMU姿态递推 */
    qbn = rot_.euler2quaternion(nav_.att);
    phib = dtheta_cur;
    // if (single_precycle_flg) phib += 1.0 / 12 * dtheta_pre.cross(dtheta_cur);
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
    dv_fb = dvel_cur;
    if (single_precycle_flg) dv_fb += 0.5 * dtheta_cur.cross(dvel_cur) + 
            1 / 12 * (dtheta_pre.cross(dvel_cur) + dvel_cur.cross(dtheta_cur));
    if (simplify_flag) dv_fn = rot_.euler2rotmat(att) * dv_fb;
    else dv_fn = (I3 - 0.5 * rot_.vec2skewsmt(phin)) * rot_.euler2rotmat(att) * dv_fb;
    vel += dv_fn + dv_gn;

    /* 3. IMU位置递推 */
    pos = pos + 0.5 * (vel + nav_.vel) * dt; 
    
    /* 4. 将更新结果写入到状态变量 */
    nav_.time_stamp = imu0.time_stamp;
    nav_.att = rot_.quaternion2euler(qbn);
    nav_.vel = vel;
    return ret;
}

/**
 * @description: 状态的协方差递推预测，直接修改类的私有变量{X_ P_}
 * @param {int} simplify_flag 简化导航方程标志，简化后不考虑地球自传
 * @param {navstate_t} nav 当前名义导航状态，构建矩阵需要
 * @return {X_, P_}
 */    
status_t Gins::state_cov_predict(const gins_imu_t &imupre, const gins_imu_t &imucur, const gins_nav_t &navpre)
{
    /* 变量定义 */
    Eigen::MatrixXd F;      /* 连续方程误差状态矩阵 */
    Eigen::MatrixXd Phi;    /* 离散化的状态转移矩阵 */
    Eigen::MatrixXd G;      /* 噪声驱动矩阵 */
    Eigen::MatrixXd Q;      /* 输入噪声阵 */
    Eigen::MatrixXd Qk;     /* 系统噪声矩阵 */
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d att;
    Eigen::Matrix3d Cbn;
    Eigen::Vector3d blh;
    Eigen::Matrix3d I3      = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d temp    = Eigen::Matrix3d::Zero();
    Eigen::MatrixXd I;

    double dt = 0;
    dt = imucur.time_stamp - imupre.time_stamp;
    
    /* 变量赋值 */
    pos = navpre.pos;
    vel = navpre.vel;
    att = navpre.att; 
    Cbn = rot_.euler2rotmat(navpre.att);

    /* 矩阵分配内存 */
    F.resize(param_.state_dim, param_.state_dim);
    Phi.resize(param_.state_dim, param_.state_dim);
    G.resize(param_.state_dim, param_.noise_dim);    
    Q.resize(param_.noise_dim, param_.noise_dim);
    Qk.resize(param_.state_dim, param_.state_dim);
    I.resize(param_.state_dim, param_.noise_dim);
    
    /* 矩阵赋值 */
    F.setZero();
    Phi.setZero();
    G.setZero();
    Q.setZero();
    Qk.setZero();
    I.setIdentity();

    /* 1. F阵 */
    F.block<3, 3>(DP_ID, DV_ID) = I3; 
    F.block<3, 3>(DV_ID, DA_ID) = rot_.vec2skewsmt(Cbn * imupre.acc);
    F.block<3, 3>(DV_ID, DBA_ID) = Cbn;
    F.block<3, 3>(DV_ID, DSA_ID) = -Cbn * imupre.acc.asDiagonal();
    F.block<3, 3>(DA_ID, DBG_ID) = -Cbn;
    F.block<3, 3>(DA_ID, DSG_ID) = -Cbn * imupre.gyro.asDiagonal(); 
    F.block<3, 3>(DBA_ID, DBA_ID) = -1 / param_.Tba * I3;
    F.block<3, 3>(DBG_ID, DBG_ID) = -1 / param_.Tbg * I3;
    F.block<3, 3>(DSA_ID, DSA_ID) = -1 / param_.Tsa * I3;
    F.block<3, 3>(DSG_ID, DSG_ID) = -1 / param_.Tsg * I3;
   
    /* 2. Q阵 */
    Q.block<3, 3>(VRW_ID, VRW_ID) = param_.vrw * param_.vrw * I3;
    Q.block<3, 3>(ARW_ID, ARW_ID) = param_.arw * param_.arw * I3;
    Q.block<3, 3>(DBASTD_ID, DBASTD_ID) = 2 * param_.sigma_ba / (param_.Tba * param_.Tba) * I3;
    Q.block<3, 3>(DBGSTD_ID, DBGSTD_ID) = 2 * param_.sigma_bg / (param_.Tbg * param_.Tbg) * I3;
    Q.block<3, 3>(DSASTD_ID, DSASTD_ID) = 2 * param_.sigma_sa / (param_.Tsa * param_.Tsa) * I3;
    Q.block<3, 3>(DSGSTD_ID, DSGSTD_ID) = 2 * param_.sigma_sg / (param_.Tsg * param_.Tsg) * I3;

    /* 3. G阵 */
    G.block<3, 3>(DV_ID, VRW_ID) = Cbn; G.block<3, 3>(DA_ID, ARW_ID) = Cbn;
    G.block<3, 3>(DBA_ID, DBASTD_ID) = I3; 
    G.block<3, 3>(DBG_ID, DBGSTD_ID) = I3; 
    G.block<3, 3>(DSA_ID, DSASTD_ID) = I3; 
    G.block<3, 3>(DSG_ID, DSGSTD_ID) = I3; 

    /* 离散化系统噪声阵计算 */
    Qk  = 0.5 * (Phi * G * Q * G.transpose() * Phi.transpose() + G * Q * G.transpose()) * dt;

    /* 卡尔曼滤波：状态预测和协方差预测 */
    X_  = Phi * X_;
    P_  = Phi * P_ * Phi.transpose() + Qk;

    return GINS_STATUS_OK;
}

status_t Gins::process_imu(void *inbuf, int insize)
{
    /* 变量定义 */
    gins_nav_t      navpre;
    gins_nav_t      navcur;

    /* 数据赋值 */
    navpre = nav_;

    /* 增加IMU数据，写到私有变量imucur_中 */
    imupre_ = imucur_;
    add_imu_data(inbuf, insize, imucur_);

    /* imu数据外参转换 */
    imu_trans_to_veh(cfg_.T_imu_to_veh, nav_.att, imucur_);
    
    /* 补偿IMU的偏置和比例因子 */
    imu_compensate(imucur_, log_.ba, log_.bg, log_.sa, log_.sg);

    /* imu名义状态递推 */
    imu_mech(imupre_, imucur_, navpre, navcur);

    /* 系统状态和协方差预测 */
    state_cov_predict(imupre_, imucur_, navpre);

    /* 结果保存 */
    nav_ = navcur;

    return GINS_STATUS_OK;
}


status_t Gins::process_gnss(void *gpos_buf, int gpos_size)
{
    /* 变量声明 */
    status_t        ret;
    Eigen::Vector3d larm;    
    Eigen::Matrix3d Cbn;
    Eigen::Vector3d Z;
    Eigen::Vector3d pos;
    Eigen::Vector3d gpos_ned;
    Eigen::MatrixXd I;
    Eigen::MatrixXd H; 
    Eigen::MatrixXd R;
    Eigen::MatrixXd K;

    /* 变量定义 */
    ret = GINS_STATUS_NONE;
    larm.setZero();
    Cbn.setZero();
    Z.setZero();
    pos.setZero();
    gpos_ned.setZero();
    I.resize(param_.state_dim, param_.state_dim);
    I.setIdentity();
    H.resize(3, param_.state_dim);
    H.setZero();
    R.resize(3, 3);
    R.setZero();
    K.resize(param_.state_dim, 3);
    K.setZero();
    
    /* 增加gnss数据 */
    add_gpos_data(gpos_buf, gpos_size);

    /* gnss数据处理，转换到ned */
    gpos_ned = earth_.global2local(param_.gnss_base, gpos_.blh);

    /* 初始化 */
    if (nav_.sol_status == GINS_SOLSTATUS_WAIT_ALIGN)
    {
        coarse_align();
        printf("coarse_align() finished\n");
        return GINS_STATUS_OK;
    }
    
    /* 观测值，杆臂补偿至RTK定位中心的位置差 */
    larm = param_.gnss_larm;
    Cbn = rot_.euler2rotmat(nav_.att);
    Z = (nav_.pos + Cbn * larm) - gpos_ned;

    /* 观测矩阵 */
    H.block<3, 3>(0, DP_ID) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(0, DA_ID) = rot_.vec2skewsmt(Cbn * larm);

    /* 观测噪声阵 */
    R       = gpos_.std.asDiagonal(); 

    /* 滤波更新 */
    K       = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
    X_      = X_ + K * (Z - H * X_);
    P_      = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();

    return ret;
}

status_t Gins::state_feedback()
{
    Eigen::Quaterniond qbn = Eigen::Quaterniond::Identity();
    nav_.pos -= X_.block<3, 1>(DP_ID, 0);
    nav_.vel -= X_.block<3, 1>(DV_ID, 0);
    qbn = rot_.rotvec2quaternion(X_.block<3, 1>(DA_ID, 0));
    qbn = qbn * rot_.euler2quaternion(nav_.att);
    nav_.att = rot_.quaternion2euler(qbn);
    log_.ba += X_.block<3, 1>(DBA_ID, 0);
    log_.bg += X_.block<3, 1>(DBG_ID, 0);
    log_.sa += X_.block<3, 1>(DSA_ID, 0);
    log_.sg += X_.block<3, 1>(DSG_ID, 0);

    return GINS_STATUS_OK;
}


status_t Gins::gins_return_output(void *outbuf, int outsize)
{
    gins_nav_t gins_nav;
    Eigen::Matrix4d Tbn;
    ginslib_navout_t nav_output;

    gins_nav = nav_;
    Tbn.block<3, 3>(0, 0) = rot_.euler2rotmat(gins_nav.att);
    Tbn.block<3, 1>(0, 3) = gins_nav.pos.transpose();
    nav_output.time_stamp = gins_nav.time_stamp;
    memcpy(&nav_output.pos, gins_nav.pos.data(), 3 * sizeof(double));
    memcpy(&nav_output.vel, gins_nav.vel.data(), 3 * sizeof(double));
    memcpy(&nav_output.att, gins_nav.att.data(), 3 * sizeof(double));
    nav_output.att[0] *= RAD2DEG;
    nav_output.att[1] *= RAD2DEG;
    nav_output.att[2] *= RAD2DEG;
    nav_output.sol_status = (ginslib_sol_status_t)gins_nav.sol_status;
    nav_output.blh[3] = { 0 };
    nav_output.blh_valid_flg = false;
    memcpy(nav_output.Tbn, Tbn.data(), 4 * 4 * sizeof(double));

    memcpy(outbuf, &nav_output, outsize);
    return GINS_STATUS_OK;
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
status_t Gins::ginslib_process(const ginslib_prctype_t type, void* inbuf, int insize, void* outbuf, int outsize)
{
    /* 变量声明与定义 */
    status_t ret = GINS_STATUS_NONE;
    if (type == GINSLIB_PRCTYPE_IMU)
    {
        ret = process_imu(inbuf, insize);
    }
    if (type == GINSLIB_PRCTYPE_GNSSPOS)
    {
        ret = process_gnss(inbuf, insize);
    }

    gins_return_output(outbuf, outsize);
    return ret;
}

status_t Gins::ginslib_set_config(const ginslib_cfg_type_t type, void *inbuf, int insize)
{
    status_t ret = GINS_STATUS_NONE;
    ginslib_cfg_t cfg = { 0 };
    if (type == GINSLIB_CFG_TYPE_CUSTOM)
    {
        memcpy(&cfg, inbuf, insize);

        /* 将外部参数转化为内部参数格式 */
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
        
    }

    /* 完成配置后就设置算法内部参数和日志初值 */
    set_param();
    init_log_trace();
    
    return ret;
}
