/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-07-22 11:27:41
 * @LastEditors: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @LastEditTime: 2023-09-03 22:23:56
 * @FilePath: /LC_GINS/include/gi_lib.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _GI_LIB_H_
#define _GI_LIB_H_

#include "common.h"
#include "rotation.h"
#include "gilib_interface.h"

class GILib
{
private:
    double time_stamp_;
    const int state_dim_ = STATE_DIM;
    int obs_dim_;
    /* 输入数据 */
    imu_t      imu_pre_;
    imu_t      imu_cur_;
    gnss_pos_t      gnss_pos_;

    imu_err_t  imu_err_;
    navflag_t  nav_flag_;
    errstate_t err_;
    navstate_t dr_nav_;
    navstate_t fusion_nav_;

    cfg_t cfg_;
    Rotation rot_;

    /* 卡尔曼滤波相关 */
    Eigen::Matrix<double, 21, 21> P_;       /* 状态协方差 */
    Eigen::Matrix<double, 21, 1 > X_;       /* 状态(dr{m},dv{m/s},da{rad},bg,ba,sg,sa) */
    

private:
    status_t set_initstate();
    status_t imu_mech();
    status_t imu_compensate();
    status_t process_imu();
    status_t process_gnss();
    status_t state_feedback();
    double getgravity();
public:
    GILib();
    status_t add_imu_data(void *imubuf, int size);
    status_t add_pos_data(void *posbuf, int size);
    status_t load_conf(const cfgtype_t type, void *inbuf, int insize);
    status_t process(const prctype_t type, void *inbuf, int insize, void *outbuf, int outsize);
    status_t return_nav(navstate_t &navstate);
};

#endif
