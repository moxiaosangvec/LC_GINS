/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-07-22 11:27:41
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2023-07-26 22:34:01
 * @FilePath: /LC_GINS/include/gi_lib.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _GI_LIB_H_
#define _GI_LIB_H_

#include "common.h"

class GILib
{
private:
    double time_stamp_;
    const int state_dim_ = STATE_DIM;
    int obs_dim_;
    /* 输入数据 */
    imu_t      imu_pre_;
    imu_t      imu_cur_;
    gnss_t     gnss_;

    navflag_t  nav_flag_;
    errstate_t err_;
    navstate_t dr_nav_;
    navstate_t fusion_nav_;
    errcov_t   errcov_;

    cfg_t cfg_;
private:
    status_t imu_mech();
    status_t errstate_predict();
    status_t process_imu();
    status_t set_initstate();
    status_t process_gnss();
    double getgravity();
public:
    GILib();
    status_t load_conf(const cfgtype_t type, void *inbuf, int insize);
    status_t process(const prctype_t type, void *inbuf, int insize, void *outbuf, int outsize);
    status_t return_nav(navstate_t &navstate);
};

#endif
