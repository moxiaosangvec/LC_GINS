/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-07-25 00:35:58
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2023-07-25 00:42:34
 * @FilePath: /LC_GINS/include/mathcal.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _MATHCAL_H_
#define _MATHCAL_H_

#include <Eigen/Core>

Eigen::Matrix3d vec2mat_skewsmt(const Eigen::Vector3d vec3)
{
    Eigen::Matrix3d mat3 = Eigen::Matrix3d::Zero();
    mat3 <<     0,      -vec3(2),   vec3(1),
            vec3(2),        0,      -vec3(0),
            -vec3(1),   vec3(0),    0;
    return mat3;
}

#endif
