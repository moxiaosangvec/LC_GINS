
#ifndef EARTH_H
#define EARTH_H

#include <Eigen/Geometry>

/* WGS84椭球模型参数
   NOTE:如果使用其他椭球模型需要修改椭球参数 */
const double WGS84_WIE      = 7.2921151467E-5;       /* 地球自转角速度*/
const double WGS84_F        = 0.0033528106647474805; /* 扁率 */
const double WGS84_RA       = 6378137.0000000000;    /* 长半轴a */
const double WGS84_RB       = 6356752.3142451793;    /* 短半轴b */
const double WGS84_GM0      = 398600441800000.00;    /* 地球引力常数 */
const double WGS84_SQR_E1   = 0.0066943799901413156; /* 第一偏心率平方 */
const double WGS84_SQR_E2   = 0.0067394967422764341; /* 第二偏心率平方 */

typedef struct Pose
{
    /* data */
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
} Pose;

class Earth {

public:
    /* 正常重力计算 */
    static double gravity(const double lat, const double h) {

        double sin2 = sin(lat);
        sin2 *= sin2;

        return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) +
               h * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * h * h;
    }

    /* 计算子午圈半径 */
    static double get_Rm(double lat)
    {
        double tmp, sqrttmp;
        tmp = sin(lat);
        tmp *= tmp;
        tmp     = 1 - WGS84_SQR_E1 * tmp;
        sqrttmp = sqrt(tmp);
        return WGS84_RA * (1 - WGS84_SQR_E1) / (sqrttmp * tmp);
    }

    /* 计算卯酉圈半径 */
    static double get_Rn(double lat) {
        double sinlat = sin(lat);
        return WGS84_RA / sqrt(1.0 - WGS84_SQR_E1 * sinlat * sinlat);
    }

    /* n系(导航坐标系)到e系(地心地固坐标系)转换矩阵 */
    static Eigen::Matrix3d cne(const Eigen::Vector3d &blh) {
        double coslon, sinlon, coslat, sinlat;

        sinlat = sin(blh[0]);
        sinlon = sin(blh[1]);
        coslat = cos(blh[0]);
        coslon = cos(blh[1]);

        Eigen::Matrix3d dcm;
        dcm(0, 0) = -sinlat * coslon;
        dcm(0, 1) = -sinlon;
        dcm(0, 2) = -coslat * coslon;

        dcm(1, 0) = -sinlat * sinlon;
        dcm(1, 1) = coslon;
        dcm(1, 2) = -coslat * sinlon;

        dcm(2, 0) = coslat;
        dcm(2, 1) = 0;
        dcm(2, 2) = -sinlat;

        return dcm;
    }

    /* n系(导航坐标系)到e系(地心地固坐标系)转换四元数 */
    static Eigen::Quaterniond qne(const Eigen::Vector3d &blh) {
        Eigen::Quaterniond quat;

        double coslon, sinlon, coslat, sinlat;

        coslon = cos(blh[1] * 0.5);
        sinlon = sin(blh[1] * 0.5);
        coslat = cos(-M_PI * 0.25 - blh[0] * 0.5);
        sinlat = sin(-M_PI * 0.25 - blh[0] * 0.5);

        quat.w() = coslat * coslon;
        quat.x() = -sinlat * sinlon;
        quat.y() = sinlat * coslon;
        quat.z() = coslat * sinlon;

        return quat;
    }

    /* 从n系到e系转换四元数得到纬度和经度 */
    static Eigen::Vector3d blh(const Eigen::Quaterniond &qne, double height) {
        return {-2 * atan(qne.y() / qne.w()) - M_PI * 0.5, 2 * atan2(qne.z(), qne.w()), height};
    }

    /* 大地坐标(纬度、经度和高程)转地心地固坐标 */
    static Eigen::Vector3d blh2ecef(const Eigen::Vector3d &blh) {
        double coslat, sinlat, coslon, sinlon;
        double rnh, rn;

        coslat = cos(blh[0]);
        sinlat = sin(blh[0]);
        coslon = cos(blh[1]);
        sinlon = sin(blh[1]);

        rn  = get_Rn(blh[0]);
        rnh = rn + blh[2];

        return {rnh * coslat * coslon, rnh * coslat * sinlon, (rnh - rn * WGS84_SQR_E1) * sinlat};
    }

    /* 地心地固坐标转大地坐标 */
    static Eigen::Vector3d ecef2blh(const Eigen::Vector3d &ecef) {
        double p = sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1]);
        double rn;
        double lat, lon;
        double h = 0, h2;

        // 初始状态
        lat = atan(ecef[2] / (p * (1.0 - WGS84_SQR_E1)));
        lon = 2.0 * atan2(ecef[1], ecef[0] + p);

        do {
            h2  = h;
            rn  = get_Rn(lat);
            h   = p / cos(lat) - rn;
            lat = atan(ecef[2] / (p * (1.0 - WGS84_SQR_E1 * rn / (rn + h))));
        } while (fabs(h - h2) > 1.0e-4);

        return {lat, lon, h};
    }

    /* n系相对位置转大地坐标相对位置 */
    static Eigen::Matrix3d DRi(const Eigen::Vector3d &blh) {
        double rm = 0;
        double rn = 0;
        Eigen::Matrix3d dri = Eigen::Matrix3d::Zero();

        rm = get_Rm(blh[0]);
        rn = get_Rn(blh[0]);

        dri(0, 0) = 1.0 / (rm + blh[2]);
        dri(1, 1) = 1.0 / ((rn + blh[2]) * cos(blh[0]));
        dri(2, 2) = -1;
        return dri;
    }

    /* 大地坐标相对位置转n系相对位置 */
    static Eigen::Matrix3d DR(const Eigen::Vector3d &blh) {
        double rm = 0;
        double rn = 0;
        Eigen::Matrix3d dr = Eigen::Matrix3d::Zero();
    
        rm = get_Rm(blh[0]);
        rn = get_Rn(blh[0]);

        dr(0, 0) = rm + blh[2];
        dr(1, 1) = (rn + blh[2]) * cos(blh[0]);
        dr(2, 2) = -1;
        return dr;
    }

    /* 局部坐标(在origin处展开)转大地坐标 */
    static Eigen::Vector3d local2global(const Eigen::Vector3d &origin, const Eigen::Vector3d &local) {

        Eigen::Vector3d ecef0 = blh2ecef(origin);
        Eigen::Matrix3d cn0e  = cne(origin);

        Eigen::Vector3d ecef1 = ecef0 + cn0e * local;
        Eigen::Vector3d blh1  = ecef2blh(ecef1);

        return blh1;
    }

    /* 大地坐标转局部坐标(在origin处展开) */
    static Eigen::Vector3d global2local(const Eigen::Vector3d &origin, const Eigen::Vector3d &global) {
        Eigen::Vector3d ecef0 = blh2ecef(origin);
        Eigen::Matrix3d cn0e  = cne(origin);

        Eigen::Vector3d ecef1 = blh2ecef(global);

        return cn0e.transpose() * (ecef1 - ecef0);
    }

    static Pose local2global(const Eigen::Vector3d &origin, const Pose &local) {
        Pose global;

        Eigen::Vector3d ecef0 = blh2ecef(origin);
        Eigen::Matrix3d cn0e  = cne(origin);

        Eigen::Vector3d ecef1 = ecef0 + cn0e * local.t;
        Eigen::Vector3d blh1  = ecef2blh(ecef1);
        Eigen::Matrix3d cn1e  = cne(blh1);

        global.t = blh1;
        global.R = cn1e.transpose() * cn0e * local.R;

        return global;
    }

    static Pose global2local(const Eigen::Vector3d &origin, const Pose &global) {
        Pose local;

        Eigen::Vector3d ecef0 = blh2ecef(origin);
        Eigen::Matrix3d cn0e  = cne(origin);

        Eigen::Vector3d ecef1 = blh2ecef(global.t);
        Eigen::Matrix3d cn1e  = cne(global.t);

        local.t = cn0e.transpose() * (ecef1 - ecef0);
        local.R = cn0e.transpose() * cn1e * global.R;

        return local;
    }

    /* 地球自转角速度投影到e系 */
    static Eigen::Vector3d wiee() {
        return {0, 0, WGS84_WIE};
    }

    /* 地球自转角速度投影到n系 */
    static Eigen::Vector3d wien(double lat) {
        return {WGS84_WIE * cos(lat), 0, -WGS84_WIE * sin(lat)};
    }

    static Eigen::Vector3d wien(const Eigen::Vector3d &origin, const Eigen::Vector3d &local) {
        Eigen::Vector3d global = local2global(origin, local);

        return wien(global[0]);
    }

    /* n系相对于e系转动角速度投影到n系 */
    static Eigen::Vector3d wenn(const double rm, const double rn,
                                const Eigen::Vector3d &blh, 
                                const Eigen::Vector3d &vel) {
        Eigen::Vector3d wn = Eigen::Vector3d::Zero();
        wn(0) = vel[1] / (rn + blh[2]);
        wn(1) = -vel[0] / (rm + blh[2]);
        wn(2) = -vel[1] * tan(blh[0]) / (rn + blh[2]);
        return wn;
    }

    static Eigen::Vector3d wenn(const Eigen::Vector3d &origin, 
                                const Eigen::Vector3d &local, 
                                const Eigen::Vector3d &vel) {
        
        Eigen::Vector3d global     = local2global(origin, local);
        double rm = 0;
        double rn = 0;
        rm = get_Rm(global(0));
        rn = get_Rn(global(0));
        return wenn(rm, rn, global, vel);
    }
};

#endif // EARTH_H
