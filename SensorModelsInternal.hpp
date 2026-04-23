/**
 * @file SensorModelsInternal.hpp
 * @brief 三模型共享的内部 C++ 工具（不导出、不包含在对外 API 中）
 */

#ifndef SENSOR_MODELS_INTERNAL_HPP
#define SENSOR_MODELS_INTERNAL_HPP

#include "SensorModels.h"

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sensor_models_detail {

constexpr double kSpeedOfLight = 299792458.0;

inline bool IsFiniteDouble(double v) { return std::isfinite(v); }

inline SensorVec3 VecSub(const SensorVec3& a, const SensorVec3& b) {
    return SensorVec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline double VecLen(const SensorVec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline double VecDot(const SensorVec3& a, const SensorVec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/** 球体遮挡视距：地心原点、半径 R；R<=0 时不裁剪 */
inline bool LineOfSightSphericalEarth(const SensorVec3& sensor,
                                      const SensorVec3& target,
                                      double earth_radius_m) {
    if (earth_radius_m <= 0.0 || !IsFiniteDouble(earth_radius_m)) {
        return true;
    }

    const double R = earth_radius_m;
    const double R2 = R * R;
    const double rs2 = VecDot(sensor, sensor);
    const double rt2 = VecDot(target, target);

    if (rs2 < R2 - 1.0 || rt2 < R2 - 1.0) {
        return false;
    }

    const SensorVec3 d = VecSub(target, sensor);
    const double a = VecDot(d, d);
    if (a <= 1e-24) {
        return true;
    }

    const double t0 = -VecDot(sensor, d) / a;
    double t_close = t0;
    if (t_close < 0.0) {
        t_close = 0.0;
    } else if (t_close > 1.0) {
        t_close = 1.0;
    }

    auto g_len2 = [&](double t) {
        const SensorVec3 p{sensor.x + t * d.x, sensor.y + t * d.y, sensor.z + t * d.z};
        return VecDot(p, p);
    };

    const double g_min = std::min(rs2, std::min(rt2, g_len2(t_close)));
    const double margin = 1.0;
    return !(g_min < R2 - margin);
}

inline double WattsToDbm(double p_w) {
    if (p_w <= 0.0 || !IsFiniteDouble(p_w)) {
        return -300.0;
    }
    return 10.0 * std::log10(p_w * 1000.0);
}

/** v_w = R * v_b，本库用 R^T 将世界视线变到机体 */
inline void RotZYXWorldFromBody(double yaw, double pitch, double roll, double out_R[9]) {
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);

    const double r00 = cy * cp;
    const double r01 = cy * sp * sr - sy * cr;
    const double r02 = cy * sp * cr + sy * sr;
    const double r10 = sy * cp;
    const double r11 = sy * sp * sr + cy * cr;
    const double r12 = sy * sp * cr - cy * sr;
    const double r20 = -sp;
    const double r21 = cp * sr;
    const double r22 = cp * cr;

    out_R[0] = r00;
    out_R[1] = r01;
    out_R[2] = r02;
    out_R[3] = r10;
    out_R[4] = r11;
    out_R[5] = r12;
    out_R[6] = r20;
    out_R[7] = r21;
    out_R[8] = r22;
}

inline SensorVec3 MulRtVec(const double R[9], const SensorVec3& v) {
    return SensorVec3{R[0] * v.x + R[3] * v.y + R[6] * v.z,
                      R[1] * v.x + R[4] * v.y + R[7] * v.z,
                      R[2] * v.x + R[5] * v.y + R[8] * v.z};
}

} /* namespace sensor_models_detail */

#endif
