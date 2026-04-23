/**
 * @file EoModel.cpp
 * @brief 光电（EO）模型 — C++ 实现，extern "C" 导出
 *
 * 生命周期（3 个函数）：Eo_Init → Eo_Process → Eo_Destroy
 */

#include "SensorModels.h"
#include "SensorModelsInternal.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

namespace {

struct EoContext {
    double horizontal_fov_rad;
    double vertical_fov_rad;
};

using sensor_models_detail::IsFiniteDouble;
using sensor_models_detail::MulRtVec;
using sensor_models_detail::RotZYXWorldFromBody;
using sensor_models_detail::VecLen;
using sensor_models_detail::VecSub;

} /* namespace */

extern "C" SENSOR_MODELS_API SensorResult Eo_Init(double horizontal_fov_rad,
                                                  double vertical_fov_rad,
                                                  void** out_handle) {
    if (out_handle == nullptr) {
        return SENSOR_ERR_NULL_PTR;
    }
    *out_handle = nullptr;
    if (horizontal_fov_rad <= 0.0 || vertical_fov_rad <= 0.0 ||
        !IsFiniteDouble(horizontal_fov_rad) || !IsFiniteDouble(vertical_fov_rad)) {
        return SENSOR_ERR_INVALID_ARG;
    }

    auto* ctx = new (std::nothrow) EoContext{};
    if (ctx == nullptr) {
        return SENSOR_ERR_BUFFER;
    }
    ctx->horizontal_fov_rad = horizontal_fov_rad;
    ctx->vertical_fov_rad = vertical_fov_rad;
    *out_handle = static_cast<void*>(ctx);
    return SENSOR_OK;
}

extern "C" SENSOR_MODELS_API SensorResult Eo_Process(void* handle,
                                                     const EoSensorState* sensor,
                                                     uint32_t target_count,
                                                     const EoTargetTruth* targets,
                                                     uint32_t max_out,
                                                     EoTargetImage* out_targets,
                                                     uint32_t* out_count) {
    if (out_count != nullptr) {
        *out_count = 0;
    }
    if (handle == nullptr || sensor == nullptr ||
        (target_count > 0 && targets == nullptr) || (max_out > 0 && out_targets == nullptr) ||
        out_count == nullptr) {
        return SENSOR_ERR_NULL_PTR;
    }

    auto* ctx = static_cast<EoContext*>(handle);
    const double half_h = 0.5 * ctx->horizontal_fov_rad;
    const double half_v = 0.5 * ctx->vertical_fov_rad;

    double R[9];
    RotZYXWorldFromBody(sensor->yaw_rad, sensor->pitch_rad, sensor->roll_rad, R);

    std::vector<EoTargetImage> tmp;
    tmp.reserve(static_cast<size_t>(target_count));

    for (uint32_t i = 0; i < target_count; ++i) {
        const EoTargetTruth& tgt = targets[i];
        const SensorVec3 d_w = VecSub(tgt.position, sensor->position);
        const double r = VecLen(d_w);
        if (r < 1e-9) {
            continue;
        }

        SensorVec3 u_w{d_w.x / r, d_w.y / r, d_w.z / r};
        const SensorVec3 u_b = MulRtVec(R, u_w);

        const double x = u_b.x;
        const double y = u_b.y;
        const double z = u_b.z;
        const double horiz = std::sqrt(x * x + y * y);
        const double rel_az = std::atan2(y, x);
        const double rel_el = std::atan2(-z, horiz);

        if (std::fabs(rel_az) > half_h + 1e-12 || std::fabs(rel_el) > half_v + 1e-12) {
            continue;
        }

        EoTargetImage im{};
        im.id = tgt.id;
        im.rel_azimuth_rad = rel_az;
        im.rel_elevation_rad = rel_el;
        tmp.push_back(im);
    }

    if (tmp.size() > static_cast<size_t>(max_out)) {
        return SENSOR_ERR_BUFFER;
    }
    for (std::size_t k = 0; k < tmp.size(); ++k) {
        out_targets[k] = tmp[k];
    }
    *out_count = static_cast<uint32_t>(tmp.size());
    return SENSOR_OK;
}

extern "C" SENSOR_MODELS_API void Eo_Destroy(void* handle) {
    if (handle == nullptr) {
        return;
    }
    delete static_cast<EoContext*>(handle);
}
