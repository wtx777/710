/**
 * @file RadarModel.cpp
 * @brief 雷达模型 — 实现为 C++（std::vector、算法等），导出符号为 extern "C"
 *
 * 生命周期（3 个函数）：Radar_Init → Radar_Process → Radar_Destroy
 */

#include "SensorModels.h"
#include "SensorModelsInternal.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace {

struct RadarContext {
    double bandwidth_hz;
    double earth_radius_m;
};

struct RadarWorkItem {
    int32_t id;
    double range_m;
    double az_rad;
    double el_rad;
};

using sensor_models_detail::IsFiniteDouble;
using sensor_models_detail::LineOfSightSphericalEarth;
using sensor_models_detail::VecLen;
using sensor_models_detail::VecSub;
using sensor_models_detail::kSpeedOfLight;

} /* namespace */

extern "C" SENSOR_MODELS_API SensorResult Radar_Init(double bandwidth_hz,
                                                     double earth_radius_m,
                                                     void** out_handle) {
    if (out_handle == nullptr) {
        return SENSOR_ERR_NULL_PTR;
    }
    *out_handle = nullptr;
    if (bandwidth_hz <= 0.0 || !IsFiniteDouble(bandwidth_hz)) {
        return SENSOR_ERR_INVALID_ARG;
    }

    auto* ctx = new (std::nothrow) RadarContext{};
    if (ctx == nullptr) {
        return SENSOR_ERR_BUFFER;
    }
    ctx->bandwidth_hz = bandwidth_hz;
    if (earth_radius_m > 0.0 && IsFiniteDouble(earth_radius_m)) {
        ctx->earth_radius_m = earth_radius_m;
    } else {
        ctx->earth_radius_m = -1.0;
    }
    *out_handle = static_cast<void*>(ctx);
    return SENSOR_OK;
}

extern "C" SENSOR_MODELS_API SensorResult Radar_Process(void* handle,
                                                        const RadarSensorState* sensor,
                                                        uint32_t target_count,
                                                        const RadarTargetTruth* targets,
                                                        uint32_t max_measurements,
                                                        RadarMeasurement* out_measurements,
                                                        uint32_t* out_count) {
    if (out_count != nullptr) {
        *out_count = 0;
    }
    if (handle == nullptr || sensor == nullptr ||
        (target_count > 0 && targets == nullptr) ||
        (max_measurements > 0 && out_measurements == nullptr) || out_count == nullptr) {
        return SENSOR_ERR_NULL_PTR;
    }

    auto* ctx = static_cast<RadarContext*>(handle);
    if (ctx->bandwidth_hz <= 0.0) {
        return SENSOR_ERR_HANDLE;
    }

    const double delta_r = kSpeedOfLight / (2.0 * ctx->bandwidth_hz);

    std::vector<RadarWorkItem> visible;
    visible.reserve(static_cast<size_t>(target_count));

    for (uint32_t i = 0; i < target_count; ++i) {
        const RadarTargetTruth& tgt = targets[i];
        if (!IsFiniteDouble(tgt.position.x) || !IsFiniteDouble(tgt.position.y) ||
            !IsFiniteDouble(tgt.position.z)) {
            continue;
        }

        const SensorVec3 d = VecSub(tgt.position, sensor->position);
        const double r = VecLen(d);
        if (r < 1e-6) {
            continue;
        }

        if (!LineOfSightSphericalEarth(sensor->position, tgt.position, ctx->earth_radius_m)) {
            continue;
        }

        const double horiz = std::sqrt(d.x * d.x + d.y * d.y);
        const double az = std::atan2(d.y, d.x);
        const double el = std::atan2(d.z, horiz);

        RadarWorkItem w{};
        w.id = tgt.id;
        w.range_m = r;
        w.az_rad = az;
        w.el_rad = el;
        visible.push_back(w);
    }

    if (visible.empty()) {
        *out_count = 0;
        return SENSOR_OK;
    }

    std::sort(visible.begin(), visible.end(),
              [](const RadarWorkItem& a, const RadarWorkItem& b) { return a.range_m < b.range_m; });

    std::vector<RadarMeasurement> merged;
    merged.reserve(visible.size());

    std::size_t idx = 0;
    while (idx < visible.size()) {
        std::size_t j = idx + 1;
        double sum_w = 1.0;
        double r_acc = visible[idx].range_m;
        double az_acc = visible[idx].az_rad;
        double el_acc = visible[idx].el_rad;
        const int32_t rep_id = visible[idx].id;

        while (j < visible.size() &&
               (visible[j].range_m - visible[idx].range_m) <= delta_r + 1e-9) {
            r_acc += visible[j].range_m;
            az_acc += visible[j].az_rad;
            el_acc += visible[j].el_rad;
            sum_w += 1.0;
            ++j;
        }

        RadarMeasurement m{};
        m.id = rep_id;
        m.range_m = r_acc / sum_w;
        m.azimuth_rad = az_acc / sum_w;
        m.elevation_rad = el_acc / sum_w;
        merged.push_back(m);
        idx = j;
    }

    if (merged.size() > static_cast<size_t>(max_measurements)) {
        return SENSOR_ERR_BUFFER;
    }
    for (std::size_t k = 0; k < merged.size(); ++k) {
        out_measurements[k] = merged[k];
    }
    *out_count = static_cast<uint32_t>(merged.size());
    return SENSOR_OK;
}

extern "C" SENSOR_MODELS_API void Radar_Destroy(void* handle) {
    if (handle == nullptr) {
        return;
    }
    delete static_cast<RadarContext*>(handle);
}
