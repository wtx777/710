/**
 * @file EsmModel.cpp
 * @brief 电子侦察（ESM）模型 — C++ 实现，extern "C" 导出
 *
 * 生命周期（3 个函数）：Esm_Init → Esm_Process → Esm_Destroy
 */

#include "SensorModels.h"
#include "SensorModelsInternal.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

namespace {

struct EsmContext {
    double sensitivity_dbm;
    double meas_freq_error_hz;
};

using sensor_models_detail::IsFiniteDouble;
using sensor_models_detail::VecLen;
using sensor_models_detail::VecSub;
using sensor_models_detail::WattsToDbm;
using sensor_models_detail::kSpeedOfLight;

} /* namespace */

extern "C" SENSOR_MODELS_API SensorResult Esm_Init(double sensitivity_dbm,
                                                  double meas_freq_error_hz,
                                                  void** out_handle) {
    if (out_handle == nullptr) {
        return SENSOR_ERR_NULL_PTR;
    }
    *out_handle = nullptr;
    if (!IsFiniteDouble(sensitivity_dbm) || !IsFiniteDouble(meas_freq_error_hz)) {
        return SENSOR_ERR_INVALID_ARG;
    }

    auto* ctx = new (std::nothrow) EsmContext{};
    if (ctx == nullptr) {
        return SENSOR_ERR_BUFFER;
    }
    ctx->sensitivity_dbm = sensitivity_dbm;
    ctx->meas_freq_error_hz = meas_freq_error_hz;
    *out_handle = static_cast<void*>(ctx);
    return SENSOR_OK;
}

extern "C" SENSOR_MODELS_API SensorResult Esm_Process(void* handle,
                                                      const EsmSensorState* sensor,
                                                      uint32_t emitter_count,
                                                      const EsmEmitterTruth* emitters,
                                                      uint32_t max_intercepts,
                                                      EsmIntercept* out_intercepts,
                                                      uint32_t* out_count) {
    if (out_count != nullptr) {
        *out_count = 0;
    }
    if (handle == nullptr || sensor == nullptr ||
        (emitter_count > 0 && emitters == nullptr) ||
        (max_intercepts > 0 && out_intercepts == nullptr) || out_count == nullptr) {
        return SENSOR_ERR_NULL_PTR;
    }

    auto* ctx = static_cast<EsmContext*>(handle);

    std::vector<EsmIntercept> out;
    out.reserve(static_cast<size_t>(emitter_count));

    for (uint32_t i = 0; i < emitter_count; ++i) {
        const EsmEmitterTruth& em = emitters[i];
        if (em.frequency_hz <= 0.0 || !IsFiniteDouble(em.frequency_hz) ||
            em.tx_power_w < 0.0 || !IsFiniteDouble(em.tx_power_w)) {
            continue;
        }

        const SensorVec3 dvec = VecSub(em.position, sensor->position);
        const double d = VecLen(dvec);
        if (d < 1e-3) {
            continue;
        }

        const double lambda = kSpeedOfLight / em.frequency_hz;
        if (lambda <= 0.0 || !IsFiniteDouble(lambda)) {
            continue;
        }

        const double four_pi_d = 4.0 * M_PI * d;
        const double ratio = lambda / four_pi_d;
        const double pr_w = em.tx_power_w * ratio * ratio;
        const double pr_dbm = WattsToDbm(pr_w);

        if (pr_dbm < ctx->sensitivity_dbm) {
            continue;
        }

        const double horiz = std::sqrt(dvec.x * dvec.x + dvec.y * dvec.y);
        const double az = std::atan2(dvec.y, dvec.x);
        const double el = std::atan2(dvec.z, horiz);

        EsmIntercept z{};
        z.emitter_id = em.id;
        z.doa_azimuth_rad = az;
        z.doa_elevation_rad = el;
        z.measured_freq_hz = em.frequency_hz + ctx->meas_freq_error_hz;
        z.rx_power_dbm = pr_dbm;
        out.push_back(z);
    }

    if (out.size() > static_cast<size_t>(max_intercepts)) {
        return SENSOR_ERR_BUFFER;
    }
    for (std::size_t k = 0; k < out.size(); ++k) {
        out_intercepts[k] = out[k];
    }
    *out_count = static_cast<uint32_t>(out.size());
    return SENSOR_OK;
}

extern "C" SENSOR_MODELS_API void Esm_Destroy(void* handle) {
    if (handle == nullptr) {
        return;
    }
    delete static_cast<EsmContext*>(handle);
}
