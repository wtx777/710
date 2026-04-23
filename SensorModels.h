/**
 * @file SensorModels.h
 * @brief 跨平台传感器仿真模型库 — 纯 C 导出接口（雷达 / ESM / 光电）
 *
 * 约定：角度单位为弧度；位置为右手笛卡尔坐标（米），与具体仿真坐标系一致即可。
 */

#ifndef SENSOR_MODELS_H
#define SENSOR_MODELS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* 跨平台导出宏                                                               */
/* -------------------------------------------------------------------------- */
#if defined(_WIN32) || defined(__CYGWIN__)
#ifdef BUILDING_SENSOR_MODELS_DLL
#define EXPORT_API __declspec(dllexport)
#elif !defined(SENSOR_MODELS_STATIC)
#define EXPORT_API __declspec(dllimport)
#else
#define EXPORT_API
#endif
#else
#if defined(__GNUC__) && __GNUC__ >= 4
#define EXPORT_API __attribute__((visibility("default")))
#else
#define EXPORT_API
#endif
#endif

#ifndef SENSOR_MODELS_API
#define SENSOR_MODELS_API EXPORT_API
#endif

/* -------------------------------------------------------------------------- */
/* 通用 POD                                                                   */
/* -------------------------------------------------------------------------- */

/** 三维位置 / 向量（米） */
typedef struct SensorVec3 {
    double x;
    double y;
    double z;
} SensorVec3;

/** 通用错误码 */
typedef enum SensorResult {
    SENSOR_OK = 0,
    SENSOR_ERR_NULL_PTR = -1,
    SENSOR_ERR_INVALID_ARG = -2,
    SENSOR_ERR_BUFFER = -3,
    SENSOR_ERR_HANDLE = -4
} SensorResult;

/* ========================================================================== */
/* 雷达 Radar                                                                 */
/* ========================================================================== */
/*
 * 生命周期（3 个函数，按顺序调用）：
 *   Radar_Init → Radar_Process（可多次）→ Radar_Destroy
 *
 * 输入 / 输出（均为 POD，经指针传递）：
 *   Init  输入：bandwidth_hz, earth_radius_m；输出：out_handle。
 *   Process 输入：handle, sensor, target_count, targets[]；
 *           输出：out_measurements[0..*out_count-1], *out_count（需预先分配 max_measurements 容量）。
 *   Destroy 输入：handle（之后不得再使用）。
 */

typedef struct RadarSensorState {
    SensorVec3 position; /**< 雷达位置（米） */
} RadarSensorState;

typedef struct RadarTargetTruth {
    int32_t id;
    SensorVec3 position; /**< 目标位置（米） */
    double rcs_m2;         /**< 雷达截面积（平方米），当前极简模型未用于 SNR，仅保留字段 */
} RadarTargetTruth;

typedef struct RadarMeasurement {
    int32_t id;
    double range_m;      /**< 斜距（米） */
    double azimuth_rad;  /**< 方位角（弧度），在水平面内相对北向或 X 轴由实现约定：相对 X 轴 atan2(y,x) */
    double elevation_rad;/**< 俯仰角（弧度） */
} RadarMeasurement;

/**
 * @brief 创建雷达模型实例。
 * @param bandwidth_hz 信号带宽（Hz），用于瑞利距离分辨率 Δr = c/(2B)。
 * @param earth_radius_m 地球半径（米）。仅当地心坐标系且位置相对地心时启用球地遮挡视距；<=0 表示不启用球地遮挡（任意局部直角坐标系下建议传 0）。
 * @param out_handle 非空时写入句柄。
 * @return SENSOR_OK 或错误码。
 */
SENSOR_MODELS_API SensorResult Radar_Init(double bandwidth_hz,
                                           double earth_radius_m,
                                           void** out_handle);

/**
 * @brief 雷达探测处理：视距 + 距离向瑞利分辨率合并。
 * @param handle Radar_Init 返回的句柄。
 * @param sensor 雷达状态。
 * @param target_count 目标数量。
 * @param targets 目标真值数组，长度 >= target_count。
 * @param max_measurements 输出缓冲区容量。
 * @param out_measurements 输出测量数组。
 * @param out_count 成功时写入输出条数。
 */
SENSOR_MODELS_API SensorResult Radar_Process(void* handle,
                                             const RadarSensorState* sensor,
                                             uint32_t target_count,
                                             const RadarTargetTruth* targets,
                                             uint32_t max_measurements,
                                             RadarMeasurement* out_measurements,
                                             uint32_t* out_count);

SENSOR_MODELS_API void Radar_Destroy(void* handle);

/* ========================================================================== */
/* 电子侦察 ESM                                                               */
/* ========================================================================== */
/*
 * 生命周期：Esm_Init → Esm_Process → Esm_Destroy
 *
 * 输入 / 输出：
 *   Init  输入：sensitivity_dbm, meas_freq_error_hz；输出：out_handle。
 *   Process 输入：handle, sensor, emitter_count, emitters[]；
 *           输出：out_intercepts[], *out_count（容量 max_intercepts）。
 *   Destroy 输入：handle。
 */

typedef struct EsmSensorState {
    SensorVec3 position;
} EsmSensorState;

typedef struct EsmEmitterTruth {
    int32_t id;
    SensorVec3 position;
    double frequency_hz;   /**< 发射频率（Hz） */
    double tx_power_w;     /**< 辐射功率（瓦，各向同性等效） */
} EsmEmitterTruth;

typedef struct EsmIntercept {
    int32_t emitter_id;
    double doa_azimuth_rad;   /**< 到达角方位（弧度） */
    double doa_elevation_rad; /**< 到达角俯仰（弧度） */
    double measured_freq_hz;  /**< 测量频率（极简模型：无误差时为真值频率） */
    double rx_power_dbm;      /**< 接收功率（dBm） */
} EsmIntercept;

/**
 * @param sensitivity_dbm 接收机灵敏度（dBm），低于该值的截获结果被剔除。
 * @param meas_freq_error_hz 测量频率误差（Hz，绝对值叠加到输出频率上，可为 0）。
 */
SENSOR_MODELS_API SensorResult Esm_Init(double sensitivity_dbm,
                                        double meas_freq_error_hz,
                                        void** out_handle);

SENSOR_MODELS_API SensorResult Esm_Process(void* handle,
                                           const EsmSensorState* sensor,
                                           uint32_t emitter_count,
                                           const EsmEmitterTruth* emitters,
                                           uint32_t max_intercepts,
                                           EsmIntercept* out_intercepts,
                                           uint32_t* out_count);

SENSOR_MODELS_API void Esm_Destroy(void* handle);

/* ========================================================================== */
/* 光电 EO                                                                    */
/* ========================================================================== */
/*
 * 生命周期：Eo_Init → Eo_Process → Eo_Destroy
 *
 * 输入 / 输出：
 *   Init  输入：horizontal_fov_rad, vertical_fov_rad（全角，弧度）；输出：out_handle。
 *   Process 输入：handle, sensor（位姿 + 位置）, target_count, targets[]；
 *           输出：out_targets[], *out_count（容量 max_out）。
 *   Destroy 输入：handle。
 */

typedef struct EoSensorState {
    SensorVec3 position;
    double yaw_rad;   /**< 绕 Z 轴偏航（弧度） */
    double pitch_rad; /**< 绕 Y 轴俯仰（弧度） */
    double roll_rad;  /**< 绕 X 轴横滚（弧度） */
} EoSensorState;

typedef struct EoTargetTruth {
    int32_t id;
    SensorVec3 position;
} EoTargetTruth;

typedef struct EoTargetImage {
    int32_t id;
    double rel_azimuth_rad;   /**< 相对机体/载荷坐标系方位角（弧度） */
    double rel_elevation_rad; /**< 相对俯仰角（弧度） */
} EoTargetImage;

/**
 * @param horizontal_fov_rad 水平视场角全角（弧度）
 * @param vertical_fov_rad   垂直视场角全角（弧度）
 */
SENSOR_MODELS_API SensorResult Eo_Init(double horizontal_fov_rad,
                                      double vertical_fov_rad,
                                      void** out_handle);

SENSOR_MODELS_API SensorResult Eo_Process(void* handle,
                                          const EoSensorState* sensor,
                                          uint32_t target_count,
                                          const EoTargetTruth* targets,
                                          uint32_t max_out,
                                          EoTargetImage* out_targets,
                                          uint32_t* out_count);

SENSOR_MODELS_API void Eo_Destroy(void* handle);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MODELS_H */
