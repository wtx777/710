/**
 * @file demo_sensor_models.cpp
 * @brief 最小可运行示例：演示 Radar / ESM / EO 的 Init → Process → Destroy
 */

#include "SensorModels.h"

#include <cstdio>
#include <cmath>
#include <clocale>

#ifdef _WIN32
#include <windows.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/** Windows 控制台默认多为 GBK；源文件为 UTF-8（/utf-8），启动时切到 UTF-8 避免中文乱码 */
static void setup_console_utf8(void) {
#ifdef _WIN32
    SetConsoleOutputCP(65001);
    SetConsoleCP(65001);
#endif
    std::setlocale(LC_ALL, ".UTF8");
}

static void print_radar_demo(void) {
    void* h = nullptr;
    /* 10 MHz 带宽 → 瑞利距离分辨约 c/(2B) ≈ 15 m */
    SensorResult r = Radar_Init(10e6, 0.0, &h);
    if (r != SENSOR_OK || h == nullptr) {
        std::printf("Radar_Init 失败: %d\n", static_cast<int>(r));
        return;
    }

    RadarSensorState sensor{};
    sensor.position = SensorVec3{0, 0, 1000};

    /* 两个目标方位相近、距离差小于分辨单元，Process 中会合并为一条 */
    RadarTargetTruth targets[2] = {
        {1, {5000, 0, 1000}, 1.0},
        {2, {5010, 0, 1000}, 2.0}, /* 与目标 1 斜距差约 10 m < 15 m */
    };

    RadarMeasurement out[8];
    uint32_t n = 0;
    r = Radar_Process(h, &sensor, 2, targets, 8, out, &n);
    if (r != SENSOR_OK) {
        std::printf("Radar_Process 失败: %d\n", static_cast<int>(r));
        Radar_Destroy(h);
        return;
    }

    std::printf("--- 雷达示例（2 个真值 → 合并后 %u 条测量）---\n", n);
    for (uint32_t i = 0; i < n; ++i) {
        std::printf("  id=%d  range=%.2f m  az=%.4f rad  el=%.4f rad\n", static_cast<int>(out[i].id),
                    out[i].range_m, out[i].azimuth_rad, out[i].elevation_rad);
    }
    Radar_Destroy(h);
}

static void print_esm_demo(void) {
    void* h = nullptr;
    /* -90 dBm 灵敏度；频率测量固定 +1 kHz 偏差 */
    SensorResult r = Esm_Init(-90.0, 1000.0, &h);
    if (r != SENSOR_OK || h == nullptr) {
        std::printf("Esm_Init 失败: %d\n", static_cast<int>(r));
        return;
    }

    EsmSensorState sensor{};
    sensor.position = SensorVec3{0, 0, 0};

    EsmEmitterTruth emitters[2] = {
        {101, {10000, 0, 0}, 3e9, 50.0},   /* 10 km, 3 GHz, 50 W — 应能截获 */
        {102, {1e7, 0, 0}, 3e9, 0.01},     /* 10000 km — 过弱，应被剔除 */
    };

    EsmIntercept out[8];
    uint32_t n = 0;
    r = Esm_Process(h, &sensor, 2, emitters, 8, out, &n);
    if (r != SENSOR_OK) {
        std::printf("Esm_Process 失败: %d\n", static_cast<int>(r));
        Esm_Destroy(h);
        return;
    }

    std::printf("--- ESM 示例（2 个辐射源 → 高于灵敏度 %u 条）---\n", n);
    for (uint32_t i = 0; i < n; ++i) {
        std::printf("  emitter_id=%d  DOA_az=%.4f  DOA_el=%.4f  f=%.0f Hz  Pr=%.2f dBm\n",
                    static_cast<int>(out[i].emitter_id), out[i].doa_azimuth_rad,
                    out[i].doa_elevation_rad, out[i].measured_freq_hz, out[i].rx_power_dbm);
    }
    Esm_Destroy(h);
}

static void print_eo_demo(void) {
    void* h = nullptr;
    const double hfov = 60.0 * M_PI / 180.0;
    const double vfov = 40.0 * M_PI / 180.0;
    SensorResult r = Eo_Init(hfov, vfov, &h);
    if (r != SENSOR_OK || h == nullptr) {
        std::printf("Eo_Init 失败: %d\n", static_cast<int>(r));
        return;
    }

    EoSensorState sensor{};
    sensor.position = SensorVec3{0, 0, 10};
    sensor.yaw_rad = 0;
    sensor.pitch_rad = 0;
    sensor.roll_rad = 0;

    EoTargetTruth targets[2] = {
        {201, {1000, 50, 0}},  /* 应在视场内 */
        {202, {1000, 800, 0}}, /* 水平偏离大，应被 FOV 剔除 */
    };

    EoTargetImage out[8];
    uint32_t n = 0;
    r = Eo_Process(h, &sensor, 2, targets, 8, out, &n);
    if (r != SENSOR_OK) {
        std::printf("Eo_Process 失败: %d\n", static_cast<int>(r));
        Eo_Destroy(h);
        return;
    }

    std::printf("--- 光电示例（2 个目标 → FOV 内 %u 个）---\n", n);
    for (uint32_t i = 0; i < n; ++i) {
        std::printf("  id=%d  rel_az=%.4f rad  rel_el=%.4f rad\n", static_cast<int>(out[i].id),
                    out[i].rel_azimuth_rad, out[i].rel_elevation_rad);
    }
    Eo_Destroy(h);
}

int main(void) {
    setup_console_utf8();
    std::printf("SensorModels 试用程序\n\n");
    print_radar_demo();
    std::printf("\n");
    print_esm_demo();
    std::printf("\n");
    print_eo_demo();
    std::printf("\n完成。\n");
    return 0;
}
