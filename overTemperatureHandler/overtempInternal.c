#include "overtempInternal.h"

sensor_attributes_t g_sensor_array[SENSOR_MAX];                    // 传感器属性数组
uint8_t g_sensor_enable_flags[SENSOR_MAX];                         // 传感器启用标志数组

// 传感器名称字符串数组
const char* g_sensor_names[SENSOR_MAX] = {
    "DFE0",      // SENSOR_DFE0
    "AFE0",      // SENSOR_AFE0
    "BOARD0",    // SENSOR_BOARD0
    "FPA0",      // SENSOR_FPA0
    "DPA0",      // SENSOR_DPA0
    "DPA1",      // SENSOR_DPA1
    "TX0",       // SENSOR_TX0
    "TOR0",      // SENSOR_TOR0
    "RX0"        // SENSOR_RX0
};

// ---- 通道管理 ----
channel_t g_channels[MAX_ANT_COUNT];                               // 通道数组

// ---- 系统配置参数 ----
uint8_t hysteresis_count = 3;                                     // 滞后计数阈值
float TREC_min_seconds = 720.0f;                                  // 温度恢复最小保持时间（秒）
uint32_t dynamicBackoffPeriod = 300;                              // 温度传感器查询时间间隔（秒）
float tdelta_seconds = 300.0f;                                    // 温度传感器查询时间间隔（秒）
float tmax_seconds = 360.0f;                                      // THO 的最大允许时长（秒）
float tmax_minutes = 6.0f;                                        // THO 的最大允许时长（分钟）
float tdelta_minutes = 5.0f;                                      // 缓慢下降阶段总时长（分钟）
float tempExtra = 5.0f;                                           // ETH 的附加温度裕度（°C）
uint8_t g_enable_extended_backoff_pbo_calc = 1;                    // 是否在功率回退保持态下计算回退值

// ---- 功率回退参数 ----
float g_pbo_step_size_db = 0.5f;                                  // PBO_OTH 的步进（dB）
float g_pbo_max_attenuation_db = 3.0f;                            // 最大回退量（dB）
float g_pbo_max_attenuation_extra_db = 1.0f;                      // 保持态可用的额外最大回退量（dB）

// 每通道每传感器的数据
float g_channel_iho_accum[MAX_ANT_COUNT][SENSOR_MAX];                       // I_HO 累计（°C*minute）
float g_channel_sensor_pbo[MAX_ANT_COUNT][SENSOR_MAX];                      // PBO_OTH 值（dB）
uint8_t g_channel_sensor_calc_mask[MAX_ANT_COUNT][SENSOR_MAX];              // 回退计算掩码（1=参与，0=不参与）
sensor_stage_t g_channel_sensor_stages[MAX_ANT_COUNT][SENSOR_MAX];          // 传感器当前阶段
float g_channel_sensor_slowdrop_minutes[MAX_ANT_COUNT][SENSOR_MAX];         // 缓慢下降阶段累计时间（分钟）
float g_channel_sensor_slowdrop_tho_minutes[MAX_ANT_COUNT][SENSOR_MAX];     // 缓慢下降阶段 THO 累计
float g_channel_sensor_slowdrop_iho_accum[MAX_ANT_COUNT][SENSOR_MAX];       // 缓慢下降阶段 I_HO 累计
uint8_t g_channel_sensor_slowdrop_gate_open[MAX_ANT_COUNT][SENSOR_MAX];     // 缓慢下降阶段门禁标志
uint8_t g_channel_sensor_mask[MAX_ANT_COUNT][SENSOR_MAX];                   // 通道传感器关联掩码 