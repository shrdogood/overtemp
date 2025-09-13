#ifndef OVERTEMP_INTERNAL_H
#define OVERTEMP_INTERNAL_H

#include <stdint.h>
#include "dis_dfe8219_common_types.h"


#define MAX_SENSORS_PER_CHANNEL    8    // 每个通道最大传感器数量

// 温度读取函数指针类型定义
typedef float (*read_temperature_func_t)(void);

/**
 * @brief 传感器实例枚举
 */
typedef enum {
    SENSOR_DFE0 = 0,          
    SENSOR_AFE0,              
    SENSOR_BOARD0,            
    SENSOR_FPA0,             
    SENSOR_DPA0,              
    SENSOR_DPA1,              
    SENSOR_TX0,              
    SENSOR_TOR0,              
    SENSOR_RX0,               
    SENSOR_MAX                // 传感器实例总数
} sensor_type_t;

/**
 * @brief 温度处理状态枚举
 */
typedef enum {
    TEMP_STATE_NORMAL = 0,           // 正常运行态（Normal Operation）
    TEMP_STATE_HOLD_OFF,             // 状态保持态（Hold-Off）
    TEMP_STATE_BACK_OFF,             // 功率回退态（Back-Off）
    TEMP_STATE_EXTENDED_BACK_OFF,    // 功率回退保持态（Extended Back Off）
    TEMP_STATE_REQUEST_PA_OFF,       // 请求关PA
    TEMP_STATE_REQUEST_SHUTDOWN,     // 请求关机
    TEMP_STATE_MAX                   // 状态总数
} temp_handling_state_t;

/**
 * @brief 传感器处理阶段枚举（用于Back-Off状态下的分阶段计算）
 */
typedef enum {
    STAGE_INITIAL_BACKOFF = 0,       // 初始回退阶段
    STAGE_SLOW_DECREASE,             // 缓慢下降阶段
    STAGE_STABLE_CONTROL,            // 稳定控制阶段
    STAGE_MAX
} sensor_stage_t;

/**
 * @brief 传感器属性结构体
 * @details 包含传感器的各种阈值属性和检测温度值
 */
typedef struct {
    int16_t sensor_index;            // 该传感器在全局传感器数组中的索引
    
    float nth_threshold;             // 常规高门限 (NTH) - Normal High Threshold  
    float hot_threshold;             // 回退触发温度门限 Hot
    float eth_threshold;             // 异常高门限 (ETH) - Abnormal High Threshold
    float iho_max_threshold;         // I_HO 累积上限（°C*minute）
    
    float ntl_threshold;             // 常规低门限 (NTL) - Normal Low Threshold
    float etl_threshold;             // 异常低门限 (ETL) - Abnormal Low Threshold
    float holdoff_temp_threshold;    // Hold_off 的温度门限
    float holdoff_duration_threshold;// Hold_off 阶段的持续门限
    
    float current_temperature;       // 当前检测的温度值
    
    uint8_t over_nth_count;          // 连续超过NTH的次数
    uint8_t under_nth_count;         // 连续低于NTH的次数
    uint8_t over_hot_count;          // 连续超过Hot的次数
    uint8_t under_hot_count;         // 连续低于Hot的次数
    uint8_t over_eth_count;          // 连续超过ETH的次数
    uint8_t under_eth_count;         // 连续低于或等于ETH的次数
    uint8_t over_eth_extra_count;    // 连续超过 ETH+TempExtra 的次数（保持态用）
    uint8_t under_eth_extra_count;   // 连续低于或等于 ETH+TempExtra 的次数（请求关PA->保持态用）
} sensor_attributes_t;

/**
 * @brief 通道结构体
 * @details 每个通道可以对应多个传感器
 */
typedef struct {
    uint8_t channel_id;              // 通道ID
    uint8_t sensor_count;            // 当前通道关联的传感器数量
    sensor_attributes_t *sensors[MAX_SENSORS_PER_CHANNEL]; // 传感器指针数组
    
    temp_handling_state_t temp_handling_state; // 温度处理状态
    
    float P_current;                 // 当前功率回退值（也作为下一周期计算的"上一值"使用）
    
    uint32_t trec_counter;           // 状态保持态恢复计时计数器(TREC)
    float tho_minutes;               // 在 Hold-Off 状态下累计的时长 THO（单位：minute）
    uint8_t ho2bo_counter;           // Hold-Off -> Back-Off 的连续满足计数器（任一条件满足则计一次）
} channel_t;

// ---- 传感器管理 ----
extern sensor_attributes_t g_sensor_array[SENSOR_MAX];                    // 传感器属性数组
extern uint8_t g_sensor_enable_flags[SENSOR_MAX];                         // 传感器启用标志数组
extern const char* g_sensor_names[SENSOR_MAX];                           // 传感器名称字符串数组

// ---- 通道管理 ----
extern channel_t g_channels[MAX_ANT_COUNT];                               // 通道数组

// ---- 系统配置参数 ----
extern uint8_t hysteresis_count;                                         // 滞后计数阈值
extern float TREC_min_seconds;                                           // 温度恢复最小保持时间（秒）
extern uint32_t dynamicBackoffPeriod;                                    // 温度传感器查询时间间隔（秒）
extern float tdelta_seconds;                                             // 温度传感器查询时间间隔（秒）
extern float tmax_seconds;                                               // THO 的最大允许时长（秒）
extern float tmax_minutes;                                               // THO 的最大允许时长（分钟）
extern float tdelta_minutes;                                             // 缓慢下降阶段总时长（分钟）
extern float tempExtra;                                                  // ETH 的附加温度裕度（°C）
extern uint8_t g_enable_extended_backoff_pbo_calc;                       // 是否在功率回退保持态下计算回退值

// ---- 功率回退参数 ----
extern float g_pbo_step_size_db;                                         // PBO_OTH 的步进（dB）
extern float g_pbo_max_attenuation_db;                                   // 最大回退量（dB）
extern float g_pbo_max_attenuation_extra_db;                             // 保持态可用的额外最大回退量（dB）

// ---- 多维数据数组 ----
// 每通道每传感器的数据（避免共享传感器跨通道相互影响）
extern float g_channel_iho_accum[MAX_ANT_COUNT][SENSOR_MAX];              // I_HO 累计（°C*minute）
extern float g_channel_sensor_pbo[MAX_ANT_COUNT][SENSOR_MAX];             // PBO_OTH 值（dB）
extern uint8_t g_channel_sensor_calc_mask[MAX_ANT_COUNT][SENSOR_MAX];     // 回退计算掩码（1=参与，0=不参与）
extern sensor_stage_t g_channel_sensor_stages[MAX_ANT_COUNT][SENSOR_MAX]; // 传感器当前阶段
extern float g_channel_sensor_slowdrop_minutes[MAX_ANT_COUNT][SENSOR_MAX];// 缓慢下降阶段累计时间（分钟）
extern float g_channel_sensor_slowdrop_tho_minutes[MAX_ANT_COUNT][SENSOR_MAX]; // 缓慢下降阶段 THO 累计
extern float g_channel_sensor_slowdrop_iho_accum[MAX_ANT_COUNT][SENSOR_MAX];   // 缓慢下降阶段 I_HO 累计
extern uint8_t g_channel_sensor_slowdrop_gate_open[MAX_ANT_COUNT][SENSOR_MAX]; // 缓慢下降阶段门禁标志

// ---- 数据库配置缓存 ----
extern uint8_t g_channel_sensor_mask[MAX_ANT_COUNT][SENSOR_MAX];          // 通道传感器关联掩码

#endif /* OVERTEMP_INTERNAL_H */ 