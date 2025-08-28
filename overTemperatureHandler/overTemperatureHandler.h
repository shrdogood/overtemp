#ifndef OVER_TEMPERATURE_HANDLER_H
#define OVER_TEMPERATURE_HANDLER_H

#include <stdint.h>
#include "dis_dfe8219_common_types.h"

/*==============================================================================
 * 常量定义
 *============================================================================*/
#define MAX_SENSORS_PER_CHANNEL    8    // 每个通道最大传感器数量

/*==============================================================================
 * 类型定义
 *============================================================================*/
// 温度读取函数指针类型定义
typedef float (*read_temperature_func_t)(void);

// 关机处理回调函数指针类型
typedef void (*request_shutdown_callback_t)(void);

/*==============================================================================
 * 枚举定义
 *============================================================================*/
/**
 * @brief 传感器类型枚举
 */
typedef enum {
    SENSOR_DFE_TEMP = 0,      // DFE芯片
    SENSOR_AFE_TEMP,          // AFE芯片
    SENSOR_BOARD_TEMP,        // 板温
    SENSOR_FPA_TEMP,          // 末级PA sensor温度
    SENSOR_DPA_TEMP,          // PA driver sensor温度
    SENSOR_TX_TEMP,           // TX的sensor温度
    SENSOR_TOR_TEMP,          // TOR的sensor温度
    SENSOR_RX_TEMP,           // RX的sensor温度
    SENSOR_MAX                // 传感器类型总数
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

/*==============================================================================
 * 结构体定义
 *============================================================================*/
/**
 * @brief 传感器属性结构体
 * @details 包含传感器的各种阈值属性和检测温度值
 */
typedef struct {
    // ---- 传感器基本信息 ----
    int16_t sensor_index;            // 该传感器在全局传感器数组中的索引
    
    // ---- 温度阈值配置 ----
    float nth_threshold;             // 常规高门限 (NTH) - Normal High Threshold  
    float hot_threshold;             // 回退触发温度门限 Hot
    float eth_threshold;             // 异常高门限 (ETH) - Abnormal High Threshold
    float iho_max_threshold;         // I_HO 累积上限（°C*minute）
    
    // ---- 保留阈值字段（暂未使用） ----
    float ntl_threshold;             // 常规低门限 (NTL) - Normal Low Threshold
    float etl_threshold;             // 异常低门限 (ETL) - Abnormal Low Threshold
    float holdoff_temp_threshold;    // Hold_off 的温度门限
    float holdoff_duration_threshold;// Hold_off 阶段的持续门限
    
    // ---- 运行时温度数据 ----
    float current_temperature;       // 当前检测的温度值
    
    // ---- 阈值超限计数器 ----
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
    // ---- 通道基本信息 ----
    uint8_t channel_id;              // 通道ID
    uint8_t sensor_count;            // 当前通道关联的传感器数量
    sensor_attributes_t *sensors[MAX_SENSORS_PER_CHANNEL]; // 传感器指针数组
    
    // ---- 状态管理 ----
    temp_handling_state_t temp_handling_state; // 温度处理状态
    
    // ---- 功率回退控制 ----
    float P_current;                 // 当前功率回退值（也作为下一周期计算的"上一值"使用）
    
    // ---- 时间计数器 ----
    uint32_t trec_counter;           // 状态保持态恢复计时计数器(TREC)
    float tho_minutes;               // 在 Hold-Off 状态下累计的时长 THO（单位：minute）
    uint8_t ho2bo_counter;           // Hold-Off -> Back-Off 的连续满足计数器（任一条件满足则计一次）
} channel_t;

/*==============================================================================
 * 外部函数声明
 *============================================================================*/
// 外部温度读取函数
float stmp75a_getTemperatureReg();

/*==============================================================================
 * 初始化函数
 *============================================================================*/
/**
 * @brief 启动过温处理服务
 * @return 0-成功，非0-失败
 */
int start_overtemp_service(void);

/**
 * @brief 初始化温度读取函数指针数组
 */
void init_temperature_read_functions(void);

/**
 * @brief 初始化传感器元数据
 * @details 预计算并写入 sensor_index，清零通道累计数据
 */
void init_sensor_metadata(void);

/**
 * @brief 将数据库中读取的阈值写入传感器对象
 */
void init_sensor_thresholds_from_db(void);

/**
 * @brief 注册请求关机回调函数
 * @param cb 关机回调函数指针
 */
void register_request_shutdown_callback(request_shutdown_callback_t cb);

/*==============================================================================
 * 温度采集与监控函数
 *============================================================================*/
/**
 * @brief 通过函数指针数组获取所有启用的传感器温度值
 */
void get_all_temperatures(void);

/**
 * @brief 获取所有启用的传感器温度值（备用接口）
 */
void get_all_enabled_temperatures();

/**
 * @brief 更新所有启用传感器的阈值超限计数
 * @details 遍历所有启用的传感器，检查当前温度与各种阈值的关系，更新相应的超限计数器
 */
void update_all_sensors_threshold_counts(void);

/**
 * @brief 更新通道载波存在状态
 * @details 检查每个 TX MCB 的 `fbTx` 列表，若某通道全部为 `INVALID_FB_ID(255)`，
 *          则将对应 `g_channels[channel_id].sensor_count` 置为 0，使该通道跳过温度处理与回退计算。
 */
void update_channels_carrier_presence(void);

/**
 * @brief 初始化传感器启用标志
 */
void init_sensor_enable_flags(void);

/**
 * @brief 初始化通道传感器关联
 */
void init_channel_sensors(void);

/*==============================================================================
 * 状态机控制函数
 *============================================================================*/
/**
 * @brief 温度处理状态控制主函数
 * @details 遍历所有通道，根据当前状态和温度条件进行状态切换
 */
void TempHandlingStateControl(void);

/**
 * @brief 设置通道温度处理状态
 * @param channel 通道指针
 * @param state 目标状态
 */
void channel_set_temp_state(channel_t* channel, temp_handling_state_t state);

/*==============================================================================
 * 状态转换检查函数
 *============================================================================*/
// Normal <-> Hold-Off 转换检查
bool check_normal_to_holdoff_transition(channel_t* channel);
bool check_holdoff_to_normal_transition(channel_t* channel);

// Hold-Off <-> Back-Off 转换检查
bool check_holdoff_to_backoff_transition(channel_t* channel);
void update_holdoff_to_backoff_counter(channel_t* channel);

// Back-Off 相关转换检查
bool check_backoff_to_holdoff_transition(channel_t* channel);
bool check_backoff_to_normal_transition(channel_t* channel);
bool check_backoff_to_extended_backoff_transition(channel_t* channel);

// Extended Back-Off 相关转换检查
bool check_extended_backoff_to_backoff_transition(channel_t* channel);
bool check_extended_backoff_to_request_paoff_transition(channel_t* channel);

// Request PA OFF 相关转换检查
bool check_request_paoff_to_extended_backoff_transition(channel_t* channel);
bool check_request_paoff_to_request_shutdown_transition(channel_t* channel);

/*==============================================================================
 * 状态转换处理函数
 *============================================================================*/
// Normal <-> Hold-Off 转换处理
void handle_normal_to_holdoff_transition(channel_t* channel);
void handle_holdoff_to_normal_transition(channel_t* channel);

// Hold-Off <-> Back-Off 转换处理
void handle_holdoff_to_backoff_transition(channel_t* channel);
void handle_backoff_to_holdoff_transition(channel_t* channel);

// Back-Off 相关转换处理
void handle_backoff_to_normal_transition(channel_t* channel);
void handle_backoff_to_extended_backoff_transition(channel_t* channel);

// Extended Back-Off 相关转换处理
void handle_extended_backoff_to_backoff_transition(channel_t* channel);
void handle_extended_backoff_to_request_paoff_transition(channel_t* channel);

// Request PA OFF 相关转换处理
void handle_request_paoff_to_extended_backoff_transition(channel_t* channel);
void handle_request_paoff_to_request_shutdown_transition(channel_t* channel);

/*==============================================================================
 * 功率回退计算函数
 *============================================================================*/
/**
 * @brief 功率回退值计算控制主函数
 * @details 遍历所有通道，根据当前状态进行相应的功率回退值计算
 */
void PowerBackoffCalculationControl(void);

/**
 * @brief 功率回退态下的功率回退值计算
 * @param channel 通道指针
 */
void calculate_power_backoff_in_backoff_state(channel_t* channel);

/**
 * @brief 功率回退保持态下的功率回退值计算
 * @param channel 通道指针
 */
void calculate_power_backoff_in_extended_backoff_state(channel_t* channel);

/*==============================================================================
 * 兼容性接口（保留但建议使用上述明确接口）
 *============================================================================*/
/**
 * @brief 检查并处理从正常运行态到状态保持态的跳转
 * @details 检查指定通道，如果满足跳转条件则进行状态切换
 * @param channel 要检查的通道指针
 * @return 返回是否发生状态切换
 */
bool check_and_handle_normal_to_holdoff_transition(channel_t* channel);

#endif // OVER_TEMPERATURE_HANDLER_H 
