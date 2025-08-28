/*==============================================================================
 * 过温回退处理服务实现
 * 功能：多传感器温度监控、状态机管理、智能功率回退算法
 *============================================================================*/

#include "overTemperatureHandler.h"
#include "faultManager.h"
#include "switchCtrl.h"
#include "database.h"
#include "dis_common_error_type.h"
#include <unistd.h>
#include <pthread.h>
#include "rtcDriver.h"
#include "elog_mtd.h"
#include "commonLog.h"

/*==============================================================================
 * 外部引用
 *============================================================================*/
// External mapping from carrier resource handler
extern dis_dfe8219_tx_mapping_t s_txMapping[MAX_TX_MCB_CNT];

/*==============================================================================
 * 全局变量定义
 *============================================================================*/

// ---- 传感器管理 ----
static sensor_attributes_t g_sensor_array[SENSOR_MAX];                    // 传感器属性数组
static uint8_t g_sensor_enable_flags[SENSOR_MAX];                         // 传感器启用标志数组
static read_temperature_func_t g_read_temperature_funcs[SENSOR_MAX];       // 温度读取函数指针数组

// ---- 通道管理 ----
static channel_t g_channels[MAX_ANT_COUNT];                               // 通道数组

// ---- 系统配置参数 ----
static request_shutdown_callback_t g_request_shutdown_cb = NULL;          // 请求关机回调
static uint8_t hysteresis_count = 3;                                     // 滞后计数阈值
static float TREC_MIN = 12.0f;                                           // 温度恢复最小保持时间（分钟）
static uint32_t dynamicBackoffPeriod = 300;                              // 温度传感器查询时间间隔（秒）
static float tmax_minutes = 6.0f;                                        // THO 的最大允许时长（分钟）
static float tdelta_minutes = 5.0f;                                      // 缓慢下降阶段总时长（分钟）
static float tempExtra = 5.0f;                                           // ETH 的附加温度裕度（°C）
static uint8_t g_enable_extended_backoff_pbo_calc = 1;                    // 是否在功率回退保持态下计算回退值

// ---- 功率回退参数 ----
static float g_pbo_step_size_db = 0.5f;                                  // PBO_OTH 的步进（dB）
static float g_pbo_max_attenuation_db = 3.0f;                            // 最大回退量（dB）
static float g_pbo_max_attenuation_extra_db = 1.0f;                      // 保持态可用的额外最大回退量（dB）

// ---- 多维数据数组 ----
// 每通道每传感器的数据（避免共享传感器跨通道相互影响）
static float g_channel_iho_accum[MAX_ANT_COUNT][SENSOR_MAX];              // I_HO 累计（°C*minute）
static float g_channel_sensor_pbo[MAX_ANT_COUNT][SENSOR_MAX];             // PBO_OTH 值（dB）
static uint8_t g_channel_sensor_calc_mask[MAX_ANT_COUNT][SENSOR_MAX];     // 回退计算掩码（1=参与，0=不参与）
static sensor_stage_t g_channel_sensor_stages[MAX_ANT_COUNT][SENSOR_MAX]; // 传感器当前阶段
static float g_channel_sensor_slowdrop_minutes[MAX_ANT_COUNT][SENSOR_MAX];// 缓慢下降阶段累计时间（分钟）
static float g_channel_sensor_slowdrop_tho_minutes[MAX_ANT_COUNT][SENSOR_MAX]; // 缓慢下降阶段 THO 累计
static float g_channel_sensor_slowdrop_iho_accum[MAX_ANT_COUNT][SENSOR_MAX];   // 缓慢下降阶段 I_HO 累计
static uint8_t g_channel_sensor_slowdrop_gate_open[MAX_ANT_COUNT][SENSOR_MAX]; // 缓慢下降阶段门禁标志

// ---- 数据库配置缓存 ----
static uint8_t g_channel_sensor_mask[MAX_ANT_COUNT][SENSOR_MAX];          // 通道传感器关联掩码
static uint32_t g_sensor_nth_threshold[SENSOR_MAX];                       // 传感器 NTH 阈值
static uint32_t g_sensor_hot_threshold[SENSOR_MAX];                       // 传感器 HOT 阈值
static uint32_t g_sensor_eth_threshold[SENSOR_MAX];                       // 传感器 ETH 阈值
static float g_sensor_iho_max_threshold[SENSOR_MAX];                      // 传感器 IHO_MAX 阈值

/*==============================================================================
 * 数据库初始化与配置加载
 *============================================================================*/

/**
 * @brief 初始化数据库并加载配置参数
 * @return DIS_COMMON_ERR_OK-成功，其他-失败
 */
uint8_t overTemperatureDbInit(void)
{
    int ret = 0;
    
    // 初始化数据库区域
    ret = dataBaseInitWithRegion(DFE8219, OVERTEMP);
    if (ret != NO_ERROR) {
        DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Error: Database init failed for region %u\n", OVERTEMP);
        return -1;
    }
    
    // 读取通道传感器关联掩码
    for(int i = 0; i < MAX_ANT_COUNT; i++){
        char key[32];
        sprintf(key, "/overTemp/channel%d", i);
        ret = dataBaseGetU8(DFE8219, OVERTEMP, key, g_channel_sensor_mask[i], SENSOR_MAX);
        if (ret != NO_ERROR) {
            DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Error: Database get failed for region %u\n", OVERTEMP);
            return -1;
        }
    }
    
    // 读取传感器阈值配置
    ret = dataBaseGetU32(DFE8219, OVERTEMP, "/overTemp/sensors/NTH", g_sensor_nth_threshold, SENSOR_MAX);
    if (ret != NO_ERROR) {
        DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Error: Database get NTH failed\n");
        return -1;
    }
    
    ret = dataBaseGetU32(DFE8219, OVERTEMP, "/overTemp/sensors/HOT", g_sensor_hot_threshold, SENSOR_MAX);
    if (ret != NO_ERROR) {
        DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Error: Database get HOT failed\n");
        return -1;
    }
    
    ret = dataBaseGetU32(DFE8219, OVERTEMP, "/overTemp/sensors/ETH", g_sensor_eth_threshold, SENSOR_MAX);
    if (ret != NO_ERROR) {
        DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Error: Database get ETH failed\n");
        return -1;
    }
    
    ret = dataBaseGetF32(DFE8219, OVERTEMP, "/overTemp/sensors/IHO_MAX", g_sensor_iho_max_threshold, SENSOR_MAX);
    if (ret != NO_ERROR) {
        DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Error: Database get IHO_MAX failed\n");
        return -1;
    }
    
    // 读取全局参数配置
    dataBaseGetF32(DFE8219, OVERTEMP, "/overTemp/global/Tdelta", &tdelta_minutes, 1);
    dataBaseGetU32(DFE8219, OVERTEMP, "/overTemp/global/dynamicBackoffPeriod", &dynamicBackoffPeriod, 1);
    dataBaseGetF32(DFE8219, OVERTEMP, "/overTemp/global/TREC_MIN", &TREC_MIN, 1);
    dataBaseGetU8(DFE8219, OVERTEMP, "/overTemp/global/hysteresis_count", &hysteresis_count, 1);
    dataBaseGetF32(DFE8219, OVERTEMP, "/overTemp/global/tmax", &tmax_minutes, 1);
    dataBaseGetF32(DFE8219, OVERTEMP, "/overTemp/global/tempExtra", &tempExtra, 1);
    
    // 读取功率回退参数（需要从0.1dB单位转换）
    uint8_t temp = 0;
    dataBaseGetU8(DFE8219, OVERTEMP, "/overTemp/global/maxAttenuation", &temp, 1);
    g_pbo_max_attenuation_db = temp / 10.0f;
    
    dataBaseGetU8(DFE8219, OVERTEMP, "/overTemp/global/stepSize", &temp, 1);
    g_pbo_step_size_db = temp / 10.0f;
    
    dataBaseGetU8(DFE8219, OVERTEMP, "/overTemp/global/maxAttenuationExtra", &temp, 1);
    g_pbo_max_attenuation_extra_db = temp / 10.0f;

    return DIS_COMMON_ERR_OK;
}

/*==============================================================================
 * 系统初始化函数
 *============================================================================*/

/**
 * @brief 将数据库中读取的阈值加载到全局传感器对象数组中，并复位相关计数
 */
void init_sensor_thresholds_from_db(void)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        sensor_attributes_t *sensor = &g_sensor_array[i];
        
        // 从数据库配置转换为实际阈值（0.1°C -> °C）
        sensor->nth_threshold = (float)g_sensor_nth_threshold[i] * 0.1f;
        sensor->hot_threshold = (float)g_sensor_hot_threshold[i] * 0.1f;
        sensor->eth_threshold = (float)g_sensor_eth_threshold[i] * 0.1f;
        sensor->iho_max_threshold = g_sensor_iho_max_threshold[i] * 0.1f;

        // 复位运行时数据
        sensor->current_temperature = 0.0f;
        sensor->over_nth_count = 0;
        sensor->under_nth_count = 0;
        sensor->over_hot_count = 0;
        sensor->under_hot_count = 0;
        sensor->over_eth_count = 0;
        sensor->under_eth_count = 0;
        sensor->over_eth_extra_count = 0;
        sensor->under_eth_extra_count = 0;
    }
}

/**
 * @brief 初始化传感器启用标志
 */
void init_sensor_enable_flags(void)
{
    // 清零所有传感器启用标志
    for (int sensor_index = 0; sensor_index < SENSOR_MAX; sensor_index++) {
        g_sensor_enable_flags[sensor_index] = 0;
    }

    // 若任一通道关联了该传感器，则将其启用标志置为 1
    for (int channel_id = 0; channel_id < MAX_ANT_COUNT; channel_id++) {
        for (int sensor_index = 0; sensor_index < SENSOR_MAX; sensor_index++) {
            if (g_channel_sensor_mask[channel_id][sensor_index]) {
                g_sensor_enable_flags[sensor_index] = 1;
            }
        }
    }
}

/**
 * @brief 初始化通道传感器关联
 */
void init_channel_sensors(void)
{
    // 遍历所有通道，根据 g_channel_sensor_mask 填充每个通道的传感器列表
    for (int channel_id = 0; channel_id < MAX_ANT_COUNT; channel_id++) {
        channel_t *channel = &g_channels[channel_id];
        channel->channel_id = (uint8_t)channel_id;
        channel->sensor_count = 0;
        channel->temp_handling_state = TEMP_STATE_NORMAL;
        
        // 先清空指针槽位
        for (int i = 0; i < MAX_SENSORS_PER_CHANNEL; i++) {
            channel->sensors[i] = NULL;
        }
        
        // 按掩码关联全局传感器对象
        for (int sensor_index = 0; sensor_index < SENSOR_MAX; sensor_index++) {
            if (g_channel_sensor_mask[channel_id][sensor_index]) {
                if (channel->sensor_count < MAX_SENSORS_PER_CHANNEL) {
                    channel->sensors[channel->sensor_count++] = &g_sensor_array[sensor_index];
                }
            }
        }
    }
}

/**
 * @brief 更新通道载波存在状态
 */
void update_channels_carrier_presence(void)
{
    /* For each TX channel (MCB), if all fbTx entries are INVALID_FB_ID (255),
     * treat the corresponding over-temp channel as having no carriers -> disable sensors. */
    for (int channel_id = 0; channel_id < MAX_ANT_COUNT; channel_id++) {
        int all_invalid = 1;
        if (channel_id < MAX_ANT_COUNT) {
            for (int i = 0; i < MAX_CARRIER_PER_BRANCH; i++) {
                if (s_txMapping[channel_id].fbTx[i] != INVALID_FB_ID) {
                    all_invalid = 0;
                    break;
                }
            }
        }
        channel_t *channel = &g_channels[channel_id];
        if (all_invalid) {
            channel->sensor_count = 0;
        } 
    }
}

/**
 * @brief 初始化传感器元数据和多维数据数组
 */
void init_sensor_metadata(void)
{
    // 预计算每个全局传感器的索引，写入其属性
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_sensor_array[i].sensor_index = i;
    }
    
    // 清零每通道每传感器的多维数据数组
    for (int ch = 0; ch < MAX_ANT_COUNT; ch++) {
        for (int i = 0; i < SENSOR_MAX; i++) {
            g_channel_iho_accum[ch][i] = 0.0f;
            g_channel_sensor_pbo[ch][i] = 0.0f;
            g_channel_sensor_calc_mask[ch][i] = 0;
            g_channel_sensor_stages[ch][i] = STAGE_INITIAL_BACKOFF;
            g_channel_sensor_slowdrop_minutes[ch][i] = 0.0f;
            g_channel_sensor_slowdrop_tho_minutes[ch][i] = 0.0f;
            g_channel_sensor_slowdrop_iho_accum[ch][i] = 0.0f;
            g_channel_sensor_slowdrop_gate_open[ch][i] = 0;
        }
    }
}

/*==============================================================================
 * 数据清理辅助函数
 *============================================================================*/

// 清零指定通道的所有 I_HO 累计
static inline void clear_channel_iho_accum(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_iho_accum[channel_id][i] = 0.0f;
    }
}

// 清理指定通道的所有 PBO_OTH 值
static inline void clear_channel_sensor_pbo(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_sensor_pbo[channel_id][i] = 0.0f;
    }
}

// 清理指定通道的所有 PBO_OTH 计算掩码
static inline void clear_channel_sensor_calc_mask(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_sensor_calc_mask[channel_id][i] = 0;
    }
}

// 清理指定通道的所有传感器阶段
static inline void clear_channel_sensor_stages(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_sensor_stages[channel_id][i] = STAGE_INITIAL_BACKOFF;
    }
}

// 清理指定通道的所有"缓慢下降阶段累计分钟数"
static inline void clear_channel_sensor_slowdrop_minutes(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_sensor_slowdrop_minutes[channel_id][i] = 0.0f;
    }
}

// 清理指定通道的缓慢下降阶段度量数据
static inline void clear_channel_sensor_slowdrop_metrics(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_sensor_slowdrop_tho_minutes[channel_id][i] = 0.0f;
        g_channel_sensor_slowdrop_iho_accum[channel_id][i] = 0.0f;
        g_channel_sensor_slowdrop_gate_open[channel_id][i] = 0;
    }
}

// 标记进入 Back-Off 时哪些传感器需要计算回退（温度当前超过 Hot）
static inline void mark_channel_sensor_calc_mask(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    uint8_t channel_id = channel->channel_id;
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        int sensor_index = sensor->sensor_index;
        g_channel_sensor_calc_mask[channel_id][sensor_index] =
            (sensor->current_temperature > sensor->hot_threshold) ? 1 : 0;
    }
}

// 按通道-传感器维度在一个周期内累积 I_HO
static void accumulate_channel_iho_tick(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    float minutes_per_tick = (float)dynamicBackoffPeriod / 60.0f;
    if (minutes_per_tick <= 0.0f) {
        return;
    }
    uint8_t channel_id = channel->channel_id;
    // 累计 THO（分钟）
    channel->tho_minutes += minutes_per_tick;
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        int sensor_index = sensor->sensor_index;
        float delta = sensor->current_temperature - sensor->nth_threshold;
        g_channel_iho_accum[channel_id][sensor_index] += delta * minutes_per_tick;
    }
}

/*==============================================================================
 * 温度采集与监控
 *============================================================================*/

/**
 * @brief 获取所有启用传感器的温度值
 */
void get_all_temperatures(void)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        if (g_sensor_enable_flags[i]) {
            // 通过函数指针数组调用对应的温度读取函数
            if (g_read_temperature_funcs[i] != NULL) {
                g_sensor_array[i].current_temperature = g_read_temperature_funcs[i]();
                DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "sensor_array[%d].current_temperature = %f\n", i, g_sensor_array[i].current_temperature);
            }
        }
    }
}

/**
 * @brief 更新所有启用传感器的阈值超限计数
 */
void update_all_sensors_threshold_counts(void)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        // 检查传感器是否启用
        if (g_sensor_enable_flags[i]){
            sensor_attributes_t* sensor = &g_sensor_array[i];
            
            // ---- NTH 阈值检查 ----
            if (sensor->current_temperature > sensor->nth_threshold) {
                if (sensor->over_nth_count < hysteresis_count) {
                    sensor->over_nth_count++;
                }
            } else {
                sensor->over_nth_count = 0;
            }

            if(sensor->current_temperature <= sensor->nth_threshold){
                if(sensor->under_nth_count < hysteresis_count){
                    sensor->under_nth_count++; 
                }
            }else{
                sensor->under_nth_count = 0;
            }

            // ---- HOT 阈值检查 ----
            if (sensor->current_temperature > sensor->hot_threshold) {
                if(sensor->over_hot_count < hysteresis_count){
                    sensor->over_hot_count++;
                }
            }else{
                sensor->over_hot_count = 0;
            }

            if(sensor->current_temperature <= sensor->hot_threshold){
                if(sensor->under_hot_count < hysteresis_count){
                    sensor->under_hot_count++;
                }
            }else{
                sensor->under_hot_count = 0;
            }

            // ---- ETH 阈值检查 ----
            if (sensor->current_temperature > sensor->eth_threshold) {
                if (sensor->over_eth_count < hysteresis_count) {
                    sensor->over_eth_count++;
                }
            } else {
                sensor->over_eth_count = 0;
            }

            if (sensor->current_temperature <= sensor->eth_threshold) {
                if (sensor->under_eth_count < hysteresis_count) {
                    sensor->under_eth_count++;
                }
            } else {
                sensor->under_eth_count = 0;
            }

            // ---- ETH + TempExtra 阈值检查 ----
            float temp_extra_limit = sensor->eth_threshold + tempExtra;
            if (sensor->current_temperature > temp_extra_limit) {
                if (sensor->over_eth_extra_count < hysteresis_count) {
                    sensor->over_eth_extra_count++;
                }
            } else {
                sensor->over_eth_extra_count = 0;
            }

            if (sensor->current_temperature <= temp_extra_limit) {
                if (sensor->under_eth_extra_count < hysteresis_count) {
                    sensor->under_eth_extra_count++;
                }
            } else {
                sensor->under_eth_extra_count = 0;
            }
        }
    }
}

/*==============================================================================
 * 状态转换检查函数
 *============================================================================*/

/**
 * @brief 检查从正常运行态到状态保持态的转换条件
 */
bool check_normal_to_holdoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return false;
    }
    
    // 检查该通道的所有传感器
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        // 连续超过NTH阈值达到迟滞次数
        if (sensor->over_nth_count >= hysteresis_count) {
            return true; // 满足跳转条件
        }
    }
    return false; // 未满足跳转条件
}

/**
 * @brief 检查从状态保持态到正常运行态的转换条件
 */
bool check_holdoff_to_normal_transition(channel_t* channel)
{
    if (channel == NULL) {
        return false;
    }

    // 规则：当且仅当所有关联传感器的 under_nth_count 均 > 0 时，开始/继续累计 TREC；
    // 满足 TREC 最小时间后，且所有传感器 under_nth_count 均 >= hysteresis_count 才允许恢复正常状态。
    bool all_under_positive = true;
    bool all_under_reached_hysteresis = true;
    
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        if (sensor->under_nth_count == 0) {
            all_under_positive = false;
        }
        if (sensor->under_nth_count < hysteresis_count) {
            all_under_reached_hysteresis = false;
        }
    }

    if (!all_under_positive) {
        // 有传感器未开始连续低于 NTH，TREC 清零
        channel->trec_counter = 0;
        return false;
    }

    // 计算达到最小保持时间所需的周期数（向上取整）
    uint32_t required_ticks = (TREC_MIN * 60 + dynamicBackoffPeriod - 1) / dynamicBackoffPeriod;
    if (channel->trec_counter < required_ticks) {
        channel->trec_counter++;
        return false;
    }

    // 仅当达到 TREC，且所有传感器满足迟滞计数，才恢复
    return all_under_reached_hysteresis;
}

/**
 * @brief 检查从状态保持态到功率回退态的转换条件
 */
bool check_holdoff_to_backoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return false;
    }
    return channel->ho2bo_counter >= hysteresis_count;
}

/**
 * @brief 更新状态保持态到功率回退态的转换计数器
 */
void update_holdoff_to_backoff_counter(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    
    // 条件1：任一传感器温度结果满足 temp_result(i) > Hot
    bool any_hot = false;
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        if (sensor->current_temperature > sensor->hot_threshold) {
            any_hot = true;
            break;
        }
    }

    // 条件2：任一传感器 I_HO(i) > IHO_max（按通道-传感器维度累计）
    bool any_iho_over = false;
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        int sensor_index = sensor->sensor_index;
        if (g_channel_iho_accum[channel->channel_id][sensor_index] > sensor->iho_max_threshold) {
            any_iho_over = true;
            break;
        }
    }

    // 条件3：保持阶段时长 THO > T_max
    bool tho_over = (channel->tho_minutes > tmax_minutes);

    // 任一条件满足则计一次，否则清零
    bool any_condition = (any_hot || any_iho_over || tho_over);
    if (any_condition) {
        if (channel->ho2bo_counter < hysteresis_count) {
            channel->ho2bo_counter++;
        }
    } else {
        channel->ho2bo_counter = 0;
    }
}

/**
 * @brief 检查从功率回退态到状态保持态的转换条件
 */
bool check_backoff_to_holdoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return false;
    }
    
    // 所有传感器的 under_hot_count 均达到 hysteresis_count 才返回 true
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        if (sensor->under_hot_count < hysteresis_count) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 检查从功率回退态到正常运行态的转换条件
 */
bool check_backoff_to_normal_transition(channel_t* channel)
{
    if (channel == NULL) {
        return false;
    }
    
    // 规则：当且仅当所有关联传感器的 under_nth_count 均 > 0 时，开始/继续累计 TREC；
    // 满足 TREC 最小时间后，且所有传感器 under_nth_count 均 >= hysteresis_count 才允许恢复正常状态。
    bool all_under_positive = true;
    bool all_under_reached_hysteresis = true;
    
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        if (sensor->under_nth_count == 0) {
            all_under_positive = false;
        }
        if (sensor->under_nth_count < hysteresis_count) {
            all_under_reached_hysteresis = false;
        }
    }

    if (!all_under_positive) {
        // 有传感器未开始连续低于 NTH，TREC 清零
        channel->trec_counter = 0;
        return false;
    }

    // 计算达到最小保持时间所需的周期数（向上取整）
    uint32_t required_ticks = (TREC_MIN * 60 + dynamicBackoffPeriod - 1) / dynamicBackoffPeriod;
    if (channel->trec_counter < required_ticks) {
        channel->trec_counter++;
        return false;
    }

    // 仅当达到 TREC，且所有传感器满足迟滞计数，才恢复
    return all_under_reached_hysteresis;
}

/**
 * @brief 检查从功率回退态到功率回退保持态的转换条件
 */
bool check_backoff_to_extended_backoff_transition(channel_t* channel)
{
    // 任一传感器连续 Hysteresis 次满足 temp_result(i) > ETH，则 Back-Off -> Extended Back-Off
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        if (sensor->over_eth_count >= hysteresis_count) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 检查从功率回退保持态到功率回退态的转换条件
 */
bool check_extended_backoff_to_backoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return false;
    }
    
    // 统一采用计数：所有传感器的 under_eth_count 均达到 hysteresis_count
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        if (sensor->under_eth_count < hysteresis_count) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 检查从功率回退保持态到请求关PA的转换条件
 */
bool check_extended_backoff_to_request_paoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return false;
    }
    
    // 统一采用计数：任一传感器 over_eth_extra_count 达到 hysteresis_count
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        if (sensor->over_eth_extra_count >= hysteresis_count) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 检查从请求关PA到请求关机的转换条件
 */
bool check_request_paoff_to_request_shutdown_transition(channel_t* channel)
{
    if (channel == NULL) {
        return false;
    }
    
    // 任一传感器连续 Hysteresis 次满足 temp_result(i) > ETH + TempExtra
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        if (sensor->over_eth_extra_count >= hysteresis_count) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 检查从请求关PA到功率回退保持态的转换条件
 */
bool check_request_paoff_to_extended_backoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return false;
    }
    
    // 所有传感器连续 Hysteresis 次满足 temp_result(i) <= ETH + TempExtra
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        if (sensor->under_eth_extra_count < hysteresis_count) {
            return false;
        }
    }
    return true;
}

/*==============================================================================
 * 状态转换处理函数
 *============================================================================*/

/**
 * @brief 处理从正常运行态到状态保持态的转换
 */
void handle_normal_to_holdoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    
    // 进行状态切换
    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "Channel %d: Transition from Normal Operation to Hold-Off state\n", channel->channel_id);
    channel_set_temp_state(channel, TEMP_STATE_HOLD_OFF);
    
    // 重置计数器和累计值
    channel->tho_minutes = 0.0f;
    channel->trec_counter = 0;
    channel->ho2bo_counter = 0;
    
    // 进入 Hold-Off 状态时，清零该通道所有 I_HO 累计（不影响其他通道）
    clear_channel_iho_accum(channel->channel_id);
    
    // 上报告警：一般过温（Normal high temp Over Threshold）
    dis_dfe_faultRaise(FM_ID_TEMP_NORMAL_OVER_THRESHOLD);
}

/**
 * @brief 处理从状态保持态到正常运行态的转换
 */
void handle_holdoff_to_normal_transition(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    
    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "Channel %d: Transition from Hold-Off to Normal Operation state\n", channel->channel_id);
    channel_set_temp_state(channel, TEMP_STATE_NORMAL);

    // 重置计数器和累计值
    channel->tho_minutes = 0.0f;
    channel->trec_counter = 0;
    channel->ho2bo_counter = 0;
    clear_channel_iho_accum(channel->channel_id);
    
    // 解除一般过温告警
    dis_dfe_faultCease(FM_ID_TEMP_NORMAL_OVER_THRESHOLD);
}

/**
 * @brief 处理从状态保持态到功率回退态的转换
 */
void handle_holdoff_to_backoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    
    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "Channel %d: Transition from Hold-Off to Back-Off state\n", channel->channel_id);
    channel_set_temp_state(channel, TEMP_STATE_BACK_OFF);
    
    // 重置计数器和累计值
    channel->tho_minutes = 0.0f;
    channel->trec_counter = 0;
    channel->ho2bo_counter = 0;
    
    // 清理回退相关数组
    clear_channel_iho_accum(channel->channel_id);
    clear_channel_sensor_pbo(channel->channel_id);
    clear_channel_sensor_calc_mask(channel->channel_id);
    clear_channel_sensor_stages(channel->channel_id);
    clear_channel_sensor_slowdrop_minutes(channel->channel_id);
    clear_channel_sensor_slowdrop_metrics(channel->channel_id);
    
    // 标记进入 Back-Off 时哪些传感器需要计算回退（温度当前超过 Hot）
    mark_channel_sensor_calc_mask(channel);
    
    // 上报告警：过温回退告警（Hot门限触发）
    dis_dfe_faultRaise(FM_ID_TEMP_HOT_OVER_THRESHOLD);
}

/**
 * @brief 处理从功率回退态到状态保持态的转换
 */
void handle_backoff_to_holdoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    
    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "Channel %d: Transition from Back-Off to Hold-Off state\n", channel->channel_id);
    channel_set_temp_state(channel, TEMP_STATE_HOLD_OFF);
    
    // 重置计数器和累计值
    channel->tho_minutes = 0.0f;
    channel->trec_counter = 0;
    channel->ho2bo_counter = 0;
    clear_channel_iho_accum(channel->channel_id);
    
    // 清理回退相关数组，避免残留影响后续再次进入 Back-Off 的计算
    clear_channel_sensor_pbo(channel->channel_id);
    clear_channel_sensor_calc_mask(channel->channel_id);
    clear_channel_sensor_stages(channel->channel_id);
    clear_channel_sensor_slowdrop_minutes(channel->channel_id);
    clear_channel_sensor_slowdrop_metrics(channel->channel_id);
    
    // 取消过温回退告警
    dis_dfe_faultCease(FM_ID_TEMP_HOT_OVER_THRESHOLD);
}

/**
 * @brief 处理从功率回退态到正常运行态的转换
 */
void handle_backoff_to_normal_transition(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    
    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "Channel %d: Transition from Back-Off to Normal Operation state\n", channel->channel_id);
    channel_set_temp_state(channel, TEMP_STATE_NORMAL);
    
    // 离开 Back-Off 时同步清理回退相关数组
    clear_channel_sensor_pbo(channel->channel_id);
    clear_channel_sensor_calc_mask(channel->channel_id);
    clear_channel_sensor_stages(channel->channel_id);
    clear_channel_sensor_slowdrop_minutes(channel->channel_id);
    clear_channel_sensor_slowdrop_metrics(channel->channel_id);
    
    // 解除过温回退告警
    dis_dfe_faultCease(FM_ID_TEMP_HOT_OVER_THRESHOLD);
    // 解除一般过温告警
    dis_dfe_faultCease(FM_ID_TEMP_NORMAL_OVER_THRESHOLD);
}

/**
 * @brief 处理从功率回退态到功率回退保持态的转换
 */
void handle_backoff_to_extended_backoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    
    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "Channel %d: Transition from Back-Off to Extended Back-Off state\n", channel->channel_id);
    channel_set_temp_state(channel, TEMP_STATE_EXTENDED_BACK_OFF);
    
    // 上报告警：严重过温告警（进入功率回退保持态）
    dis_dfe_faultRaise(FM_ID_TEMP_EXCEPTIONAL_HIGH);
}

/**
 * @brief 处理从功率回退保持态到功率回退态的转换
 */
void handle_extended_backoff_to_backoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    
    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "Channel %d: Transition from Extended Back-Off to Back-Off state\n", channel->channel_id);
    channel_set_temp_state(channel, TEMP_STATE_BACK_OFF);
    
    // 重置回退控制相关数组，以便重新进入区域1按 Hot 规则计算
    clear_channel_sensor_pbo(channel->channel_id);
    clear_channel_sensor_calc_mask(channel->channel_id);
    clear_channel_sensor_stages(channel->channel_id);
    clear_channel_sensor_slowdrop_minutes(channel->channel_id);
    clear_channel_sensor_slowdrop_metrics(channel->channel_id);
    
    // 重新标记需要参与回退计算的传感器（当前温度 > Hot）
    mark_channel_sensor_calc_mask(channel);
    
    // 解除严重过温告警
    dis_dfe_faultCease(FM_ID_TEMP_EXCEPTIONAL_HIGH);
}

/**
 * @brief 处理从功率回退保持态到请求关PA的转换
 */
void handle_extended_backoff_to_request_paoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    
    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "Channel %d: Transition from Extended Back-Off to Request PA OFF state\n", channel->channel_id);
    
    // 清理所有关联sensors的sensor->over_eth_extra_count
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        sensor->over_eth_extra_count = 0;
    }
    
    channel_set_temp_state(channel, TEMP_STATE_REQUEST_PA_OFF);
    
    // 触发对该通道的 PA 关闭，保持上一次的 PBO 值不变
    dis_dfe8219_swPaOff(channel->channel_id);
    
    // 上报告警：过温关PA
    dis_dfe_faultRaise(FM_ID_TEMP_PA_SHUTDOWN);
}

/**
 * @brief 处理从请求关PA到功率回退保持态的转换
 */
void handle_request_paoff_to_extended_backoff_transition(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    
    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "Channel %d: Transition from Request PA OFF to Extended Back-Off state\n", channel->channel_id);
    channel_set_temp_state(channel, TEMP_STATE_EXTENDED_BACK_OFF);
    
    // 重新打开该通道的 PA，保持回退值不变
    dis_dfe8219_swPaOn(channel->channel_id);
    
    // 清除"过温关PA"告警
    dis_dfe_faultCease(FM_ID_TEMP_PA_SHUTDOWN);
}

/**
 * @brief 处理从请求关PA到请求关机的转换
 */
void handle_request_paoff_to_request_shutdown_transition(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }
    
    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "Channel %d: Transition from Request PA OFF to Request Shutdown state\n", channel->channel_id);
    channel_set_temp_state(channel, TEMP_STATE_REQUEST_SHUTDOWN);
    
    // 清理温度相关历史告警
    dis_dfe_faultCease(FM_ID_TEMP_HOT_OVER_THRESHOLD);
    dis_dfe_faultCease(FM_ID_TEMP_EXCEPTIONAL_HIGH);
    dis_dfe_faultCease(FM_ID_TEMP_NORMAL_OVER_THRESHOLD);
    dis_dfe_faultCease(FM_ID_TEMP_PA_SHUTDOWN);
    
    // 上报告警：过温关机
    dis_dfe_faultRaise(FM_ID_OVER_TEMP_SHUTDOWN);
    
    // 写入elog：温度过高，过温关机，记录当前通道关联传感器的最高温度
    float highest_temp_c = 0.0f;
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        if (sensor->current_temperature > highest_temp_c) {
            highest_temp_c = sensor->current_temperature;
        }
    }
    ELOG_WRITE(OVERTEMP_ELOG, "Over-temperature shutting down. Highest sensor temperature = %.2f C (channel %d)", highest_temp_c, channel->channel_id);
    
    // 触发系统下电流程（若注册）
    if (g_request_shutdown_cb) {
        (void)g_request_shutdown_cb();
    }
}

/*==============================================================================
 * 状态机控制主函数
 *============================================================================*/

/**
 * @brief 温度处理状态控制主函数
 */
void TempHandlingStateControl(void)
{
    for (int channel_id = 0; channel_id < MAX_ANT_COUNT; channel_id++) {
        channel_t* channel = &g_channels[channel_id];
        
        // 若该通道无任何关联传感器，跳过状态机与回退相关操作，并确保功率回退为0
        if (channel->sensor_count == 0) {
            channel->P_current = 0.0f;
            continue;
        }
        
        temp_handling_state_t current_state = channel->temp_handling_state;
        DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "channel %d: current_state = %d\n", channel->channel_id, current_state);
        
        // 根据当前状态进行switch控制
        switch (current_state) {
            case TEMP_STATE_NORMAL:
                // 检查是否需要从正常运行态跳转到状态保持态
                if (check_normal_to_holdoff_transition(channel)) {
                    handle_normal_to_holdoff_transition(channel);
                    break;
                }
                break;
                
            case TEMP_STATE_HOLD_OFF:
                // THO 与 I_HO 累计
                accumulate_channel_iho_tick(channel);
                
                // 检查是否需要从状态保持态跳转到正常运行态
                if (check_holdoff_to_normal_transition(channel)) {
                    handle_holdoff_to_normal_transition(channel);
                    break;
                }
                
                // 更新 Hold-Off -> Back-Off 连续计数并判断是否跳转
                update_holdoff_to_backoff_counter(channel);
                if(check_holdoff_to_backoff_transition(channel)){
                    handle_holdoff_to_backoff_transition(channel);
                    break;
                }
                break;
                
            case TEMP_STATE_BACK_OFF:
                // 检查是否需要从功率回退态跳转到状态保持态
                if (check_backoff_to_holdoff_transition(channel)) {
                    handle_backoff_to_holdoff_transition(channel);
                    break;
                }
                
                // 检查是否需要从功率回退态跳转到正常运行态
                if (check_backoff_to_normal_transition(channel)) {
                    handle_backoff_to_normal_transition(channel);
                    break;
                }
                
                // 检查是否需要从功率回退态跳转到功率回退保持状态
                if (check_backoff_to_extended_backoff_transition(channel)) {
                    handle_backoff_to_extended_backoff_transition(channel);
                    break;
                }
                break;
                
            case TEMP_STATE_EXTENDED_BACK_OFF:
                // 检查是否需要从功率回退保持状态跳转到功率回退态
                if (check_extended_backoff_to_backoff_transition(channel)) {
                    handle_extended_backoff_to_backoff_transition(channel);
                    break;
                }
                
                // 检查是否需要从功率回退保持状态跳转到请求关PA状态
                if (check_extended_backoff_to_request_paoff_transition(channel)) {
                    handle_extended_backoff_to_request_paoff_transition(channel);
                    break;
                }
                break;
                
            case TEMP_STATE_REQUEST_PA_OFF:
                // 检查是否需要从请求关PA状态跳转到功率回退保持态
                if (check_request_paoff_to_extended_backoff_transition(channel)) {
                    handle_request_paoff_to_extended_backoff_transition(channel);
                    break;
                }
                
                // 检查是否需要从请求关PA状态跳转到请求关机状态
                if (check_request_paoff_to_request_shutdown_transition(channel)) {
                    handle_request_paoff_to_request_shutdown_transition(channel);
                    break;
                }
                break;
                
            default:
                break;
        }
    }
}

/*==============================================================================
 * 功率回退计算辅助函数
 *============================================================================*/

// 计算某通道在本周期所有传感器 PBO_OTH(i) 的最大值
static float get_channel_max_pbo(channel_t* channel)
{
    if (channel == NULL) {
        return 0.0f;
    }
    float max_pbo = 0.0f;
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        int sensor_index = sensor->sensor_index;
        float value = g_channel_sensor_pbo[channel->channel_id][sensor_index];
        if (i == 0 || value > max_pbo) {
            max_pbo = value;
        }
    }
    return max_pbo;
}

// 将目标 PBO_OTH 经过步进限制后，更新到通道的 P_current（也作为下一周期的"上一值"）
static void update_channel_pbo(channel_t* channel, float target_pbo_db)
{
    if (channel == NULL) {
        return;
    }
    float previous_pbo_db = channel->P_current;
    float next_pbo_db = target_pbo_db;

    if (target_pbo_db > previous_pbo_db) {
        float limited_up = previous_pbo_db + g_pbo_step_size_db;
        next_pbo_db = (limited_up < target_pbo_db) ? limited_up : target_pbo_db;
    } else if (target_pbo_db < previous_pbo_db) {
        float limited_down = previous_pbo_db - g_pbo_step_size_db;
        next_pbo_db = (limited_down > target_pbo_db) ? limited_down : target_pbo_db;
    }

    channel->P_current = next_pbo_db;
}

// 数值限制辅助函数
static inline float clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

// 在缓慢下降阶段每个tick无条件累计阶段内的分钟数与IHO
static void accumulate_slowdrop_metrics_tick(uint8_t channel_id, sensor_attributes_t* sensor)
{
    if (sensor == NULL) {
        return;
    }
    int sensor_index = sensor->sensor_index;
    float minutes_per_tick = (float)dynamicBackoffPeriod / 60.0f;
    if (minutes_per_tick <= 0.0f) {
        return;
    }
    // (t - t2) 与 THO 阶段累计
    g_channel_sensor_slowdrop_minutes[channel_id][sensor_index] += minutes_per_tick;
    if (!g_channel_sensor_slowdrop_gate_open[channel_id][sensor_index]) {
        g_channel_sensor_slowdrop_tho_minutes[channel_id][sensor_index] += minutes_per_tick;
        float delta_over_nth = sensor->current_temperature - sensor->nth_threshold;
        g_channel_sensor_slowdrop_iho_accum[channel_id][sensor_index] += delta_over_nth * minutes_per_tick;
    }
}

// 计算初始回退阶段的PBO值
static void compute_pbo_initial_backoff(uint8_t channel_id, sensor_attributes_t* sensor)
{
    if (sensor == NULL) {
        return;
    }
    int sensor_index = sensor->sensor_index;
    float hot_threshold = sensor->hot_threshold;
    float eth_threshold = sensor->eth_threshold;
    float current_temp = sensor->current_temperature;

    if (current_temp > hot_threshold) {
        float denominator = (eth_threshold - hot_threshold);
        float ratio = 0.0f;
        if (denominator > 0.0f) {
            ratio = (current_temp - hot_threshold) / denominator;
        }
        ratio = clamp01(ratio);
        g_channel_sensor_pbo[channel_id][sensor_index] = g_pbo_max_attenuation_db * ratio;
    } else {
        g_channel_sensor_stages[channel_id][sensor_index] = STAGE_SLOW_DECREASE;
        // 进入缓慢下降阶段，初始化该传感器的 (t - t2)
        g_channel_sensor_slowdrop_minutes[channel_id][sensor_index] = 0.0f;
        // 清零慢速阶段的 tho 与 iho 度量
        g_channel_sensor_slowdrop_tho_minutes[channel_id][sensor_index] = 0.0f;
        g_channel_sensor_slowdrop_iho_accum[channel_id][sensor_index] = 0.0f;
        g_channel_sensor_slowdrop_gate_open[channel_id][sensor_index] = 0;
    }
}

// 计算缓慢下降阶段的PBO值
static void compute_pbo_slow_decrease(uint8_t channel_id, sensor_attributes_t* sensor)
{
    if (sensor == NULL) {
        return;
    }
    int sensor_index = sensor->sensor_index;

    float t_minus_t2 = g_channel_sensor_slowdrop_minutes[channel_id][sensor_index];
    float Tdelta = (float)tdelta_minutes;

    float Hot = sensor->hot_threshold;
    float NTH = sensor->nth_threshold;
    float ETH = sensor->eth_threshold;

    float holdoff_temp_t = Hot - ((Hot - NTH) / Tdelta) * t_minus_t2;
    float delta_T_t = (ETH - Hot) - ((ETH + NTH - 2.0f * Hot) / Tdelta) * t_minus_t2;

    float numerator = sensor->current_temperature - holdoff_temp_t;
    float ratio = 0.0f;
    if (delta_T_t > 0.0f) {
        ratio = numerator / delta_T_t;
    }
    ratio = clamp01(ratio);
    g_channel_sensor_pbo[channel_id][sensor_index] = g_pbo_max_attenuation_db * ratio;

    // 超过缓慢下降阶段总时长后，进入稳定控制阶段
    if (g_channel_sensor_slowdrop_minutes[channel_id][sensor_index] >= Tdelta) {
        g_channel_sensor_stages[channel_id][sensor_index] = STAGE_STABLE_CONTROL;
    }
}

// 计算稳定控制阶段的PBO值
static void compute_pbo_stable_control(uint8_t channel_id, sensor_attributes_t* sensor)
{
    if (sensor == NULL) {
        return;
    }
    int sensor_index = sensor->sensor_index;
    float NTH = sensor->nth_threshold;
    float temp_val = sensor->current_temperature;

    if (temp_val > NTH) {
        float ratio = 0.0f;
        if (NTH > 0.0f) {
            ratio = (temp_val - NTH) / NTH;
        }
        ratio = clamp01(ratio);
        g_channel_sensor_pbo[channel_id][sensor_index] = g_pbo_max_attenuation_db * ratio;
    } else {
        // 结束该传感器的回退值计算
        g_channel_sensor_pbo[channel_id][sensor_index] = 0.0f;
        g_channel_sensor_calc_mask[channel_id][sensor_index] = 0;
    }
}

/*==============================================================================
 * 功率回退计算主函数
 *============================================================================*/

/**
 * @brief 功率回退态下的功率回退值计算
 */
void calculate_power_backoff_in_backoff_state(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }

    uint8_t channel_id = channel->channel_id;
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        int sensor_index = sensor->sensor_index;
        if (g_channel_sensor_calc_mask[channel_id][sensor_index]) {
            DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "channel %d: sensor %d: g_channel_sensor_stages = %d\n", channel_id, sensor_index, g_channel_sensor_stages[channel_id][sensor_index]);
            
            switch(g_channel_sensor_stages[channel_id][sensor_index]){
                // 初始回退阶段（区域1，t1<t<t2）
                case STAGE_INITIAL_BACKOFF:
                    compute_pbo_initial_backoff(channel_id, sensor);
                    break;
                    
                // 缓慢下降阶段
                case STAGE_SLOW_DECREASE:
                {
                    // 每tick无条件累计慢速阶段的 THO/IHO
                    accumulate_slowdrop_metrics_tick(channel_id, sensor);
                    // 阶段内条件：若 I_HO > IHO_MAX 或 THO > T_max，则允许继续缓慢下降阶段的回退计算
                    bool allow_slow_calc = g_channel_sensor_slowdrop_gate_open[channel_id][sensor_index];
                    if (!allow_slow_calc) {
                        float iho_stage = g_channel_sensor_slowdrop_iho_accum[channel_id][sensor_index];
                        float tho_stage = g_channel_sensor_slowdrop_tho_minutes[channel_id][sensor_index];
                        if (iho_stage > sensor->iho_max_threshold || tho_stage > tmax_minutes) {
                            allow_slow_calc = true;
                            g_channel_sensor_slowdrop_gate_open[channel_id][sensor_index] = 1; // 触发后停止再累计 IHO/THO
                        }
                    }
                    if (allow_slow_calc) {
                        compute_pbo_slow_decrease(channel_id, sensor);
                    }
                }
                break;
                
                // 稳定控制阶段
                case STAGE_STABLE_CONTROL:
                    compute_pbo_stable_control(channel_id, sensor);
                    break;
                    
                default:
                case STAGE_MAX:
                    // 不应该到达这里，忽略
                    break;
            }
        }
    }

    // 取通道内所有传感器的最大值作为本周期的 PBO_OTH
    float pbo_oth_max_db = get_channel_max_pbo(channel);

    // 上一周期 PBO_OTH 与本周期的 PBO_OTH 对比，并按照步进限制更新衰减结果
    update_channel_pbo(channel, pbo_oth_max_db);
}

/**
 * @brief 功率回退保持态下的功率回退值计算
 */
void calculate_power_backoff_in_extended_backoff_state(channel_t* channel)
{
    if (channel == NULL) {
        return;
    }

    // Step3：如果关闭保持态回退值计算，则仅进行状态检查，不修改 P 值
    if (!g_enable_extended_backoff_pbo_calc) {
        // 不计算，直接按状态机流程在 TempHandlingStateControl 中检查跳转
        return;
    }

    // Step1：找到超过 ETH 的最高温度值，按公式计算目标回退值
    float max_temp_over_eth = 0.0f;     // temp_norm - ETH
    for (int i = 0; i < channel->sensor_count; i++) {
        sensor_attributes_t* sensor = channel->sensors[i];
        if (sensor == NULL) {
            continue;
        }
        float over = sensor->current_temperature - sensor->eth_threshold;
        if (over > max_temp_over_eth) {
            max_temp_over_eth = over;
        }
    }

    float ratio = max_temp_over_eth / tempExtra;
    ratio = clamp01(ratio);

    float pbo_oth_max_db = g_pbo_max_attenuation_db + g_pbo_max_attenuation_extra_db * ratio; //计算得到当前通道的功率回退值
    // 与 Back-Off 相同的步进限制更新，更新通道的功率回退值
    update_channel_pbo(channel, pbo_oth_max_db);
}

/**
 * @brief 功率回退值计算控制主函数
 */
void PowerBackoffCalculationControl(void)
{
    for (int channel_id = 0; channel_id < MAX_ANT_COUNT; channel_id++) {
        channel_t* channel = &g_channels[channel_id];
        
        // 若该通道无任何关联传感器，跳过功率回退计算，并确保功率回退为0
        if (channel->sensor_count == 0) {
            channel->P_current = 0.0f;
            DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "channel %d: P_current = %f\n", channel->channel_id, channel->P_current);
            continue;
        }
        
        temp_handling_state_t current_state = channel->temp_handling_state;
        
        // 根据当前状态进行switch控制
        switch (current_state) {
            case TEMP_STATE_EXTENDED_BACK_OFF:
                // 功率回退保持态下的功率回退值计算
                calculate_power_backoff_in_extended_backoff_state(channel);
                break;
                
            case TEMP_STATE_REQUEST_PA_OFF:
            case TEMP_STATE_REQUEST_SHUTDOWN:
                // 请求关PA态和请求关机态下不进行 PBO 计算
                break;
                
            default:
                // 其余状态（包括 Back-Off/返回 Hold-Off/Normal）仍需要持续计算并逐步回退
                calculate_power_backoff_in_backoff_state(channel);
                break;
        }
        
        DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 2, "channel %d: P_current = %f\n", channel->channel_id, channel->P_current);
    }
}

/*==============================================================================
 * 温度读取函数（模拟测试用）
 *============================================================================*/

float getTemperature1(void)
{
    static float simulated_temperature_c1 = 36.0f;

    simulated_temperature_c1 += 0.5f;

    float p_current_db = 0.0f;

    p_current_db = g_channels[0].P_current;
    simulated_temperature_c1 = simulated_temperature_c1 - p_current_db * 0.6f;
    return simulated_temperature_c1;
}

float getTemperature2(void)
{
    static float simulated_temperature_c2 = 36.0f;

    simulated_temperature_c2 += 0.5f;

    float p_current_db = 0.0f;

    p_current_db = g_channels[0].P_current;
    simulated_temperature_c2 = simulated_temperature_c2 - p_current_db * 0.8f;
    return simulated_temperature_c2;
}

/**
 * @brief 初始化温度读取函数指针数组
 */
void init_temperature_read_functions(void)
{
    // 按照枚举顺序设置函数指针
    g_read_temperature_funcs[SENSOR_DFE_TEMP] = NULL;
    g_read_temperature_funcs[SENSOR_AFE_TEMP] = NULL;
    g_read_temperature_funcs[SENSOR_BOARD_TEMP] = getTemperature1;
    g_read_temperature_funcs[SENSOR_FPA_TEMP] = NULL;
    g_read_temperature_funcs[SENSOR_DPA_TEMP] = NULL;
    g_read_temperature_funcs[SENSOR_TX_TEMP] = getTemperature2;
    g_read_temperature_funcs[SENSOR_TOR_TEMP] = NULL;
    g_read_temperature_funcs[SENSOR_RX_TEMP] = NULL;

    // 初始化传感器元数据（写入 sensor_index 并清零通道累计）
    init_sensor_metadata();
}

/*==============================================================================
 * 系统管理函数
 *============================================================================*/

/**
 * @brief 注册请求关机回调函数
 */
void register_request_shutdown_callback(request_shutdown_callback_t cb)
{
    g_request_shutdown_cb = cb;
}

/**
 * @brief 设置通道温度处理状态
 */
void channel_set_temp_state(channel_t* channel, temp_handling_state_t state)
{
    if (channel != NULL && state < TEMP_STATE_MAX) {
        channel->temp_handling_state = state;
    }
}

/*==============================================================================
 * 服务主循环
 *============================================================================*/

/**
 * @brief 过温处理服务回调函数（周期性执行）
 */
static void overtemp_service_callback(void)
{
    get_all_temperatures();
    update_all_sensors_threshold_counts();
    TempHandlingStateControl();
    PowerBackoffCalculationControl();
}

/**
 * @brief 启动过温处理服务
 */
int start_overtemp_service(void)
{
    printf("start_overtemp_service\n");

    // 启用调试日志
    setModuleTraceEn(OVERTEMP_SERVICE, 1);
    
    // 系统初始化流程
    init_temperature_read_functions();
    overTemperatureDbInit();
    init_sensor_thresholds_from_db();
    init_sensor_enable_flags();
    init_channel_sensors();
    update_channels_carrier_presence();

    // 上电判断温度是否低于eth
    get_all_temperatures();
    for(int i = 0; i < SENSOR_MAX; i++){
        if(g_sensor_enable_flags[i]){
            if(g_sensor_array[i].current_temperature > g_sensor_array[i].eth_threshold){
                if (g_request_shutdown_cb) {
                    (void)g_request_shutdown_cb();
                }
            } 
        }
    }

    // 注册定时服务
    int interval = dynamicBackoffPeriod;
    if (rtc_register_service(0, "overtemp", interval, overtemp_service_callback) != 0) {
        DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "rtc_register_service overtemp failed\n");
        return -1;
    }

    return 0;
}










