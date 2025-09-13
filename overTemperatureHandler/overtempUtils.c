#include "overtempUtils.h"
#include "overtempInternal.h"
#include "dis_dfe8219_log.h"
#include "dis_dfe8219_dataBase.h"
#include "dis_common_error_type.h"
#include <string.h>
#include <stdio.h>

/*==============================================================================
 * 数据清理函数实现
 *============================================================================*/

// 清零指定通道的所有 I_HO 累计
void clear_channel_iho_accum(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_iho_accum[channel_id][i] = 0.0f;
    }
}

// 清理指定通道的所有 PBO_OTH 值
void clear_channel_sensor_pbo(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_sensor_pbo[channel_id][i] = 0.0f;
    }
}

// 清理指定通道的所有 PBO_OTH 计算掩码
void clear_channel_sensor_calc_mask(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_sensor_calc_mask[channel_id][i] = 0;
    }
}

// 清理指定通道的所有传感器阶段
void clear_channel_sensor_stages(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_sensor_stages[channel_id][i] = STAGE_INITIAL_BACKOFF;
    }
}

// 清理指定通道的所有"缓慢下降阶段累计分钟数"
void clear_channel_sensor_slowdrop_minutes(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_sensor_slowdrop_minutes[channel_id][i] = 0.0f;
    }
}

// 清理指定通道的缓慢下降阶段度量数据
void clear_channel_sensor_slowdrop_metrics(uint8_t channel_id)
{
    for (int i = 0; i < SENSOR_MAX; i++) {
        g_channel_sensor_slowdrop_tho_minutes[channel_id][i] = 0.0f;
        g_channel_sensor_slowdrop_iho_accum[channel_id][i] = 0.0f;
        g_channel_sensor_slowdrop_gate_open[channel_id][i] = 0;
    }
}

/*==============================================================================
 * 数据标记和累计函数实现
 *============================================================================*/

// 标记进入 Back-Off 时哪些传感器需要计算回退（温度当前超过 Hot）
void mark_channel_sensor_calc_mask(channel_t* channel)
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
void accumulate_channel_iho_tick(channel_t* channel)
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
 * 更新计数器函数实现
 *============================================================================*/

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

/*==============================================================================
 * 数据库辅助函数实现
 *============================================================================*/

/**
 * @brief 根据传感器名称获取传感器索引
 */
int get_sensor_index_by_name(const char* sensor_name)
{
    if (sensor_name == NULL) {
        return -1;
    }
    
    for (int i = 0; i < SENSOR_MAX; i++) {
        if (strcmp(sensor_name, g_sensor_names[i]) == 0) {
            return i;
        }
    }
    
    if(strcmp(sensor_name, "NULL") == 0) {
        return -2;
    }
    return -1; // 未找到匹配的传感器名称
}

/**
 * @brief 从数据库读取单个传感器的阈值配置
 */
int load_sensor_thresholds_from_db(int sensor_index)
{
    if (sensor_index < 0 || sensor_index >= SENSOR_MAX) {
        return -1;
    }
    
    char key[64];
    int ret;
    
    // 构造传感器配置键名
    sprintf(key, "/overTemp/%s", g_sensor_names[sensor_index]);
    
    unsigned int vals[4] = {0};
    ret = dis_dfe8219_dataBaseGetU32(DFE8219, OVERTEMP, key, vals, 4);
    if (ret != NO_ERROR) {
        DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Error: Failed to get threshold U32 data for sensor %s\n", g_sensor_names[sensor_index]);
        return -1;
    }
    
    sensor_attributes_t *sensor = &g_sensor_array[sensor_index];
    sensor->nth_threshold = (float)vals[0] * 0.1f;
    sensor->hot_threshold = (float)vals[1] * 0.1f;
    sensor->eth_threshold = (float)vals[2] * 0.1f;
    sensor->iho_max_threshold = (float)vals[3] * 0.1f;
    
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
    
    return 0;
} 