#include "overtempStateCheck.h"
#include "overtempInternal.h"
#include <stdbool.h>
#include <stddef.h>

/*==============================================================================
 * 状态转换检查函数实现
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
    uint32_t required_ticks = (TREC_min_seconds + dynamicBackoffPeriod - 1) / dynamicBackoffPeriod;
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
    uint32_t required_ticks = (TREC_min_seconds + dynamicBackoffPeriod - 1) / dynamicBackoffPeriod;
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