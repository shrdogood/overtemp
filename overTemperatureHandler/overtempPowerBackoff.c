#include "overtempPowerBackoff.h"
#include "overtempInternal.h"
#include "dis_dfe8219_log.h"
#include <stdbool.h>


// 计算某通道所有传感器 PBO_OTH(i) 的最大值
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
static float clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

/*==============================================================================
 * 阶段性功率回退计算函数实现
 *============================================================================*/

// 在缓慢下降阶段每个tick累计阶段内的分钟数与IHO
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
 * 总功率回退计算函数实现
 *============================================================================*/

// 功率回退态下的功率回退值计算
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

// 功率回退保持态下的功率回退值计算
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