#include "overtempStateHandler.h"
#include "overtempInternal.h"
#include "overtempUtils.h"
#include "faultManager.h"
#include "switchCtrl.h"
#include "dis_dfe8219_board.h"
#include "dis_dfe8219_log.h"
#include <stddef.h>

// 请求关机回调函数
request_shutdown_callback_t g_request_shutdown_cb = NULL;

/*==============================================================================
 * 状态转换处理函数实现
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