#include "overTemperatureHandler.h"
#include "overtempInternal.h"
#include "overtempUtils.h"
#include "overtempStateCheck.h"
#include "overtempPowerBackoff.h"
#include "overtempStateHandler.h"
#include "faultManager.h"
#include "switchCtrl.h"
#include "dis_dfe8219_dataBase.h"
#include "dis_dfe8219_board.h"
#include "dis_common_error_type.h"
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include "dis_dfe8219_log.h"

/*==============================================================================
 * 外部引用
 *============================================================================*/
// External mapping from carrier resource handler
extern dis_dfe8219_tx_mapping_t s_txMapping[MAX_TX_MCB_CNT];

// 请求关机回调函数
extern request_shutdown_callback_t g_request_shutdown_cb;

// 温度读取函数指针数组
static read_temperature_func_t g_read_temperature_funcs[SENSOR_MAX];

/*==============================================================================
 * 温度读取函数（模拟测试用）
 *============================================================================*/

 float getTemperature1(void)
 {
     static float simulated_temperature_c1 = 36.0f;
 
     simulated_temperature_c1 += 0.5f;
 
     float p_current_db = 0.0f;
 
     p_current_db = g_channels[0].P_current;
     simulated_temperature_c1 = simulated_temperature_c1 - p_current_db * 1.0f;
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
    ret = dis_dfe8219_dataBaseInitWithRegion(DFE8219, OVERTEMP);
    if (ret != NO_ERROR) {
        DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Error: Database init failed for region %u\n", OVERTEMP);
        return -1;
    }
    
    // 清零通道传感器关联掩码和传感器启用标志
    memset(g_channel_sensor_mask, 0, sizeof(g_channel_sensor_mask));
    memset(g_sensor_enable_flags, 0, sizeof(g_sensor_enable_flags));
    
    // 读取通道传感器关联配置，使用字符串匹配方法
    for(int channel_id = 0; channel_id < MAX_ANT_COUNT; channel_id++){
        char key[32];
        char sensorNames[MAX_SENSORS_PER_CHANNEL][DB_MAX_SINGLE_STR_SIZE];
        unsigned int actualSensorCount = 0;
        
        sprintf(key, "/overTemp/channel%d", channel_id);
        ret = dis_dfe8219_dataBaseGetStr(DFE8219, OVERTEMP, key, sensorNames, MAX_SENSORS_PER_CHANNEL, &actualSensorCount);
        
        if (ret == NO_ERROR) {
            // 根据传感器名称设置掩码和启用标志
            for (unsigned int i = 0; i < actualSensorCount; i++) {
                int sensor_index = get_sensor_index_by_name(sensorNames[i]);
                if (sensor_index >= 0) {
                    g_channel_sensor_mask[channel_id][sensor_index] = 1;
                    g_sensor_enable_flags[sensor_index] = 1; // 同时设置启用标志
                    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 1, "Channel %d: Mapped sensor %s(index %d)\n", 
                                    channel_id, sensorNames[i], sensor_index);
                } else if(sensor_index == -2){
                    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 1, "Channel %d: No sensor mapped\n", channel_id);
                } else {
                    DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Warning: Unknown sensor name '%s' in channel %d\n", 
                                    sensorNames[i], channel_id);
                }
            }
        } else if (ret != ITEM_NOT_FOUND) {
            DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Error: Failed to get sensor names for channel %d\n", channel_id);
            return -1;
        }
    }
    
    // 只为启用的传感器加载阈值配置
    for (int sensor_index = 0; sensor_index < SENSOR_MAX; sensor_index++) {
        if (g_sensor_enable_flags[sensor_index]) {
            if (load_sensor_thresholds_from_db(sensor_index) != 0) {
                DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Error: Failed to load thresholds for sensor %s\n", 
                                g_sensor_names[sensor_index]);
                return -1;
            }
        }
    }
    
    // 读取全局参数配置
    dis_dfe8219_dataBaseGetF32(DFE8219, OVERTEMP, "/overTemp/global/Tdelta", &tdelta_seconds, 1);
    dis_dfe8219_dataBaseGetU32(DFE8219, OVERTEMP, "/overTemp/global/dynamicBackoffPeriod", &dynamicBackoffPeriod, 1);
    dis_dfe8219_dataBaseGetF32(DFE8219, OVERTEMP, "/overTemp/global/TREC_MIN", &TREC_min_seconds, 1);
    dis_dfe8219_dataBaseGetU8(DFE8219, OVERTEMP, "/overTemp/global/hysteresis_count", &hysteresis_count, 1);
    dis_dfe8219_dataBaseGetF32(DFE8219, OVERTEMP, "/overTemp/global/tmax", &tmax_seconds, 1);
    dis_dfe8219_dataBaseGetF32(DFE8219, OVERTEMP, "/overTemp/global/tempExtra", &tempExtra, 1);
    
    // 将秒转换为分钟
    tdelta_minutes = tdelta_seconds / 60.0f;
    tmax_minutes = tmax_seconds / 60.0f;
    
    // 读取功率回退参数（需要从0.1dB单位转换）
    uint8_t temp = 0;
    dis_dfe8219_dataBaseGetU8(DFE8219, OVERTEMP, "/overTemp/global/maxAttenuation", &temp, 1);
    g_pbo_max_attenuation_db = temp / 10.0f;
    
    dis_dfe8219_dataBaseGetU8(DFE8219, OVERTEMP, "/overTemp/global/stepSize", &temp, 1);
    g_pbo_step_size_db = temp / 10.0f;
    
    dis_dfe8219_dataBaseGetU8(DFE8219, OVERTEMP, "/overTemp/global/maxAttenuationExtra", &temp, 1);
    g_pbo_max_attenuation_extra_db = temp / 10.0f;

    return DIS_COMMON_ERR_OK;
}

/*==============================================================================
 * 系统初始化函数
 *============================================================================*/

/**
 * @brief 初始化通道传感器关联
 */
static void init_channel_sensors(void)
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
static void update_channels_carrier_presence(void)
{
    /* For each TX channel (MCB), if all fbTx entries are INVALID_FB_ID (255),
     * treat the corresponding over-temp channel as having no carriers -> disable sensors. */
    for (int channel_id = 0; channel_id < MAX_ANT_COUNT; channel_id++) {
        int all_invalid = 1;
        // 检查该通道的所有载波是否都无效
        for (int i = 0; i < MAX_CARRIER_PER_BRANCH; i++) {
            if (s_txMapping[channel_id].fbTx[i] != INVALID_FB_ID) {
                all_invalid = 0;
                break;
            }
        }
        
        channel_t *channel = &g_channels[channel_id];
        if (all_invalid) {
            // 如果所有载波都无效，则禁用该通道的传感器
            channel->sensor_count = 0;
            DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 1, "Channel %d: No valid carriers, disabling temperature sensors\n", channel_id);
        } 
    }
}

/**
 * @brief 初始化传感器元数据和多维数据数组
 */
static void init_sensor_metadata(void)
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

/**
 * @brief 初始化温度读取函数指针数组
 */
static void init_temperature_read_functions(void)
{
    // 按照枚举顺序设置函数指针
    g_read_temperature_funcs[SENSOR_DFE0] = getTemperature1;
    g_read_temperature_funcs[SENSOR_AFE0] = NULL;
    g_read_temperature_funcs[SENSOR_BOARD0] = getTemperature1;
    g_read_temperature_funcs[SENSOR_FPA0] = NULL;
    g_read_temperature_funcs[SENSOR_DPA0] = NULL;
    g_read_temperature_funcs[SENSOR_DPA1] = getTemperature2;
    g_read_temperature_funcs[SENSOR_TX0] = getTemperature2;
    g_read_temperature_funcs[SENSOR_TOR0] = NULL;
    g_read_temperature_funcs[SENSOR_RX0] = NULL;
}

/*==============================================================================
 * 温度采集与监控
 *============================================================================*/

/**
 * @brief 获取所有启用传感器的温度值
 */
static void get_all_temperatures(void)
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
static void update_all_sensors_threshold_counts(void)
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
 * 状态机控制主函数
 *============================================================================*/

/**
 * @brief 温度处理状态控制主函数
 */
static void TempHandlingStateControl(void)
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
 * 功率回退计算主函数
 *============================================================================*/

/**
 * @brief 功率回退值计算控制主函数
 */
static void PowerBackoffCalculationControl(void)
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
    init_sensor_metadata();
    overTemperatureDbInit();
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
                break;
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

/**
 * @brief 获取指定通道的当前功率回退值
 * @param channel_id 通道ID (0 ~ MAX_ANT_COUNT-1)
 * @return 当前功率回退值(dB)，通道ID无效时返回0.0f
 */
float get_channel_power_backoff(unsigned int channel_id)
{
    // 参数校验
    if (channel_id >= MAX_ANT_COUNT) {
        DEBUG_LOG_SAMPLE(OVERTEMP_SERVICE, 0, "Error: Invalid channel_id %u, max allowed is %u\n", 
                        channel_id, MAX_ANT_COUNT - 1);
        return 0.0f;
    }
    
    // 返回指定通道的当前功率回退值
    float power_backoff = g_channels[channel_id].P_current;
    
    return power_backoff;
}










