#ifndef OVER_TEMPERATURE_HANDLER_H
#define OVER_TEMPERATURE_HANDLER_H


/*==============================================================================
 * 外部接口函数
 *============================================================================*/

// 外部温度读取函数声明
float stmp75a_getTemperatureReg();

/**
 * @brief 启动过温处理服务
 * @return 0-成功，非0-失败
 */
int start_overtemp_service(void);

/**
 * @brief 获取指定通道的当前功率回退值
 * @param channel_id 通道ID (0 ~ MAX_ANT_COUNT-1)
 * @return 当前功率回退值(dB)，通道ID无效时返回0.0f
 */
float get_channel_power_backoff(unsigned int channel_id);


#endif // OVER_TEMPERATURE_HANDLER_H 
