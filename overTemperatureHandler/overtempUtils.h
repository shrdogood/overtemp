#ifndef OVERTEMP_UTILS_H
#define OVERTEMP_UTILS_H

#include "overtempInternal.h"


/*==============================================================================
 * 数据清理函数
 *============================================================================*/

/**
 * @brief 清零指定通道的所有 I_HO 累计
 * @param channel_id 通道ID
 */
void clear_channel_iho_accum(uint8_t channel_id);

/**
 * @brief 清理指定通道的所有 PBO_OTH 值
 * @param channel_id 通道ID
 */
void clear_channel_sensor_pbo(uint8_t channel_id);

/**
 * @brief 清理指定通道的所有 PBO_OTH 计算掩码
 * @param channel_id 通道ID
 */
void clear_channel_sensor_calc_mask(uint8_t channel_id);

/**
 * @brief 清理指定通道的所有传感器阶段
 * @param channel_id 通道ID
 */
void clear_channel_sensor_stages(uint8_t channel_id);

/**
 * @brief 清理指定通道的所有"缓慢下降阶段累计分钟数"
 * @param channel_id 通道ID
 */
void clear_channel_sensor_slowdrop_minutes(uint8_t channel_id);

/**
 * @brief 清理指定通道的缓慢下降阶段度量数据
 * @param channel_id 通道ID
 */
void clear_channel_sensor_slowdrop_metrics(uint8_t channel_id);

/*==============================================================================
 * 数据标记和累计函数
 *============================================================================*/

/**
 * @brief 标记进入 Back-Off 时哪些传感器需要计算回退（温度当前超过 Hot）
 * @param channel 通道指针
 */
void mark_channel_sensor_calc_mask(channel_t* channel);

/**
 * @brief 按通道-传感器维度在一个周期内累积 I_HO
 * @param channel 通道指针
 */
void accumulate_channel_iho_tick(channel_t* channel);

/*==============================================================================
 * 更新计数器函数
 *============================================================================*/

/**
 * @brief 更新状态保持态到功率回退态的转换计数器
 * @param channel 通道指针
 */
void update_holdoff_to_backoff_counter(channel_t* channel);

/*==============================================================================
 * 数据库辅助函数
 *============================================================================*/

/**
 * @brief 根据传感器名称获取传感器索引
 * @param sensor_name 传感器名称字符串
 * @return 传感器索引，未找到返回-1
 */
int get_sensor_index_by_name(const char* sensor_name);

/**
 * @brief 从数据库读取单个传感器的阈值配置
 * @param sensor_index 传感器索引
 * @return 0-成功，其他-失败
 */
int load_sensor_thresholds_from_db(int sensor_index);


#endif /* OVERTEMP_UTILS_H */ 