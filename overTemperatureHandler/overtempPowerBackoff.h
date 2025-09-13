#ifndef OVERTEMP_POWER_BACKOFF_H
#define OVERTEMP_POWER_BACKOFF_H

#include "overtempInternal.h"

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

#endif /* OVERTEMP_POWER_BACKOFF_H */ 