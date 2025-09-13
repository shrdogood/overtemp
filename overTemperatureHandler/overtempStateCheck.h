#ifndef OVERTEMP_STATE_CHECK_H
#define OVERTEMP_STATE_CHECK_H

#include "overtempInternal.h"
#include <stdbool.h>

/**
 * @brief 检查从正常运行态到状态保持态的转换条件
 * @param channel 通道指针
 * @return true-满足转换条件，false-不满足
 */
bool check_normal_to_holdoff_transition(channel_t* channel);

/**
 * @brief 检查从状态保持态到正常运行态的转换条件
 * @param channel 通道指针
 * @return true-满足转换条件，false-不满足
 */
bool check_holdoff_to_normal_transition(channel_t* channel);

/**
 * @brief 检查从状态保持态到功率回退态的转换条件
 * @param channel 通道指针
 * @return true-满足转换条件，false-不满足
 */
bool check_holdoff_to_backoff_transition(channel_t* channel);

/**
 * @brief 检查从功率回退态到状态保持态的转换条件
 * @param channel 通道指针
 * @return true-满足转换条件，false-不满足
 */
bool check_backoff_to_holdoff_transition(channel_t* channel);

/**
 * @brief 检查从功率回退态到正常运行态的转换条件
 * @param channel 通道指针
 * @return true-满足转换条件，false-不满足
 */
bool check_backoff_to_normal_transition(channel_t* channel);

/**
 * @brief 检查从功率回退态到功率回退保持态的转换条件
 * @param channel 通道指针
 * @return true-满足转换条件，false-不满足
 */
bool check_backoff_to_extended_backoff_transition(channel_t* channel);

/**
 * @brief 检查从功率回退保持态到功率回退态的转换条件
 * @param channel 通道指针
 * @return true-满足转换条件，false-不满足
 */
bool check_extended_backoff_to_backoff_transition(channel_t* channel);

/**
 * @brief 检查从功率回退保持态到请求关PA的转换条件
 * @param channel 通道指针
 * @return true-满足转换条件，false-不满足
 */
bool check_extended_backoff_to_request_paoff_transition(channel_t* channel);

/**
 * @brief 检查从请求关PA到请求关机的转换条件
 * @param channel 通道指针
 * @return true-满足转换条件，false-不满足
 */
bool check_request_paoff_to_request_shutdown_transition(channel_t* channel);

/**
 * @brief 检查从请求关PA到功率回退保持态的转换条件
 * @param channel 通道指针
 * @return true-满足转换条件，false-不满足
 */
bool check_request_paoff_to_extended_backoff_transition(channel_t* channel);


#endif /* OVERTEMP_STATE_CHECK_H */ 