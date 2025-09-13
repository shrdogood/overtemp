#ifndef OVERTEMP_STATE_HANDLER_H
#define OVERTEMP_STATE_HANDLER_H

#include "overtempInternal.h"

// 关机处理回调函数指针类型
typedef void (*request_shutdown_callback_t)(void);

/*==============================================================================
 * 状态转换处理函数
 *============================================================================*/

/**
 * @brief 处理从正常运行态到状态保持态的转换
 * @param channel 通道指针
 */
void handle_normal_to_holdoff_transition(channel_t* channel);

/**
 * @brief 处理从状态保持态到正常运行态的转换
 * @param channel 通道指针
 */
void handle_holdoff_to_normal_transition(channel_t* channel);

/**
 * @brief 处理从状态保持态到功率回退态的转换
 * @param channel 通道指针
 */
void handle_holdoff_to_backoff_transition(channel_t* channel);

/**
 * @brief 处理从功率回退态到状态保持态的转换
 * @param channel 通道指针
 */
void handle_backoff_to_holdoff_transition(channel_t* channel);

/**
 * @brief 处理从功率回退态到正常运行态的转换
 * @param channel 通道指针
 */
void handle_backoff_to_normal_transition(channel_t* channel);

/**
 * @brief 处理从功率回退态到功率回退保持态的转换
 * @param channel 通道指针
 */
void handle_backoff_to_extended_backoff_transition(channel_t* channel);

/**
 * @brief 处理从功率回退保持态到功率回退态的转换
 * @param channel 通道指针
 */
void handle_extended_backoff_to_backoff_transition(channel_t* channel);

/**
 * @brief 处理从功率回退保持态到请求关PA的转换
 * @param channel 通道指针
 */
void handle_extended_backoff_to_request_paoff_transition(channel_t* channel);

/**
 * @brief 处理从请求关PA到功率回退保持态的转换
 * @param channel 通道指针
 */
void handle_request_paoff_to_extended_backoff_transition(channel_t* channel);

/**
 * @brief 处理从请求关PA到请求关机的转换
 * @param channel 通道指针
 */
void handle_request_paoff_to_request_shutdown_transition(channel_t* channel);

/**
 * @brief 注册请求关机回调函数
 * @param cb 关机回调函数指针
 */
void register_request_shutdown_callback(request_shutdown_callback_t cb);

/**
 * @brief 设置通道温度处理状态
 * @param channel 通道指针
 * @param state 目标状态
 */
void channel_set_temp_state(channel_t* channel, temp_handling_state_t state);


#endif /* OVERTEMP_STATE_HANDLER_H */ 