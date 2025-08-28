#ifndef RTC_CMD_H
#define RTC_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize RTC CLI commands
 *
 * @note This registers RTC-related CLI commands, including enable_irq and disable_irq
 */
void rtcCmdInit(void);

/**
 * @brief RTC command handler
 *
 * @param argc Argument count
 * @param argv Argument vector
 *
 * @note Handles RTC-related CLI options: enable_irq and disable_irq
 */
void rtcCmd(int argc, char *argv[]);

#ifdef __cplusplus
}
#endif

#endif /* RTC_CMD_H */ 