#ifndef __RTC_DRIVER_H__
#define __RTC_DRIVER_H__

#include <linux/rtc.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <poll.h>
#include <signal.h>
#include <pthread.h>

/* ========================= Macro Definitions ========================= */
#define RTC_0 "/dev/rtc0"
#define RTC_1 "/dev/rtc1"

/* Nuclei RTC character device paths */
#define NUCLEI_RTC_CHR_DEV0 "/dev/nuclei_rtc0"
#define NUCLEI_RTC_CHR_DEV1 "/dev/nuclei_rtc1"

/* Configuration parameters */
#define MAX_SERVICES 10             /* Maximum number of supported services */
#define MAX_SERVICE_NAME_LEN 32     /* Maximum service name length */

/* ========================= Data Structures ========================= */

/**
 * @brief Timer service structure
 * 
 * This structure manages each registered timer service, including service
 * state information, callback function pointers, and mutexes for thread safety.
 */
typedef struct {
    int count;                                  /* Current count value */
    int threshold;                              /* Trigger threshold */
    void (*callback_func)(void);                /* Callback function pointer */
    char service_name[MAX_SERVICE_NAME_LEN];    /* Service name */
    pthread_mutex_t service_mutex;              /* Mutex to protect individual service state */
    int is_running;                             /* Running state flag */
} timer_service_t;

/**
 * @brief RTC device management structure
 * 
 * Used to manage RTC device file descriptors and related state information
 */
typedef struct {
    int fd;                         /* Device file descriptor */
    char device_path[64];           /* Device path */
    int is_initialized;             /* Initialization state */
} rtc_device_t;

/* ========================= Function Declarations ========================= */

/* ------------- Initialization and Cleanup Functions ------------- */

/**
 * @brief Initialize RTC devices
 *
 * @return int Result code
 *         - 0: Initialization successful
 *         - negative: Initialization failed
 *
 * @note This function opens RTC character devices and starts interrupt monitoring thread
 */
int rtc_init(void);

/**
 * @brief Cleanup RTC devices
 *
 * @note This function closes RTC character devices and releases resources.
 *       Should be called before program exit.
 */
void rtc_cleanup(void);

/* ------------- Interrupt Control Functions ------------- */

/**
 * @brief Enable interrupts for the specified RTC device
 *
 * @param rtc_num RTC device index to operate on (0 or 1)
 * @return int Result code
 *         - 0: Interrupt enabled successfully
 *         - negative: Operation failed
 */
int rtc_enable_irq(int rtc_num);

/**
 * @brief Disable interrupts for the specified RTC device
 *
 * @param rtc_num RTC device index to operate on (0 or 1)
 * @return int Result code
 *         - 0: Interrupt disabled successfully
 *         - negative: Operation failed
 */
int rtc_disable_irq(int rtc_num);

/* ------------- Service Management Functions ------------- */

/**
 * @brief Register a timer service
 *
 * @param timer_id Timer index (0 or 1)
 * @param name Service name
 * @param interval Trigger interval
 * @param callback_func Callback function invoked in a detached thread when triggered
 * @return int Result code
 *         - 0: Registration successful
 *         - -1: Registration failed (no available slot or invalid parameters)
 *
 * @note Each trigger spawns a temporary thread to execute the callback. The
 *       thread exits automatically after completion; no lifecycle management needed.
 */
int rtc_register_service(int timer_id, const char *name, int interval, void (*callback_func)(void));

/**
 * @brief Unregister a timer service
 *
 * @param timer_id Timer index (0 or 1)
 * @param name Service name to unregister
 * @return int Result code
 *         - 0: Unregistration successful
 *         - -1: Unregistration failed (not found or invalid parameters)
 */
int rtc_unregister_service(int timer_id, const char *name);

/* ------------- Query and Status Functions ------------- */

/**
 * @brief Get the number of registered services on the specified timer
 *
 * @param timer_id Timer index (0 or 1)
 * @return int Number of registered services, -1 indicates invalid parameters
 */
int rtc_get_service_count(int timer_id);

/**
 * @brief Check if RTC devices are initialized
 *
 * @return int Initialization status
 *         - 1: Initialized
 *         - 0: Not initialized
 */
int rtc_is_initialized(void);

#endif /* __RTC_DRIVER_H__ */