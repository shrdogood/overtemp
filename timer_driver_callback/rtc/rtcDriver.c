#include "rtcDriver.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <time.h>

/* ========================= Global Variables ========================= */

/* RTC device management */
static rtc_device_t rtc_devices[2] = {
    {.fd = -1, .device_path = NUCLEI_RTC_CHR_DEV0, .is_initialized = 0},
    {.fd = -1, .device_path = NUCLEI_RTC_CHR_DEV1, .is_initialized = 0}
};

/* Interrupt monitoring thread related */
static pthread_t monitor_thread;
static int monitor_running = 0;

/* Mutexes */
static pthread_mutex_t rtc_mutex = PTHREAD_MUTEX_INITIALIZER;  /* Protects service list operations */

/* Timer service arrays */
static timer_service_t services_timer0[MAX_SERVICES];
static timer_service_t services_timer1[MAX_SERVICES];

/* ========================= Private Function Declarations ========================= */
static void* rtc_irq_monitor_thread(void *arg);
static void* task_thread_func(void* arg);
static void rtc_timer_tick_handler(void);
static void rtc_timer_tick_handler2(void);
static int init_timer_services(void);
static void cleanup_timer_services(void);
static timer_service_t* get_timer_services(int timer_id);

/* ========================= Thread Related Functions ========================= */

/**
 * @brief Task thread function - executes callback and then exits
 */
static void* task_thread_func(void* arg) {
    timer_service_t* service = (timer_service_t*)arg;
    
    if (service && service->callback_func) {
        service->callback_func();
    }

    /* Clear running flag */
    if (service) {
        pthread_mutex_lock(&service->service_mutex);
        service->is_running = 0;
        pthread_mutex_unlock(&service->service_mutex);
    }
    
    /* Detach thread to let system reclaim resources automatically */
    pthread_detach(pthread_self());
    return NULL;
}

/**
 * @brief Timer0 tick handler function
 */
static void rtc_timer_tick_handler(void) {
    for (int i = 0; i < MAX_SERVICES; i++) {
        timer_service_t* service = &services_timer0[i];
        
        if (!service->callback_func) {
            continue;
        }
        
        pthread_mutex_lock(&service->service_mutex);
        service->count++;
        
        if (service->count >= service->threshold) {
            if (!service->is_running && service->callback_func) {
                service->is_running = 1;  /* Mark as running to prevent re-entrancy */
                service->count = 0;       /* Reset counter */
                pthread_mutex_unlock(&service->service_mutex);

                /* Create thread to execute callback */
                pthread_t task_thread;
                int ret = pthread_create(&task_thread, NULL, task_thread_func, service);
                if (ret != 0) {
                    printf("Failed to create task thread for service %s: %s\n", 
                           service->service_name, strerror(ret));
                    /* Restore running flag if thread creation fails */
                    pthread_mutex_lock(&service->service_mutex);
                    service->is_running = 0;
                    pthread_mutex_unlock(&service->service_mutex);
                }
            } else {
                /* Already running or callback unregistered */
                pthread_mutex_unlock(&service->service_mutex);
            }
        } else {
            pthread_mutex_unlock(&service->service_mutex);
        }
    }
}

/**
 * @brief Timer1 tick handler function
 */
static void rtc_timer_tick_handler2(void) {
    for (int i = 0; i < MAX_SERVICES; i++) {
        timer_service_t* service = &services_timer1[i];
        
        if (!service->callback_func) {
            continue;
        }
        
        pthread_mutex_lock(&service->service_mutex);
        service->count++;
        
        if (service->count >= service->threshold) {
            if (!service->is_running && service->callback_func) {
                service->is_running = 1;  /* Mark as running to prevent re-entrancy */
                service->count = 0;       /* Reset counter */
                pthread_mutex_unlock(&service->service_mutex);

                /* Create thread to execute callback */
                pthread_t task_thread;
                int ret = pthread_create(&task_thread, NULL, task_thread_func, service);
                if (ret != 0) {
                    printf("Failed to create task thread for service %s: %s\n", 
                           service->service_name, strerror(ret));
                    /* Restore running flag if thread creation fails */
                    pthread_mutex_lock(&service->service_mutex);
                    service->is_running = 0;
                    pthread_mutex_unlock(&service->service_mutex);
                }
            } else {
                /* Already running or callback unregistered */
                pthread_mutex_unlock(&service->service_mutex);
            }
        } else {
            pthread_mutex_unlock(&service->service_mutex);
        }
    }
}

/**
 * @brief IRQ monitoring thread
 */
static void* rtc_irq_monitor_thread(void *arg) {
    struct pollfd pfd[2];
    int ret;
    unsigned long irq_count;

    printf("RTC interrupt monitoring thread started\n");

    /* Setup pollfd array */
    pfd[0].fd = rtc_devices[0].fd;
    pfd[0].events = POLLIN;
    
    pfd[1].fd = rtc_devices[1].fd;
    pfd[1].events = POLLIN;
    
    while (monitor_running) {
        /* Wait for interrupt events */
        ret = poll(pfd, 2, -1);   /* Monitor 2 fds, -1 means wait forever */
        
        if (ret < 0) {
            if (errno == EINTR) {
                continue;  /* Interrupted by signal, continue waiting */
            }
            perror("poll failed");
            break;
        }

        if (!monitor_running) {
            break;  /* Check exit flag */
        }
        
        if (ret > 0) {
            /* Check which device has events */
            for (int i = 0; i < 2; i++) {
                if (pfd[i].revents & POLLIN) {
                    /* Interrupt occurred, read interrupt count */
                    if (read(pfd[i].fd, &irq_count, sizeof(irq_count)) == sizeof(irq_count)) {
                        /* Dispatch to corresponding timer handler by device index */
                        if (i == 0) {
                            rtc_timer_tick_handler();   /* Timer0 handler */
                        } else {
                            rtc_timer_tick_handler2();  /* Timer1 handler */
                        }
                    } 
                }
            }
        } 
    }

    printf("RTC interrupt monitoring thread exited\n");
    return NULL;
}

/* ========================= Private Helper Functions ========================= */

/**
 * @brief Initialize timer service arrays
 */
static int init_timer_services(void) {
    /* Initialize mutexes for all services */
    for (int i = 0; i < MAX_SERVICES; i++) {
        if (pthread_mutex_init(&services_timer0[i].service_mutex, NULL) != 0) {
            /* Cleanup already initialized mutexes */
            for (int j = 0; j < i; j++) {
                pthread_mutex_destroy(&services_timer0[j].service_mutex);
            }
            return -1;
        }
        
        if (pthread_mutex_init(&services_timer1[i].service_mutex, NULL) != 0) {
            /* Cleanup already initialized mutexes */
            for (int j = 0; j <= i; j++) {
                pthread_mutex_destroy(&services_timer0[j].service_mutex);
            }
            for (int j = 0; j < i; j++) {
                pthread_mutex_destroy(&services_timer1[j].service_mutex);
            }
            return -1;
        }
        
        /* Initialize service structures */
        memset(&services_timer0[i], 0, sizeof(timer_service_t));
        memset(&services_timer1[i], 0, sizeof(timer_service_t));
    }
    
    return 0;
}

/**
 * @brief Cleanup timer services
 */
static void cleanup_timer_services(void) {
    /* Clear all services but don't destroy mutexes to avoid racing with exiting task threads */
    pthread_mutex_lock(&rtc_mutex);
    
    for (int i = 0; i < MAX_SERVICES; i++) {
        /* Clear Timer0 services */
        pthread_mutex_lock(&services_timer0[i].service_mutex);
        services_timer0[i].callback_func = NULL;
        services_timer0[i].threshold = 0;
        services_timer0[i].count = 0;
        services_timer0[i].is_running = 0;
        services_timer0[i].service_name[0] = '\0';
        pthread_mutex_unlock(&services_timer0[i].service_mutex);
        
        /* Clear Timer1 services */
        pthread_mutex_lock(&services_timer1[i].service_mutex);
        services_timer1[i].callback_func = NULL;
        services_timer1[i].threshold = 0;
        services_timer1[i].count = 0;
        services_timer1[i].is_running = 0;
        services_timer1[i].service_name[0] = '\0';
        pthread_mutex_unlock(&services_timer1[i].service_mutex);
    }
    
    pthread_mutex_unlock(&rtc_mutex);
}

/**
 * @brief Get timer service array by timer ID
 */
static timer_service_t* get_timer_services(int timer_id) {
    return (timer_id == 0) ? services_timer0 : services_timer1;
}

/* ========================= Public API Functions ========================= */

/**
 * @brief Initialize RTC devices and monitoring thread
 */
int rtc_init(void) {
    pthread_mutex_lock(&rtc_mutex);
    
    /* Check if already initialized */
    if (rtc_devices[0].is_initialized && rtc_devices[1].is_initialized) {
        pthread_mutex_unlock(&rtc_mutex);
        printf("RTC devices already initialized\n");
        return 0;
    }
    
    /* Open RTC devices */
    for (int i = 0; i < 2; i++) {
        rtc_devices[i].fd = open(rtc_devices[i].device_path, O_RDWR);
        if (rtc_devices[i].fd < 0) {
            perror("Failed to open RTC device");
            /* Close already opened devices */
            for (int j = 0; j < i; j++) {
                if (rtc_devices[j].fd >= 0) {
                    close(rtc_devices[j].fd);
                    rtc_devices[j].fd = -1;
                    rtc_devices[j].is_initialized = 0;
                }
            }
            pthread_mutex_unlock(&rtc_mutex);
            return -1;
        }
        rtc_devices[i].is_initialized = 1;
        printf("RTC device %s opened successfully (fd=%d)\n", 
               rtc_devices[i].device_path, rtc_devices[i].fd);
    }

    /* Initialize timer services */
    if (init_timer_services() != 0) {
        perror("Failed to initialize timer services");
        /* Close devices */
        for (int i = 0; i < 2; i++) {
            if (rtc_devices[i].fd >= 0) {
                close(rtc_devices[i].fd);
                rtc_devices[i].fd = -1;
                rtc_devices[i].is_initialized = 0;
            }
        }
        pthread_mutex_unlock(&rtc_mutex);
        return -1;
    }
    
    /* Create interrupt monitoring thread */
    monitor_running = 1;
    if (pthread_create(&monitor_thread, NULL, rtc_irq_monitor_thread, NULL) != 0) {
        perror("Failed to create RTC interrupt monitor thread");
        /* Cleanup resources */
        for (int i = 0; i < 2; i++) {
            close(rtc_devices[i].fd);
            rtc_devices[i].fd = -1;
            rtc_devices[i].is_initialized = 0;
        }
        monitor_running = 0;
        pthread_mutex_unlock(&rtc_mutex);
        return -1;
    }
    
    pthread_mutex_unlock(&rtc_mutex);
    printf("RTC initialization completed successfully\n");
    return 0;
}

/**
 * @brief Enable RTC interrupt via standard RTC ioctl
 */
int rtc_enable_irq(int rtc_num) {
    if (rtc_num < 0 || rtc_num > 1) {
        printf("Invalid RTC number: %d\n", rtc_num);
        return -1;
    }
    
    const char* device_path = (rtc_num == 0) ? RTC_0 : RTC_1;
    int fd = open(device_path, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open RTC device");
        return -1;
    }
    
    /* Enable interrupt using standard RTC interface */
    int ret = ioctl(fd, RTC_VL_READ, 0);
    close(fd);
    
    if (ret < 0) {
        perror("Failed to enable RTC interrupt");
        return -1;
    }
    
    printf("RTC%d interrupt enabled successfully\n", rtc_num);
    return 0;
}

/**
 * @brief Disable RTC interrupt via standard RTC ioctl
 */
int rtc_disable_irq(int rtc_num) {
    if (rtc_num < 0 || rtc_num > 1) {
        printf("Invalid RTC number: %d\n", rtc_num);
        return -1;
    }
    
    const char* device_path = (rtc_num == 0) ? RTC_0 : RTC_1;
    int fd = open(device_path, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open RTC device");
        return -1;
    }
    
    /* Disable interrupt using standard RTC interface */
    int ret = ioctl(fd, RTC_VL_CLR, 0);
    close(fd);
    
    if (ret < 0) {
        perror("Failed to disable RTC interrupt");
        return -1;
    }
    
    printf("RTC%d interrupt disabled successfully\n", rtc_num);
    return 0;
}

/**
 * @brief Register a timer service
 */
int rtc_register_service(int timer_id, const char *name, int interval, void (*callback_func)(void)) {
    /* Parameter validation */
    if (!name || !callback_func || interval <= 0 || timer_id < 0 || timer_id > 1) {
        printf("Invalid parameters for service registration\n");
        return -1;
    }
    
    if (strlen(name) >= MAX_SERVICE_NAME_LEN) {
        printf("Service name too long (max %d chars)\n", MAX_SERVICE_NAME_LEN - 1);
        return -1;
    }
    
    pthread_mutex_lock(&rtc_mutex);
    
    timer_service_t *services = get_timer_services(timer_id);
    
    /* Find free slot to register service */
    for (int i = 0; i < MAX_SERVICES; i++) {
        if (!services[i].callback_func) {
            /* Initialize service state */
            pthread_mutex_lock(&services[i].service_mutex);
            services[i].threshold = interval;
            services[i].callback_func = callback_func;
            services[i].count = 0;
            services[i].is_running = 0;
            strncpy(services[i].service_name, name, MAX_SERVICE_NAME_LEN - 1);
            services[i].service_name[MAX_SERVICE_NAME_LEN - 1] = '\0';
            pthread_mutex_unlock(&services[i].service_mutex);
            
            pthread_mutex_unlock(&rtc_mutex);
            printf("Service '%s' registered successfully on timer%d (slot %d, interval=%d)\n", 
                   name, timer_id, i, interval);
            return 0;
        }
    }
    
    pthread_mutex_unlock(&rtc_mutex);
    printf("Failed to register service '%s' on timer%d: no available slots\n", name, timer_id);
    return -1; 
}

/**
 * @brief Unregister a timer service
 */
int rtc_unregister_service(int timer_id, const char *name) {
    /* Parameter validation */
    if (!name || timer_id < 0 || timer_id > 1) {
        printf("Invalid service name or timer_id\n");
        return -1;
    }
    
    pthread_mutex_lock(&rtc_mutex);
    
    timer_service_t *services = get_timer_services(timer_id);
    
    for (int i = 0; i < MAX_SERVICES; i++) {
        if (services[i].callback_func && strcmp(services[i].service_name, name) == 0) {
            /* Clear service slot */
            pthread_mutex_lock(&services[i].service_mutex);
            services[i].callback_func = NULL;
            services[i].threshold = 0;
            services[i].count = 0;
            services[i].is_running = 0;
            services[i].service_name[0] = '\0';
            pthread_mutex_unlock(&services[i].service_mutex);
            
            pthread_mutex_unlock(&rtc_mutex);
            printf("Service '%s' unregistered successfully from timer%d (slot %d)\n", 
                   name, timer_id, i);
            return 0;
        }
    }
    
    pthread_mutex_unlock(&rtc_mutex);
    printf("Service '%s' not found on timer%d\n", name, timer_id);
    return -1;
}

/**
 * @brief Get the number of registered services on the specified timer
 */
int rtc_get_service_count(int timer_id) {
    if (timer_id < 0 || timer_id > 1) {
        return -1;
    }
    
    pthread_mutex_lock(&rtc_mutex);
    timer_service_t *services = get_timer_services(timer_id);
    
    int count = 0;
    for (int i = 0; i < MAX_SERVICES; i++) {
        if (services[i].callback_func) {
            count++;
        }
    }
    
    pthread_mutex_unlock(&rtc_mutex);
    return count;
}

/**
 * @brief Check if RTC devices are initialized
 */
int rtc_is_initialized(void) {
    return (rtc_devices[0].is_initialized && rtc_devices[1].is_initialized) ? 1 : 0;
}

/**
 * @brief Cleanup RTC devices and services
 */
void rtc_cleanup(void) {
    /* Stop monitoring thread */
    if (monitor_running) {
        monitor_running = 0;
        pthread_cancel(monitor_thread);
        pthread_join(monitor_thread, NULL);
        printf("RTC monitor thread stopped\n");
    }
    
    /* Close devices */
    for (int i = 0; i < 2; i++) {
        if (rtc_devices[i].fd >= 0) {
            close(rtc_devices[i].fd);
            rtc_devices[i].fd = -1;
            rtc_devices[i].is_initialized = 0;
            printf("RTC device %s closed\n", rtc_devices[i].device_path);
        }
    }
    
    /* Cleanup services */
    cleanup_timer_services();
    
    printf("RTC cleanup completed\n");
}