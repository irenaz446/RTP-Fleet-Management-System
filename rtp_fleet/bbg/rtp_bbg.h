/**
 * @file rtp_bbg.h
 * @brief Shared types for the BBG RTP gateway.
 */

#ifndef RTP_BBG_H
#define RTP_BBG_H

#include <pthread.h>
#include "../common/common.h"

#define I2C_DEV          "/dev/i2c-2"
#define STM32_SLAVE_ADDR 0x08

/**
 * @brief Simple single-slot queue between I2C reader and RTP sender.
 *        Protected by mutex + condvar.
 */
typedef struct {
    telemetry_frame_t frame;
    int               has_data;
    pthread_mutex_t   mutex;
    pthread_cond_t    cond;
} frame_slot_t;

/**
 * @brief Arguments for the RTP sender thread.
 */
typedef struct {
    const char  *server_ip;
    int          server_port;
    frame_slot_t *slot;
} rtp_thread_arg_t;

#endif /* RTP_BBG_H */
