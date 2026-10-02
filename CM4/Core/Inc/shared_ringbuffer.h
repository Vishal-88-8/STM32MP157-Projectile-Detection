/**
 * @file shared_ringbuffer.h
 * @brief Memory map and data structure for Cortex-M4 <-> Cortex-A7 IPC
 * @note Target Address: SRAM3 (0x10040000) on STM32MP157F
 *
 * VASAVI COLLEGE OF ENGINEERING - DEPARTMENT OF ECE
 * PROJECTILE DETECTION WITH IR SENSOR USING STM32 (Nanosecond Framework)
 */

#ifndef SHARED_RINGBUFFER_H
#define SHARED_RINGBUFFER_H

#include <stdint.h>

#define SHARED_SRAM_BASE_ADDR   0x10040000UL
#define SHARED_SRAM_SIZE        0x00010000UL  /* 64 KB */
#define RINGBUFFER_CAPACITY     64
#define RINGBUFFER_MAGIC        0x53544D33UL  /* "STM3" */

/* Hardware Semaphore Channel ID for ringbuffer arbitration */
#define HSEM_CHANNEL_RINGBUF    0

/* Event classification codes */
typedef enum {
    EVENT_TYPE_TEST        = 0,  /* Internal software/timer simulated trigger */
    EVENT_TYPE_IR_TRIGGER  = 1,  /* Physical projectile crossing IR optical screen */
    EVENT_TYPE_HOLD_IN     = 2,  /* Upstream station Hold-In gating edge */
    EVENT_TYPE_PPS_TICK    = 3   /* PPS reference capture event */
} event_type_t;

/* Status and error flags */
#define STATUS_FLAG_VALID        (1U << 0)
#define STATUS_FLAG_OVERFLOW     (1U << 1)
#define STATUS_FLAG_TIMER_WRAP   (1U << 2)

/**
 * @brief Single timestamp capture record (32 bytes, cache-line aligned)
 */
typedef struct {
    uint32_t trigger_id;      /**< Monotonic trigger/shot counter */
    uint32_t event_type;      /**< event_type_t enum */
    uint32_t t_event_ticks;   /**< M4 32-bit TIM2 capture for the IR event */
    uint32_t t_k_ticks;       /**< M4 32-bit TIM2 capture for the k-th PPS edge */
    uint32_t t_k1_ticks;      /**< M4 32-bit TIM2 capture for the (k+1)-th PPS edge */
    uint32_t pps_sequence;    /**< Monotonic PPS second epoch counter */
    uint32_t flags;           /**< Status/overflow flags */
    uint32_t reserved;        /**< Aligns struct to 32 bytes */
} __attribute__((packed, aligned(4))) timestamp_record_t;

/**
 * @brief Circular Ring Buffer layout in Shared SRAM
 */
typedef struct {
    volatile uint32_t magic;                              /**< Integrity magic: RINGBUFFER_MAGIC */
    volatile uint32_t write_idx;                          /**< Written by Cortex-M4 */
    volatile uint32_t read_idx;                           /**< Read/Updated by Cortex-A7 */
    volatile uint32_t overflow_count;                     /**< Total buffer overrun counter */
    timestamp_record_t records[RINGBUFFER_CAPACITY];      /**< Array of capture records */
} __attribute__((packed, aligned(8))) shared_ringbuffer_t;

#endif /* SHARED_RINGBUFFER_H */
