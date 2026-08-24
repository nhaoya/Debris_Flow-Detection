#ifndef LORA_UART_H
#define LORA_UART_H

#include "monitor_queue.h"
#include "telemetry_protocol.h"

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DF_LORA_TX_QUEUE_CAPACITY 64U
#define DF_LORA_DEVICE_PATH_MAX 96U
#define DF_LORA_RX_BUFFER_SIZE 512U

typedef struct {
    char device_path[DF_LORA_DEVICE_PATH_MAX];
    int baud_rate;
    uint16_t device_id;

    int gpio_m0;
    int gpio_m1;
    int gpio_aux;
    bool gpio_ready;
    bool aux_ready;
    bool aux_advisory_only;
    unsigned heartbeat_interval_s;
    uint32_t heartbeat_sequence;

    pthread_t worker_thread;
    pthread_mutex_t queue_mutex;
    bool mutex_ready;
    bool worker_started;
    bool stop_requested;
    bool tx_in_flight;

    MonitorMessage tx_queue[DF_LORA_TX_QUEUE_CAPACITY];
    size_t tx_head;
    size_t tx_tail;
    size_t tx_count;
    uint64_t dropped_messages;

    int fd;
    uint8_t rx_buffer[DF_LORA_RX_BUFFER_SIZE];
    size_t rx_size;

    uint64_t tx_ok;
    uint64_t tx_fail;
    uint64_t rx_ok;
    uint64_t rx_bad;
    uint64_t aux_wait_timeouts;
} LoraUartLink;

int lora_uart_init(LoraUartLink *link,
                   const char *device_path,
                   int baud_rate,
                   uint16_t device_id,
                   int gpio_m0,
                   int gpio_m1,
                   int gpio_aux,
                   unsigned heartbeat_interval_s);
int lora_uart_start(LoraUartLink *link);
void lora_uart_stop(LoraUartLink *link);
void lora_uart_deinit(LoraUartLink *link);

/* Non-blocking with respect to UART/LoRa I/O: copies into a mutex-protected RAM queue. */
bool lora_uart_enqueue(LoraUartLink *link, const MonitorMessage *message);

/* Best-effort wait for queued telemetry before shutdown. */
bool lora_uart_wait_queue_empty(LoraUartLink *link, unsigned timeout_ms);

#endif
