#ifndef UART2_LINK_H
#define UART2_LINK_H

#include "monitor_queue.h"
#include "telemetry_protocol.h"

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DF_UART2_TX_QUEUE_CAPACITY 64U
#define DF_UART2_DEVICE_PATH_MAX 96U
#define DF_UART2_RX_BUFFER_SIZE 512U

typedef struct {
    char device_path[DF_UART2_DEVICE_PATH_MAX];
    int baud_rate;
    uint16_t device_id;

    pthread_t worker_thread;
    pthread_mutex_t queue_mutex;
    bool mutex_ready;
    bool worker_started;
    bool stop_requested;
    bool tx_in_flight;

    MonitorMessage tx_queue[DF_UART2_TX_QUEUE_CAPACITY];
    size_t tx_head;
    size_t tx_tail;
    size_t tx_count;
    uint64_t dropped_messages;

    int fd;
    uint8_t rx_buffer[DF_UART2_RX_BUFFER_SIZE];
    size_t rx_size;

    uint64_t tx_ok;
    uint64_t tx_fail;
    uint64_t rx_ok;
    uint64_t rx_bad;
} Uart2Link;

int uart2_link_init(Uart2Link *link,
                    const char *device_path,
                    int baud_rate,
                    uint16_t device_id);
int uart2_link_start(Uart2Link *link);
void uart2_link_stop(Uart2Link *link);
void uart2_link_deinit(Uart2Link *link);

/* Non-blocking with respect to UART I/O. It only copies into a mutex-protected RAM queue. */
bool uart2_link_enqueue(Uart2Link *link, const MonitorMessage *message);

/* Best-effort wait for queued telemetry before shutdown. Returns true if queue became empty. */
bool uart2_link_wait_queue_empty(Uart2Link *link, unsigned timeout_ms);

#endif
