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
#define DF_LORA_IMAGE_MAX_CHUNKS DF_IMAGE_MAX_CHUNKS
#define DF_LORA_IMAGE_BITMAP_BYTES DF_IMAGE_FEEDBACK_BITMAP_MAX_BYTES

typedef struct {
    uint16_t source_device_id;
    uint8_t hop_count;
    bool relayed;
    bool expect_feedback;
    uint64_t not_before_ms;
    uint32_t image_id;
    uint64_t event_id;
    uint64_t capture_epoch_ms;
    uint16_t width;
    uint16_t height;
    uint8_t quality;
    uint8_t purpose;
    size_t jpeg_size;
    uint8_t jpeg[DF_IMAGE_MAX_JPEG_BYTES];
} LoraImageJob;

typedef struct {
    bool active;
    uint16_t source_device_id;
    uint32_t image_id;
    uint32_t event_id;
    uint32_t capture_epoch_s;
    uint32_t total_size;
    uint16_t width;
    uint16_t height;
    uint16_t total_chunks;
    uint16_t received_chunks;
    uint8_t quality;
    uint8_t purpose;
    uint8_t flags;
    uint8_t hop_count;
    bool complete;
    bool file_saved;
    bool relay_queued;
    uint8_t last_done_round;
    uint8_t received_bitmap[DF_LORA_IMAGE_BITMAP_BYTES];
    uint8_t data[DF_IMAGE_MAX_JPEG_BYTES];
    uint64_t last_activity_ms;
} LoraImageRxSlot;

typedef enum {
    LORA_CONTROL_NONE = 0,
    LORA_CONTROL_IMAGE_FEEDBACK,
    LORA_CONTROL_EVENT_ACK
} LoraControlKind;

typedef struct {
    bool active;
    LoraControlKind kind;
    uint64_t due_ms;
    size_t frame_len;
    uint8_t frame[DF_WIRE_MAX_FRAME_SIZE];
    uint16_t source_device_id;
    uint32_t sequence;
    uint32_t image_id;
    uint8_t event_type;
    uint8_t round;
    uint8_t copies_total;
    uint8_t copies_sent;
    uint32_t repeat_interval_ms;
} LoraScheduledControl;

typedef struct {
    bool active;
    uint16_t source_device_id;
    uint32_t sequence;
    uint32_t event_id;
    uint8_t event_type;
    uint8_t retries;
    uint64_t next_retry_ms;
    uint64_t quiet_until_ms;
    size_t frame_len;
    uint8_t frame[DF_WIRE_MAX_FRAME_SIZE];
} LoraCriticalEventPending;

typedef enum {
    LORA_RELAY_PRIORITY_CONTROL = 0,
    LORA_RELAY_PRIORITY_EVENT = 1,
    LORA_RELAY_PRIORITY_BULK = 2
} LoraRelayPriority;

typedef struct {
    bool active;
    uint64_t frame_hash;
    uint64_t due_ms;
    uint16_t emitter_device_id;
    uint8_t packet_type;
    uint8_t priority;
    size_t frame_len;
    uint8_t frame[DF_WIRE_MAX_FRAME_SIZE];
} LoraRelayFrame;

typedef struct {
    bool valid;
    uint64_t frame_hash;
    uint64_t last_seen_ms;
    uint16_t emitter_device_id;
    uint8_t packet_type;
    size_t frame_len;
} LoraRelayCacheEntry;

typedef struct {
    char device_path[DF_LORA_DEVICE_PATH_MAX];
    int baud_rate;
    uint16_t device_id;
    bool chain_enabled;
    uint16_t chain_first_id;
    uint16_t chain_last_id;

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
    bool image_tx_in_flight;

    MonitorMessage tx_queue[DF_LORA_TX_QUEUE_CAPACITY];
    size_t tx_head;
    size_t tx_tail;
    size_t tx_count;
    uint64_t dropped_messages;

    LoraImageJob image_queue[DF_LORA_IMAGE_QUEUE_CAPACITY];
    size_t image_count;
    uint32_t next_image_id;
    uint64_t dropped_images;

    int fd;
    uint8_t rx_buffer[DF_LORA_RX_BUFFER_SIZE];
    size_t rx_size;
    LoraImageRxSlot image_rx[DF_LORA_IMAGE_RX_SLOT_COUNT];

    LoraScheduledControl control_tx[DF_LORA_RELIABILITY_CONTROL_QUEUE_CAPACITY];
    LoraCriticalEventPending critical_event_tx[DF_LORA_CRITICAL_EVENT_PENDING_CAPACITY];
    DfImageFeedbackDecoded image_feedback_rx[DF_LORA_RELIABILITY_CONTROL_QUEUE_CAPACITY];
    size_t image_feedback_rx_head;
    size_t image_feedback_rx_tail;
    size_t image_feedback_rx_count;

    LoraRelayFrame relay_queue[DF_LORA_RELAY_QUEUE_CAPACITY];
    LoraRelayCacheEntry relay_cache[DF_LORA_RELAY_CACHE_CAPACITY];

    uint64_t tx_ok;
    uint64_t tx_fail;
    uint64_t rx_ok;
    uint64_t rx_bad;
    uint64_t aux_wait_timeouts;
    uint64_t aux_busy_cycles;
    uint64_t aux_no_busy_observed;
    uint64_t tx_images_ok;
    uint64_t rx_images_ok;
    uint64_t rx_images_incomplete;
    uint64_t tx_images_confirmed;
    uint64_t tx_images_unconfirmed;
    uint64_t tx_images_gateway_sent;
    uint64_t tx_image_repair_rounds;
    uint64_t tx_image_repair_chunks;
    uint64_t image_feedback_ack_tx;
    uint64_t image_feedback_nack_tx;
    uint64_t image_feedback_rx_count_total;
    uint64_t event_ack_tx;
    uint64_t event_ack_rx;
    uint64_t critical_event_retries;
    uint64_t critical_event_unconfirmed;
    uint64_t relay_queued;
    uint64_t relay_tx_ok;
    uint64_t relay_suppressed;
    uint64_t relay_dropped;
    uint64_t relay_own_ignored;
    uint64_t chain_filtered_frames;
    uint64_t chain_images_queued;
    uint64_t chain_feedback_repeat_tx;
} LoraUartLink;

int lora_uart_init(LoraUartLink *link,
                   const char *device_path,
                   int baud_rate,
                   uint16_t device_id,
                   int gpio_m0,
                   int gpio_m1,
                   int gpio_aux,
                   unsigned heartbeat_interval_s,
                   uint16_t chain_first_id,
                   uint16_t chain_last_id);
int lora_uart_start(LoraUartLink *link);
void lora_uart_stop(LoraUartLink *link);
void lora_uart_deinit(LoraUartLink *link);

/* Non-blocking with respect to UART/LoRa I/O: copies into a mutex-protected RAM queue. */
bool lora_uart_enqueue(LoraUartLink *link, const MonitorMessage *message);

/*
 * Queue a compressed grayscale JPEG. Telemetry always has priority over image chunks.
 * Preview images are latest-wins so a 5 s deployment schedule cannot build an unbounded backlog.
 */
bool lora_uart_enqueue_image(LoraUartLink *link,
                             uint64_t event_id,
                             uint64_t capture_epoch_ms,
                             DfImagePurpose purpose,
                             uint16_t width,
                             uint16_t height,
                             uint8_t quality,
                             const uint8_t *jpeg,
                             size_t jpeg_size,
                             uint32_t *image_id_out);

/* Best-effort wait for queued telemetry/images before shutdown. */
bool lora_uart_wait_queue_empty(LoraUartLink *link, unsigned timeout_ms);

#endif
