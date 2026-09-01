#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include "monitor_queue.h"

#include <stddef.h>
#include <stdint.h>

#define DF_WIRE_MAGIC0 0x44U /* 'D' */
#define DF_WIRE_MAGIC1 0x46U /* 'F' */
#define DF_WIRE_VERSION 1U
#define DF_WIRE_MAX_PAYLOAD 128U
#define DF_WIRE_HEADER_SIZE 6U
#define DF_WIRE_CRC_SIZE 2U
#define DF_WIRE_MAX_FRAME_SIZE (DF_WIRE_HEADER_SIZE + DF_WIRE_MAX_PAYLOAD + DF_WIRE_CRC_SIZE)
#define DF_IMAGE_CHUNK_DATA_MAX 40U /* 64-byte wire frame with current IMAGE_CHUNK header/CRC */
#define DF_IMAGE_MAX_CHUNKS ((DF_IMAGE_MAX_JPEG_BYTES + DF_IMAGE_CHUNK_DATA_MAX - 1U) / DF_IMAGE_CHUNK_DATA_MAX)
#define DF_IMAGE_FEEDBACK_BITMAP_MAX_BYTES ((DF_IMAGE_MAX_CHUNKS + 7U) / 8U)

typedef enum {
    DF_WIRE_EVENT_START = 1,
    DF_WIRE_EVENT_UPDATE = 2,
    DF_WIRE_EVENT_END = 3,
    DF_WIRE_HEARTBEAT = 4,
    DF_WIRE_IMAGE_META = 0x10,
    DF_WIRE_IMAGE_CHUNK = 0x11,
    DF_WIRE_IMAGE_DONE = 0x12,
    DF_WIRE_IMAGE_FEEDBACK = 0x13,
    DF_WIRE_EVENT_ACK = 0x14
} DfWirePacketType;

typedef struct {
    DfWirePacketType type;
    uint16_t source_device_id;
    uint32_t sequence;
    uint32_t event_id;
    uint32_t start_epoch_s;
    uint32_t end_epoch_s;
    uint32_t duration_ms;
    uint32_t observed_frames;
    uint32_t active_frames;
    uint32_t recovering_frames;
    uint16_t max_gully_x10000;
    uint16_t max_moving_x10000;
    uint16_t max_blob_x10000;
    uint32_t max_blob_area;
    uint8_t max_valid_blobs;
    uint16_t max_speed_x10;
    uint16_t avg_speed_x10;
    uint8_t direction;
    uint8_t direction_consistency_255;
    uint16_t direction_samples;
    uint8_t snapshot_count;
    uint8_t end_reason;
    uint8_t flags;
    uint8_t hop_count;
} DfTelemetryDecoded;

typedef struct {
    uint16_t source_device_id;
    uint32_t sequence;
    uint32_t uptime_s;
    uint32_t tx_ok;
    uint8_t flags;
} DfHeartbeatDecoded;

typedef enum {
    DF_IMAGE_PURPOSE_EVENT = 1,
    DF_IMAGE_PURPOSE_DEPLOY_PREVIEW = 2
} DfImagePurpose;

#define DF_IMAGE_FORMAT_JPEG_GRAY 1U

typedef struct {
    uint16_t source_device_id;
    uint32_t image_id;
    uint32_t event_id;
    uint32_t capture_epoch_s;
    uint32_t total_size;
    uint16_t width;
    uint16_t height;
    uint16_t total_chunks;
    uint8_t quality;
    uint8_t purpose;
    uint8_t format;
    uint8_t flags;
    uint8_t hop_count;
} DfImageMetaDecoded;

typedef struct {
    uint16_t source_device_id;
    uint32_t image_id;
    uint32_t event_id;
    uint16_t chunk_index;
    uint16_t total_chunks;
    uint8_t data_len;
    uint8_t purpose;
    uint8_t data[DF_IMAGE_CHUNK_DATA_MAX];
} DfImageChunkDecoded;


typedef struct {
    uint16_t source_device_id;
    uint32_t image_id;
    uint32_t event_id;
    uint16_t total_chunks;
    uint8_t round;
    uint8_t purpose;
    uint8_t flags;
} DfImageDoneDecoded;

#define DF_IMAGE_FEEDBACK_FLAG_COMPLETE 0x01U

typedef struct {
    uint16_t responder_device_id;
    uint16_t source_device_id;
    uint32_t image_id;
    uint32_t event_id;
    uint16_t total_chunks;
    uint8_t round;
    uint8_t flags;
    uint8_t bitmap_len;
    uint8_t missing_bitmap[DF_IMAGE_FEEDBACK_BITMAP_MAX_BYTES];
} DfImageFeedbackDecoded;

typedef struct {
    uint16_t responder_device_id;
    uint16_t source_device_id;
    uint32_t sequence;
    uint32_t event_id;
    uint8_t event_type;
    uint8_t flags;
} DfEventAckDecoded;

/* flags */
#define DF_WIRE_FLAG_CLOCK_VALID 0x01U

/* heartbeat flags */
#define DF_HEARTBEAT_FLAG_UART_OPEN 0x01U
#define DF_HEARTBEAT_FLAG_AUX_HIGH 0x02U /* DX-LR32: AUX HIGH = radio busy */
#define DF_HEARTBEAT_FLAG_AUX_ADVISORY 0x04U /* AUX read unavailable; flow-control fallback */

const char *df_wire_packet_type_name(DfWirePacketType type);
uint16_t df_crc16_ccitt(const uint8_t *data, size_t len);

/*
 * Chain-relay helper. EVENT_* and IMAGE_META keep hop_count as the final
 * payload byte. This mutates that field and refreshes CRC without changing
 * the original source_device_id.
 */
int df_wire_set_hop_count(uint8_t *frame, size_t frame_len, uint8_t hop_count);
int df_wire_increment_hop_count(uint8_t *frame, size_t frame_len, uint8_t *new_hop_out);

/*
 * Encode EVENT_START / EVENT_UPDATE / EVENT_END into one binary UART/LoRa frame.
 * Returns frame length, or 0 when the message type is unsupported / capacity is too small.
 */
size_t df_telemetry_encode(const MonitorMessage *message,
                           uint16_t source_device_id,
                           uint8_t *out,
                           size_t out_capacity);

/* Returns 0 on success. */
int df_telemetry_decode(const uint8_t *frame,
                        size_t frame_len,
                        DfTelemetryDecoded *out);

size_t df_heartbeat_encode(uint16_t source_device_id,
                           uint32_t sequence,
                           uint32_t uptime_s,
                           uint32_t tx_ok,
                           uint8_t flags,
                           uint8_t *out,
                           size_t out_capacity);

int df_heartbeat_decode(const uint8_t *frame,
                        size_t frame_len,
                        DfHeartbeatDecoded *out);

size_t df_image_meta_encode(uint16_t source_device_id,
                            uint32_t image_id,
                            uint32_t event_id,
                            uint64_t capture_epoch_ms,
                            uint16_t width,
                            uint16_t height,
                            uint32_t total_size,
                            uint16_t total_chunks,
                            uint8_t quality,
                            DfImagePurpose purpose,
                            uint8_t *out,
                            size_t out_capacity);

int df_image_meta_decode(const uint8_t *frame,
                         size_t frame_len,
                         DfImageMetaDecoded *out);

size_t df_image_chunk_encode(uint16_t source_device_id,
                             uint32_t image_id,
                             uint32_t event_id,
                             uint16_t chunk_index,
                             uint16_t total_chunks,
                             DfImagePurpose purpose,
                             const uint8_t *data,
                             uint8_t data_len,
                             uint8_t *out,
                             size_t out_capacity);

int df_image_chunk_decode(const uint8_t *frame,
                          size_t frame_len,
                          DfImageChunkDecoded *out);


size_t df_image_done_encode(uint16_t source_device_id,
                            uint32_t image_id,
                            uint32_t event_id,
                            uint16_t total_chunks,
                            uint8_t round,
                            DfImagePurpose purpose,
                            uint8_t *out,
                            size_t out_capacity);

int df_image_done_decode(const uint8_t *frame,
                         size_t frame_len,
                         DfImageDoneDecoded *out);

size_t df_image_feedback_encode(uint16_t responder_device_id,
                                uint16_t source_device_id,
                                uint32_t image_id,
                                uint32_t event_id,
                                uint16_t total_chunks,
                                uint8_t round,
                                uint8_t flags,
                                const uint8_t *missing_bitmap,
                                uint8_t bitmap_len,
                                uint8_t *out,
                                size_t out_capacity);

int df_image_feedback_decode(const uint8_t *frame,
                             size_t frame_len,
                             DfImageFeedbackDecoded *out);

size_t df_event_ack_encode(uint16_t responder_device_id,
                           uint16_t source_device_id,
                           uint32_t sequence,
                           uint32_t event_id,
                           DfWirePacketType event_type,
                           uint8_t *out,
                           size_t out_capacity);

int df_event_ack_decode(const uint8_t *frame,
                        size_t frame_len,
                        DfEventAckDecoded *out);

/* Generic framing helpers used by the UART stream parser. */
int df_wire_peek_frame_length(const uint8_t *buffer,
                              size_t buffer_len,
                              size_t *frame_len_out);

#endif
