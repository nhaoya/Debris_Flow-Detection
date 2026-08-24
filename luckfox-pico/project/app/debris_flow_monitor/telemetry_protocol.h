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

typedef enum {
    DF_WIRE_EVENT_START = 1,
    DF_WIRE_EVENT_UPDATE = 2,
    DF_WIRE_EVENT_END = 3,
    DF_WIRE_HEARTBEAT = 4,
    DF_WIRE_IMAGE_META = 0x10,
    DF_WIRE_IMAGE_CHUNK = 0x11
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

/* flags */
#define DF_WIRE_FLAG_CLOCK_VALID 0x01U

/* heartbeat flags */
#define DF_HEARTBEAT_FLAG_UART_OPEN 0x01U
#define DF_HEARTBEAT_FLAG_AUX_HIGH 0x02U
#define DF_HEARTBEAT_FLAG_AUX_ADVISORY 0x04U

const char *df_wire_packet_type_name(DfWirePacketType type);
uint16_t df_crc16_ccitt(const uint8_t *data, size_t len);

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

/* Generic framing helpers used by the UART stream parser. */
int df_wire_peek_frame_length(const uint8_t *buffer,
                              size_t buffer_len,
                              size_t *frame_len_out);

#endif
