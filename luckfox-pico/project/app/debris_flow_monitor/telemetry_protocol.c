#include "telemetry_protocol.h"

#include <limits.h>
#include <stdbool.h>
#include <string.h>

#define DF_TELEMETRY_PAYLOAD_SIZE 57U
#define DF_CLOCK_VALID_MIN_EPOCH_MS 1704067200000ULL /* 2024-01-01 UTC */

static void put_u16_le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xffU);
    p[1] = (uint8_t)((v >> 8) & 0xffU);
}

static void put_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xffU);
    p[1] = (uint8_t)((v >> 8) & 0xffU);
    p[2] = (uint8_t)((v >> 16) & 0xffU);
    p[3] = (uint8_t)((v >> 24) & 0xffU);
}

static uint16_t get_u16_le(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint32_t sat_u32_u64(uint64_t v) {
    return v > UINT32_MAX ? UINT32_MAX : (uint32_t)v;
}

static uint16_t sat_u16_u64(uint64_t v) {
    return v > UINT16_MAX ? UINT16_MAX : (uint16_t)v;
}

static uint8_t sat_u8_u64(uint64_t v) {
    return v > UINT8_MAX ? UINT8_MAX : (uint8_t)v;
}

static uint16_t ratio_x10000(double ratio) {
    double scaled;
    if (ratio <= 0.0) return 0;
    scaled = ratio * 10000.0 + 0.5;
    if (scaled >= 65535.0) return 65535U;
    return (uint16_t)scaled;
}

static uint16_t speed_x10(double speed) {
    double scaled;
    if (speed <= 0.0) return 0;
    scaled = speed * 10.0 + 0.5;
    if (scaled >= 65535.0) return 65535U;
    return (uint16_t)scaled;
}

static uint8_t consistency_255(double value) {
    double scaled;
    if (value <= 0.0) return 0;
    if (value >= 1.0) return 255U;
    scaled = value * 255.0 + 0.5;
    return (uint8_t)scaled;
}

static DfWirePacketType wire_type_from_monitor(MonitorMessageType type) {
    switch (type) {
    case MONITOR_EVENT_START: return DF_WIRE_EVENT_START;
    case MONITOR_EVENT_UPDATE: return DF_WIRE_EVENT_UPDATE;
    case MONITOR_EVENT_END: return DF_WIRE_EVENT_END;
    default: return (DfWirePacketType)0;
    }
}

const char *df_wire_packet_type_name(DfWirePacketType type) {
    switch (type) {
    case DF_WIRE_EVENT_START: return "EVENT_START";
    case DF_WIRE_EVENT_UPDATE: return "EVENT_UPDATE";
    case DF_WIRE_EVENT_END: return "EVENT_END";
    case DF_WIRE_IMAGE_META: return "IMAGE_META";
    case DF_WIRE_IMAGE_CHUNK: return "IMAGE_CHUNK";
    default: return "UNKNOWN";
    }
}

uint16_t df_crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xffffU;
    size_t i;
    int bit;
    if (!data) return 0;
    for (i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000U) crc = (uint16_t)((crc << 1) ^ 0x1021U);
            else crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t df_telemetry_encode(const MonitorMessage *message,
                           uint16_t source_device_id,
                           uint8_t *out,
                           size_t out_capacity) {
    DfWirePacketType type;
    const DebrisEventSummary *e;
    uint8_t *p;
    uint8_t flags = 0;
    uint16_t crc;
    const size_t total_size = DF_WIRE_HEADER_SIZE + DF_TELEMETRY_PAYLOAD_SIZE + DF_WIRE_CRC_SIZE;

    if (!message || !out || out_capacity < total_size) return 0;
    type = wire_type_from_monitor(message->type);
    if ((int)type == 0) return 0;
    e = &message->event;

    out[0] = DF_WIRE_MAGIC0;
    out[1] = DF_WIRE_MAGIC1;
    out[2] = DF_WIRE_VERSION;
    out[3] = (uint8_t)type;
    put_u16_le(out + 4, (uint16_t)DF_TELEMETRY_PAYLOAD_SIZE);

    p = out + DF_WIRE_HEADER_SIZE;
    put_u16_le(p, source_device_id); p += 2;
    put_u32_le(p, sat_u32_u64(message->sequence)); p += 4;
    put_u32_le(p, sat_u32_u64(e->event_id)); p += 4;
    put_u32_le(p, sat_u32_u64(e->start_epoch_ms / 1000ULL)); p += 4;
    put_u32_le(p, sat_u32_u64(e->end_epoch_ms / 1000ULL)); p += 4;
    put_u32_le(p, sat_u32_u64(e->duration_ms)); p += 4;
    put_u32_le(p, sat_u32_u64(e->observed_frames)); p += 4;
    put_u32_le(p, sat_u32_u64(e->active_frames)); p += 4;
    put_u32_le(p, sat_u32_u64(e->recovering_frames)); p += 4;
    put_u16_le(p, ratio_x10000(e->max_gully_ratio)); p += 2;
    put_u16_le(p, ratio_x10000(e->max_moving_ratio)); p += 2;
    put_u16_le(p, ratio_x10000(e->max_blob_ratio)); p += 2;
    put_u32_le(p, sat_u32_u64(e->max_blob_area)); p += 4;
    *p++ = sat_u8_u64(e->max_valid_blobs < 0 ? 0U : (uint64_t)e->max_valid_blobs);
    put_u16_le(p, speed_x10(e->max_speed_px_s)); p += 2;
    put_u16_le(p, speed_x10(e->avg_speed_px_s)); p += 2;
    *p++ = (uint8_t)e->dominant_direction;
    *p++ = consistency_255(e->direction_consistency);
    put_u16_le(p, sat_u16_u64(e->direction_samples)); p += 2;
    *p++ = sat_u8_u64(e->snapshot_count);
    *p++ = (uint8_t)e->end_reason;
    if (e->end_epoch_ms >= DF_CLOCK_VALID_MIN_EPOCH_MS ||
        e->start_epoch_ms >= DF_CLOCK_VALID_MIN_EPOCH_MS)
        flags |= DF_WIRE_FLAG_CLOCK_VALID;
    *p++ = flags;
    *p++ = 0U; /* hop_count: source node starts at zero. */

    if ((size_t)(p - (out + DF_WIRE_HEADER_SIZE)) != DF_TELEMETRY_PAYLOAD_SIZE) return 0;

    crc = df_crc16_ccitt(out + 2, 4U + DF_TELEMETRY_PAYLOAD_SIZE);
    put_u16_le(out + DF_WIRE_HEADER_SIZE + DF_TELEMETRY_PAYLOAD_SIZE, crc);
    return total_size;
}

int df_wire_peek_frame_length(const uint8_t *buffer,
                              size_t buffer_len,
                              size_t *frame_len_out) {
    uint16_t payload_len;
    size_t total;
    if (!buffer || !frame_len_out) return -1;
    if (buffer_len < DF_WIRE_HEADER_SIZE) return 1;
    if (buffer[0] != DF_WIRE_MAGIC0 || buffer[1] != DF_WIRE_MAGIC1) return -2;
    if (buffer[2] != DF_WIRE_VERSION) return -3;
    payload_len = get_u16_le(buffer + 4);
    if (payload_len > DF_WIRE_MAX_PAYLOAD) return -4;
    total = DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE;
    *frame_len_out = total;
    return buffer_len < total ? 1 : 0;
}

int df_telemetry_decode(const uint8_t *frame,
                        size_t frame_len,
                        DfTelemetryDecoded *out) {
    uint16_t payload_len;
    uint16_t expected_crc;
    uint16_t actual_crc;
    const uint8_t *p;
    size_t expected_total;

    if (!frame || !out) return -1;
    if (frame_len < DF_WIRE_HEADER_SIZE + DF_WIRE_CRC_SIZE) return -2;
    if (frame[0] != DF_WIRE_MAGIC0 || frame[1] != DF_WIRE_MAGIC1) return -3;
    if (frame[2] != DF_WIRE_VERSION) return -4;
    if (frame[3] < DF_WIRE_EVENT_START || frame[3] > DF_WIRE_EVENT_END) return -5;
    payload_len = get_u16_le(frame + 4);
    if (payload_len != DF_TELEMETRY_PAYLOAD_SIZE) return -6;
    expected_total = DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE;
    if (frame_len != expected_total) return -7;
    expected_crc = get_u16_le(frame + DF_WIRE_HEADER_SIZE + payload_len);
    actual_crc = df_crc16_ccitt(frame + 2, 4U + payload_len);
    if (expected_crc != actual_crc) return -8;

    memset(out, 0, sizeof(*out));
    out->type = (DfWirePacketType)frame[3];
    p = frame + DF_WIRE_HEADER_SIZE;
    out->source_device_id = get_u16_le(p); p += 2;
    out->sequence = get_u32_le(p); p += 4;
    out->event_id = get_u32_le(p); p += 4;
    out->start_epoch_s = get_u32_le(p); p += 4;
    out->end_epoch_s = get_u32_le(p); p += 4;
    out->duration_ms = get_u32_le(p); p += 4;
    out->observed_frames = get_u32_le(p); p += 4;
    out->active_frames = get_u32_le(p); p += 4;
    out->recovering_frames = get_u32_le(p); p += 4;
    out->max_gully_x10000 = get_u16_le(p); p += 2;
    out->max_moving_x10000 = get_u16_le(p); p += 2;
    out->max_blob_x10000 = get_u16_le(p); p += 2;
    out->max_blob_area = get_u32_le(p); p += 4;
    out->max_valid_blobs = *p++;
    out->max_speed_x10 = get_u16_le(p); p += 2;
    out->avg_speed_x10 = get_u16_le(p); p += 2;
    out->direction = *p++;
    out->direction_consistency_255 = *p++;
    out->direction_samples = get_u16_le(p); p += 2;
    out->snapshot_count = *p++;
    out->end_reason = *p++;
    out->flags = *p++;
    out->hop_count = *p++;
    return 0;
}
