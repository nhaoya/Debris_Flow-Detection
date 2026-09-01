#include "telemetry_protocol.h"

#include <limits.h>
#include <stdbool.h>
#include <string.h>

#define DF_TELEMETRY_PAYLOAD_SIZE 57U
#define DF_HEARTBEAT_PAYLOAD_SIZE 15U
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
    case DF_WIRE_HEARTBEAT: return "HEARTBEAT";
    case DF_WIRE_IMAGE_META: return "IMAGE_META";
    case DF_WIRE_IMAGE_CHUNK: return "IMAGE_CHUNK";
    case DF_WIRE_IMAGE_DONE: return "IMAGE_DONE";
    case DF_WIRE_IMAGE_FEEDBACK: return "IMAGE_FEEDBACK";
    case DF_WIRE_EVENT_ACK: return "EVENT_ACK";
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

int df_wire_set_hop_count(uint8_t *frame, size_t frame_len, uint8_t hop_count) {
    uint16_t payload_len;
    uint16_t crc;
    size_t expected_total;
    uint8_t type;
    if (!frame || frame_len < DF_WIRE_HEADER_SIZE + DF_WIRE_CRC_SIZE) return -1;
    if (frame[0] != DF_WIRE_MAGIC0 || frame[1] != DF_WIRE_MAGIC1 ||
        frame[2] != DF_WIRE_VERSION) return -2;
    type = frame[3];
    if (!((type >= (uint8_t)DF_WIRE_EVENT_START && type <= (uint8_t)DF_WIRE_EVENT_END) ||
          type == (uint8_t)DF_WIRE_IMAGE_META)) return -3;
    payload_len = get_u16_le(frame + 4);
    expected_total = DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE;
    if (frame_len != expected_total || payload_len == 0U) return -4;
    frame[DF_WIRE_HEADER_SIZE + payload_len - 1U] = hop_count;
    crc = df_crc16_ccitt(frame + 2, 4U + payload_len);
    put_u16_le(frame + DF_WIRE_HEADER_SIZE + payload_len, crc);
    return 0;
}

int df_wire_increment_hop_count(uint8_t *frame, size_t frame_len, uint8_t *new_hop_out) {
    uint16_t payload_len;
    uint8_t hop;
    if (!frame || frame_len < DF_WIRE_HEADER_SIZE + DF_WIRE_CRC_SIZE) return -1;
    payload_len = get_u16_le(frame + 4);
    if (payload_len == 0U || frame_len != DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE)
        return -2;
    hop = frame[DF_WIRE_HEADER_SIZE + payload_len - 1U];
    if (hop == 0xffU) return -3;
    ++hop;
    if (df_wire_set_hop_count(frame, frame_len, hop) != 0) return -4;
    if (new_hop_out) *new_hop_out = hop;
    return 0;
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


size_t df_heartbeat_encode(uint16_t source_device_id,
                           uint32_t sequence,
                           uint32_t uptime_s,
                           uint32_t tx_ok,
                           uint8_t flags,
                           uint8_t *out,
                           size_t out_capacity) {
    uint8_t *p;
    uint16_t crc;
    const size_t total_size = DF_WIRE_HEADER_SIZE + DF_HEARTBEAT_PAYLOAD_SIZE + DF_WIRE_CRC_SIZE;
    if (!out || out_capacity < total_size) return 0U;

    out[0] = DF_WIRE_MAGIC0;
    out[1] = DF_WIRE_MAGIC1;
    out[2] = DF_WIRE_VERSION;
    out[3] = (uint8_t)DF_WIRE_HEARTBEAT;
    put_u16_le(out + 4, (uint16_t)DF_HEARTBEAT_PAYLOAD_SIZE);
    p = out + DF_WIRE_HEADER_SIZE;
    put_u16_le(p, source_device_id); p += 2;
    put_u32_le(p, sequence); p += 4;
    put_u32_le(p, uptime_s); p += 4;
    put_u32_le(p, tx_ok); p += 4;
    *p++ = flags;
    if ((size_t)(p - (out + DF_WIRE_HEADER_SIZE)) != DF_HEARTBEAT_PAYLOAD_SIZE) return 0U;
    crc = df_crc16_ccitt(out + 2, 4U + DF_HEARTBEAT_PAYLOAD_SIZE);
    put_u16_le(out + DF_WIRE_HEADER_SIZE + DF_HEARTBEAT_PAYLOAD_SIZE, crc);
    return total_size;
}

int df_heartbeat_decode(const uint8_t *frame,
                        size_t frame_len,
                        DfHeartbeatDecoded *out) {
    uint16_t payload_len;
    uint16_t expected_crc;
    uint16_t actual_crc;
    const uint8_t *p;
    size_t expected_total;
    if (!frame || !out) return -1;
    if (frame_len < DF_WIRE_HEADER_SIZE + DF_WIRE_CRC_SIZE) return -2;
    if (frame[0] != DF_WIRE_MAGIC0 || frame[1] != DF_WIRE_MAGIC1) return -3;
    if (frame[2] != DF_WIRE_VERSION || frame[3] != (uint8_t)DF_WIRE_HEARTBEAT) return -4;
    payload_len = get_u16_le(frame + 4);
    if (payload_len != DF_HEARTBEAT_PAYLOAD_SIZE) return -5;
    expected_total = DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE;
    if (frame_len != expected_total) return -6;
    expected_crc = get_u16_le(frame + DF_WIRE_HEADER_SIZE + payload_len);
    actual_crc = df_crc16_ccitt(frame + 2, 4U + payload_len);
    if (expected_crc != actual_crc) return -7;
    memset(out, 0, sizeof(*out));
    p = frame + DF_WIRE_HEADER_SIZE;
    out->source_device_id = get_u16_le(p); p += 2;
    out->sequence = get_u32_le(p); p += 4;
    out->uptime_s = get_u32_le(p); p += 4;
    out->tx_ok = get_u32_le(p); p += 4;
    out->flags = *p++;
    return 0;
}

#define DF_IMAGE_META_PAYLOAD_SIZE 29U
#define DF_IMAGE_CHUNK_FIXED_PAYLOAD_SIZE 16U

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
                            size_t out_capacity) {
    uint8_t *p;
    uint8_t flags = 0U;
    uint16_t crc;
    const size_t total = DF_WIRE_HEADER_SIZE + DF_IMAGE_META_PAYLOAD_SIZE + DF_WIRE_CRC_SIZE;
    if (!out || out_capacity < total || width == 0U || height == 0U || total_size == 0U ||
        total_chunks == 0U || (purpose != DF_IMAGE_PURPOSE_EVENT && purpose != DF_IMAGE_PURPOSE_DEPLOY_PREVIEW))
        return 0U;

    out[0] = DF_WIRE_MAGIC0;
    out[1] = DF_WIRE_MAGIC1;
    out[2] = DF_WIRE_VERSION;
    out[3] = (uint8_t)DF_WIRE_IMAGE_META;
    put_u16_le(out + 4, (uint16_t)DF_IMAGE_META_PAYLOAD_SIZE);
    p = out + DF_WIRE_HEADER_SIZE;
    put_u16_le(p, source_device_id); p += 2;
    put_u32_le(p, image_id); p += 4;
    put_u32_le(p, event_id); p += 4;
    put_u32_le(p, sat_u32_u64(capture_epoch_ms / 1000ULL)); p += 4;
    put_u32_le(p, total_size); p += 4;
    put_u16_le(p, width); p += 2;
    put_u16_le(p, height); p += 2;
    put_u16_le(p, total_chunks); p += 2;
    *p++ = quality;
    *p++ = (uint8_t)purpose;
    *p++ = DF_IMAGE_FORMAT_JPEG_GRAY;
    if (capture_epoch_ms >= DF_CLOCK_VALID_MIN_EPOCH_MS) flags |= DF_WIRE_FLAG_CLOCK_VALID;
    *p++ = flags;
    *p++ = 0U; /* hop_count */
    if ((size_t)(p - (out + DF_WIRE_HEADER_SIZE)) != DF_IMAGE_META_PAYLOAD_SIZE) return 0U;
    crc = df_crc16_ccitt(out + 2, 4U + DF_IMAGE_META_PAYLOAD_SIZE);
    put_u16_le(out + DF_WIRE_HEADER_SIZE + DF_IMAGE_META_PAYLOAD_SIZE, crc);
    return total;
}

int df_image_meta_decode(const uint8_t *frame,
                         size_t frame_len,
                         DfImageMetaDecoded *out) {
    uint16_t payload_len, expected_crc, actual_crc;
    const uint8_t *p;
    size_t expected_total;
    if (!frame || !out) return -1;
    if (frame_len < DF_WIRE_HEADER_SIZE + DF_WIRE_CRC_SIZE) return -2;
    if (frame[0] != DF_WIRE_MAGIC0 || frame[1] != DF_WIRE_MAGIC1) return -3;
    if (frame[2] != DF_WIRE_VERSION || frame[3] != (uint8_t)DF_WIRE_IMAGE_META) return -4;
    payload_len = get_u16_le(frame + 4);
    if (payload_len != DF_IMAGE_META_PAYLOAD_SIZE) return -5;
    expected_total = DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE;
    if (frame_len != expected_total) return -6;
    expected_crc = get_u16_le(frame + DF_WIRE_HEADER_SIZE + payload_len);
    actual_crc = df_crc16_ccitt(frame + 2, 4U + payload_len);
    if (expected_crc != actual_crc) return -7;
    memset(out, 0, sizeof(*out));
    p = frame + DF_WIRE_HEADER_SIZE;
    out->source_device_id = get_u16_le(p); p += 2;
    out->image_id = get_u32_le(p); p += 4;
    out->event_id = get_u32_le(p); p += 4;
    out->capture_epoch_s = get_u32_le(p); p += 4;
    out->total_size = get_u32_le(p); p += 4;
    out->width = get_u16_le(p); p += 2;
    out->height = get_u16_le(p); p += 2;
    out->total_chunks = get_u16_le(p); p += 2;
    out->quality = *p++;
    out->purpose = *p++;
    out->format = *p++;
    out->flags = *p++;
    out->hop_count = *p++;
    return 0;
}

size_t df_image_chunk_encode(uint16_t source_device_id,
                             uint32_t image_id,
                             uint32_t event_id,
                             uint16_t chunk_index,
                             uint16_t total_chunks,
                             DfImagePurpose purpose,
                             const uint8_t *data,
                             uint8_t data_len,
                             uint8_t *out,
                             size_t out_capacity) {
    uint8_t *p;
    uint16_t payload_len, crc;
    size_t total;
    if (!data || data_len == 0U || data_len > DF_IMAGE_CHUNK_DATA_MAX || !out || total_chunks == 0U ||
        chunk_index >= total_chunks ||
        (purpose != DF_IMAGE_PURPOSE_EVENT && purpose != DF_IMAGE_PURPOSE_DEPLOY_PREVIEW))
        return 0U;
    payload_len = (uint16_t)(DF_IMAGE_CHUNK_FIXED_PAYLOAD_SIZE + data_len);
    total = DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE;
    if (out_capacity < total) return 0U;
    out[0] = DF_WIRE_MAGIC0;
    out[1] = DF_WIRE_MAGIC1;
    out[2] = DF_WIRE_VERSION;
    out[3] = (uint8_t)DF_WIRE_IMAGE_CHUNK;
    put_u16_le(out + 4, payload_len);
    p = out + DF_WIRE_HEADER_SIZE;
    put_u16_le(p, source_device_id); p += 2;
    put_u32_le(p, image_id); p += 4;
    put_u32_le(p, event_id); p += 4;
    put_u16_le(p, chunk_index); p += 2;
    put_u16_le(p, total_chunks); p += 2;
    *p++ = data_len;
    *p++ = (uint8_t)purpose;
    memcpy(p, data, data_len); p += data_len;
    if ((size_t)(p - (out + DF_WIRE_HEADER_SIZE)) != payload_len) return 0U;
    crc = df_crc16_ccitt(out + 2, 4U + payload_len);
    put_u16_le(out + DF_WIRE_HEADER_SIZE + payload_len, crc);
    return total;
}

int df_image_chunk_decode(const uint8_t *frame,
                          size_t frame_len,
                          DfImageChunkDecoded *out) {
    uint16_t payload_len, expected_crc, actual_crc;
    const uint8_t *p;
    size_t expected_total;
    uint8_t data_len;
    if (!frame || !out) return -1;
    if (frame_len < DF_WIRE_HEADER_SIZE + DF_WIRE_CRC_SIZE) return -2;
    if (frame[0] != DF_WIRE_MAGIC0 || frame[1] != DF_WIRE_MAGIC1) return -3;
    if (frame[2] != DF_WIRE_VERSION || frame[3] != (uint8_t)DF_WIRE_IMAGE_CHUNK) return -4;
    payload_len = get_u16_le(frame + 4);
    if (payload_len < DF_IMAGE_CHUNK_FIXED_PAYLOAD_SIZE || payload_len > DF_WIRE_MAX_PAYLOAD) return -5;
    expected_total = DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE;
    if (frame_len != expected_total) return -6;
    expected_crc = get_u16_le(frame + DF_WIRE_HEADER_SIZE + payload_len);
    actual_crc = df_crc16_ccitt(frame + 2, 4U + payload_len);
    if (expected_crc != actual_crc) return -7;
    memset(out, 0, sizeof(*out));
    p = frame + DF_WIRE_HEADER_SIZE;
    out->source_device_id = get_u16_le(p); p += 2;
    out->image_id = get_u32_le(p); p += 4;
    out->event_id = get_u32_le(p); p += 4;
    out->chunk_index = get_u16_le(p); p += 2;
    out->total_chunks = get_u16_le(p); p += 2;
    data_len = *p++;
    out->purpose = *p++;
    if (data_len == 0U || data_len > DF_IMAGE_CHUNK_DATA_MAX ||
        payload_len != (uint16_t)(DF_IMAGE_CHUNK_FIXED_PAYLOAD_SIZE + data_len)) return -8;
    out->data_len = data_len;
    memcpy(out->data, p, data_len);
    return 0;
}

#define DF_IMAGE_DONE_PAYLOAD_SIZE 15U
#define DF_IMAGE_FEEDBACK_FIXED_PAYLOAD_SIZE 18U
#define DF_EVENT_ACK_PAYLOAD_SIZE 14U

size_t df_image_done_encode(uint16_t source_device_id,
                            uint32_t image_id,
                            uint32_t event_id,
                            uint16_t total_chunks,
                            uint8_t round,
                            DfImagePurpose purpose,
                            uint8_t *out,
                            size_t out_capacity) {
    uint8_t *p;
    uint16_t crc;
    const size_t total = DF_WIRE_HEADER_SIZE + DF_IMAGE_DONE_PAYLOAD_SIZE + DF_WIRE_CRC_SIZE;
    if (!out || out_capacity < total || total_chunks == 0U ||
        (purpose != DF_IMAGE_PURPOSE_EVENT && purpose != DF_IMAGE_PURPOSE_DEPLOY_PREVIEW))
        return 0U;
    out[0] = DF_WIRE_MAGIC0;
    out[1] = DF_WIRE_MAGIC1;
    out[2] = DF_WIRE_VERSION;
    out[3] = (uint8_t)DF_WIRE_IMAGE_DONE;
    put_u16_le(out + 4, (uint16_t)DF_IMAGE_DONE_PAYLOAD_SIZE);
    p = out + DF_WIRE_HEADER_SIZE;
    put_u16_le(p, source_device_id); p += 2;
    put_u32_le(p, image_id); p += 4;
    put_u32_le(p, event_id); p += 4;
    put_u16_le(p, total_chunks); p += 2;
    *p++ = round;
    *p++ = (uint8_t)purpose;
    *p++ = 0U;
    if ((size_t)(p - (out + DF_WIRE_HEADER_SIZE)) != DF_IMAGE_DONE_PAYLOAD_SIZE) return 0U;
    crc = df_crc16_ccitt(out + 2, 4U + DF_IMAGE_DONE_PAYLOAD_SIZE);
    put_u16_le(out + DF_WIRE_HEADER_SIZE + DF_IMAGE_DONE_PAYLOAD_SIZE, crc);
    return total;
}

int df_image_done_decode(const uint8_t *frame,
                         size_t frame_len,
                         DfImageDoneDecoded *out) {
    uint16_t payload_len, expected_crc, actual_crc;
    const uint8_t *p;
    size_t expected_total;
    if (!frame || !out) return -1;
    if (frame_len < DF_WIRE_HEADER_SIZE + DF_WIRE_CRC_SIZE) return -2;
    if (frame[0] != DF_WIRE_MAGIC0 || frame[1] != DF_WIRE_MAGIC1) return -3;
    if (frame[2] != DF_WIRE_VERSION || frame[3] != (uint8_t)DF_WIRE_IMAGE_DONE) return -4;
    payload_len = get_u16_le(frame + 4);
    if (payload_len != DF_IMAGE_DONE_PAYLOAD_SIZE) return -5;
    expected_total = DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE;
    if (frame_len != expected_total) return -6;
    expected_crc = get_u16_le(frame + DF_WIRE_HEADER_SIZE + payload_len);
    actual_crc = df_crc16_ccitt(frame + 2, 4U + payload_len);
    if (expected_crc != actual_crc) return -7;
    memset(out, 0, sizeof(*out));
    p = frame + DF_WIRE_HEADER_SIZE;
    out->source_device_id = get_u16_le(p); p += 2;
    out->image_id = get_u32_le(p); p += 4;
    out->event_id = get_u32_le(p); p += 4;
    out->total_chunks = get_u16_le(p); p += 2;
    out->round = *p++;
    out->purpose = *p++;
    out->flags = *p++;
    if (out->total_chunks == 0U ||
        (out->purpose != (uint8_t)DF_IMAGE_PURPOSE_EVENT &&
         out->purpose != (uint8_t)DF_IMAGE_PURPOSE_DEPLOY_PREVIEW))
        return -8;
    return 0;
}

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
                                size_t out_capacity) {
    uint8_t *p;
    uint16_t payload_len, crc;
    size_t total;
    const bool complete = (flags & DF_IMAGE_FEEDBACK_FLAG_COMPLETE) != 0U;
    if (!out || total_chunks == 0U) return 0U;
    if (complete) {
        bitmap_len = 0U;
    } else {
        uint8_t required = (uint8_t)((total_chunks + 7U) / 8U);
        if (!missing_bitmap || bitmap_len != required || bitmap_len > DF_IMAGE_FEEDBACK_BITMAP_MAX_BYTES)
            return 0U;
    }
    payload_len = (uint16_t)(DF_IMAGE_FEEDBACK_FIXED_PAYLOAD_SIZE + bitmap_len);
    total = DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE;
    if (payload_len > DF_WIRE_MAX_PAYLOAD || out_capacity < total) return 0U;
    out[0] = DF_WIRE_MAGIC0;
    out[1] = DF_WIRE_MAGIC1;
    out[2] = DF_WIRE_VERSION;
    out[3] = (uint8_t)DF_WIRE_IMAGE_FEEDBACK;
    put_u16_le(out + 4, payload_len);
    p = out + DF_WIRE_HEADER_SIZE;
    put_u16_le(p, responder_device_id); p += 2;
    put_u16_le(p, source_device_id); p += 2;
    put_u32_le(p, image_id); p += 4;
    put_u32_le(p, event_id); p += 4;
    put_u16_le(p, total_chunks); p += 2;
    *p++ = round;
    *p++ = flags;
    *p++ = bitmap_len;
    *p++ = 0U;
    if (bitmap_len > 0U) {
        memcpy(p, missing_bitmap, bitmap_len);
        p += bitmap_len;
    }
    if ((size_t)(p - (out + DF_WIRE_HEADER_SIZE)) != payload_len) return 0U;
    crc = df_crc16_ccitt(out + 2, 4U + payload_len);
    put_u16_le(out + DF_WIRE_HEADER_SIZE + payload_len, crc);
    return total;
}

int df_image_feedback_decode(const uint8_t *frame,
                             size_t frame_len,
                             DfImageFeedbackDecoded *out) {
    uint16_t payload_len, expected_crc, actual_crc;
    const uint8_t *p;
    size_t expected_total;
    uint8_t bitmap_len;
    bool complete;
    if (!frame || !out) return -1;
    if (frame_len < DF_WIRE_HEADER_SIZE + DF_WIRE_CRC_SIZE) return -2;
    if (frame[0] != DF_WIRE_MAGIC0 || frame[1] != DF_WIRE_MAGIC1) return -3;
    if (frame[2] != DF_WIRE_VERSION || frame[3] != (uint8_t)DF_WIRE_IMAGE_FEEDBACK) return -4;
    payload_len = get_u16_le(frame + 4);
    if (payload_len < DF_IMAGE_FEEDBACK_FIXED_PAYLOAD_SIZE || payload_len > DF_WIRE_MAX_PAYLOAD) return -5;
    expected_total = DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE;
    if (frame_len != expected_total) return -6;
    expected_crc = get_u16_le(frame + DF_WIRE_HEADER_SIZE + payload_len);
    actual_crc = df_crc16_ccitt(frame + 2, 4U + payload_len);
    if (expected_crc != actual_crc) return -7;
    memset(out, 0, sizeof(*out));
    p = frame + DF_WIRE_HEADER_SIZE;
    out->responder_device_id = get_u16_le(p); p += 2;
    out->source_device_id = get_u16_le(p); p += 2;
    out->image_id = get_u32_le(p); p += 4;
    out->event_id = get_u32_le(p); p += 4;
    out->total_chunks = get_u16_le(p); p += 2;
    out->round = *p++;
    out->flags = *p++;
    bitmap_len = *p++;
    p++; /* reserved */
    if (bitmap_len > DF_IMAGE_FEEDBACK_BITMAP_MAX_BYTES ||
        payload_len != (uint16_t)(DF_IMAGE_FEEDBACK_FIXED_PAYLOAD_SIZE + bitmap_len)) return -8;
    complete = (out->flags & DF_IMAGE_FEEDBACK_FLAG_COMPLETE) != 0U;
    if (complete) {
        if (bitmap_len != 0U) return -9;
    } else {
        uint8_t required = (uint8_t)((out->total_chunks + 7U) / 8U);
        if (out->total_chunks == 0U || bitmap_len != required) return -10;
    }
    out->bitmap_len = bitmap_len;
    if (bitmap_len > 0U) memcpy(out->missing_bitmap, p, bitmap_len);
    return 0;
}

size_t df_event_ack_encode(uint16_t responder_device_id,
                           uint16_t source_device_id,
                           uint32_t sequence,
                           uint32_t event_id,
                           DfWirePacketType event_type,
                           uint8_t *out,
                           size_t out_capacity) {
    uint8_t *p;
    uint16_t crc;
    const size_t total = DF_WIRE_HEADER_SIZE + DF_EVENT_ACK_PAYLOAD_SIZE + DF_WIRE_CRC_SIZE;
    if (!out || out_capacity < total ||
        (event_type != DF_WIRE_EVENT_START && event_type != DF_WIRE_EVENT_END))
        return 0U;
    out[0] = DF_WIRE_MAGIC0;
    out[1] = DF_WIRE_MAGIC1;
    out[2] = DF_WIRE_VERSION;
    out[3] = (uint8_t)DF_WIRE_EVENT_ACK;
    put_u16_le(out + 4, (uint16_t)DF_EVENT_ACK_PAYLOAD_SIZE);
    p = out + DF_WIRE_HEADER_SIZE;
    put_u16_le(p, responder_device_id); p += 2;
    put_u16_le(p, source_device_id); p += 2;
    put_u32_le(p, sequence); p += 4;
    put_u32_le(p, event_id); p += 4;
    *p++ = (uint8_t)event_type;
    *p++ = 0U;
    if ((size_t)(p - (out + DF_WIRE_HEADER_SIZE)) != DF_EVENT_ACK_PAYLOAD_SIZE) return 0U;
    crc = df_crc16_ccitt(out + 2, 4U + DF_EVENT_ACK_PAYLOAD_SIZE);
    put_u16_le(out + DF_WIRE_HEADER_SIZE + DF_EVENT_ACK_PAYLOAD_SIZE, crc);
    return total;
}

int df_event_ack_decode(const uint8_t *frame,
                        size_t frame_len,
                        DfEventAckDecoded *out) {
    uint16_t payload_len, expected_crc, actual_crc;
    const uint8_t *p;
    size_t expected_total;
    if (!frame || !out) return -1;
    if (frame_len < DF_WIRE_HEADER_SIZE + DF_WIRE_CRC_SIZE) return -2;
    if (frame[0] != DF_WIRE_MAGIC0 || frame[1] != DF_WIRE_MAGIC1) return -3;
    if (frame[2] != DF_WIRE_VERSION || frame[3] != (uint8_t)DF_WIRE_EVENT_ACK) return -4;
    payload_len = get_u16_le(frame + 4);
    if (payload_len != DF_EVENT_ACK_PAYLOAD_SIZE) return -5;
    expected_total = DF_WIRE_HEADER_SIZE + (size_t)payload_len + DF_WIRE_CRC_SIZE;
    if (frame_len != expected_total) return -6;
    expected_crc = get_u16_le(frame + DF_WIRE_HEADER_SIZE + payload_len);
    actual_crc = df_crc16_ccitt(frame + 2, 4U + payload_len);
    if (expected_crc != actual_crc) return -7;
    memset(out, 0, sizeof(*out));
    p = frame + DF_WIRE_HEADER_SIZE;
    out->responder_device_id = get_u16_le(p); p += 2;
    out->source_device_id = get_u16_le(p); p += 2;
    out->sequence = get_u32_le(p); p += 4;
    out->event_id = get_u32_le(p); p += 4;
    out->event_type = *p++;
    out->flags = *p++;
    if (out->event_type != (uint8_t)DF_WIRE_EVENT_START &&
        out->event_type != (uint8_t)DF_WIRE_EVENT_END) return -8;
    return 0;
}

