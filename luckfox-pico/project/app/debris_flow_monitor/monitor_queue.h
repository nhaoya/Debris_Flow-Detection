#ifndef MONITOR_QUEUE_H
#define MONITOR_QUEUE_H

#include "debris_config.h"
#include "image_utils.h"
#include "motion_blob.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    TEMPORAL_IDLE = 0,
    TEMPORAL_POSSIBLE_FLOW,
    TEMPORAL_ACTIVE_FLOW,
    TEMPORAL_RECOVERING
} TemporalEventState;

typedef struct {
    TemporalEventState state;
    uint64_t confirm_frames;
    uint64_t miss_frames;
    uint64_t recovery_frames;
} TemporalEventTracker;

typedef enum {
    DEBRIS_END_NONE = 0,
    DEBRIS_END_NORMAL,
    DEBRIS_END_CAMERA_UNSTABLE,
    DEBRIS_END_PROCESS_STOPPED
} DebrisEventEndReason;

typedef enum {
    MONITOR_EVENT_START = 0,
    MONITOR_EVENT_UPDATE,
    MONITOR_SNAPSHOT_READY,
    MONITOR_EVENT_END
} MonitorMessageType;

typedef struct {
    uint64_t event_id;
    uint64_t start_epoch_ms;
    uint64_t end_epoch_ms;
    uint64_t duration_ms;
    uint64_t observed_frames;
    uint64_t active_frames;
    uint64_t recovering_frames;
    double max_gully_ratio;
    double max_moving_ratio;
    double max_blob_ratio;
    uint64_t max_blob_area;
    int max_valid_blobs;
    double max_speed_px_s;
    double avg_speed_px_s;
    CoarseDirection dominant_direction;
    double direction_consistency;
    uint64_t direction_samples;
    uint64_t snapshot_count;
    char latest_snapshot_path[128];
    DebrisEventEndReason end_reason;
} DebrisEventSummary;

typedef struct {
    bool active;
    uint64_t event_id;
    uint64_t start_epoch_ms;
    double start_monotonic;
    double last_update_message_monotonic;
    bool close_pending;
    double close_pending_since;
    uint64_t observed_frames;
    uint64_t active_frames;
    uint64_t recovering_frames;
    double max_gully_ratio;
    double max_moving_ratio;
    double max_blob_ratio;
    uint64_t max_blob_area;
    int max_valid_blobs;
    double max_speed_px_s;
    double speed_sum_px_s;
    uint64_t speed_samples;
    uint64_t direction_votes[DF_COARSE_DIRECTION_COUNT];
    uint64_t direction_samples;
    uint64_t snapshot_count;
    char latest_snapshot_path[128];
} DebrisEventAccumulator;

typedef struct {
    MonitorMessageType type;
    uint64_t sequence;
    DebrisEventSummary event;
    uint64_t snapshot_epoch_ms;
    char snapshot_path[128];
} MonitorMessage;

typedef struct {
    MonitorMessage messages[DF_MONITOR_MESSAGE_QUEUE_CAPACITY];
    size_t head;
    size_t tail;
    size_t count;
    uint64_t dropped_messages;
} MonitorMessageQueue;

typedef struct {
    uint64_t sequence;
    uint64_t event_id;
    uint64_t capture_epoch_ms;
    int width;
    int height;
    char path[128];
} SnapshotPackage;

typedef struct {
    SnapshotPackage packages[DF_SNAPSHOT_PACKAGE_QUEUE_CAPACITY];
    size_t head;
    size_t tail;
    size_t count;
    uint64_t dropped_packages;
} SnapshotPackageQueue;

typedef struct {
    DebrisEventAccumulator event;
    MonitorMessageQueue telemetry_queue;
    SnapshotPackageQueue snapshot_queue;
    uint64_t next_event_id;
    uint64_t next_telemetry_sequence;
    uint64_t next_snapshot_sequence;
    uint64_t snapshot_slot_counter;
    double last_global_snapshot_monotonic;
} EventManager;

const char *temporal_event_state_name(TemporalEventState state);
void temporal_event_reset(TemporalEventTracker *tracker);
void temporal_event_update(TemporalEventTracker *tracker,
                           bool instant_evidence,
                           bool hold_evidence,
                           bool camera_stable);

void event_manager_init(EventManager *manager);
void event_manager_update(
    EventManager *manager,
    TemporalEventState temporal_state,
    bool camera_stable,
    double gully_ratio,
    double moving_ratio,
    double blob_ratio,
    const BlobFrameStats *blob_stats,
    const DominantBlobTracker *tracker,
    const GrayImage *gray,
    double now_monotonic);
void event_manager_finish_process(EventManager *manager);

/*
 * Queue consumer API. The EventManager remains the producer; transport layers
 * can pop messages after VI frame release without touching temporal logic.
 */
bool event_manager_pop_telemetry(EventManager *manager, MonitorMessage *message);
bool event_manager_pop_snapshot(EventManager *manager, SnapshotPackage *package);
void monitor_message_log(const MonitorMessage *message, const char *channel);
void snapshot_package_log(const SnapshotPackage *package);

/* Legacy log-only drain kept for algorithm-only runs/tests. */
void event_manager_drain_to_log(EventManager *manager);

#endif
