#include "monitor_queue.h"

#include <stdio.h>
#include <string.h>

const char *temporal_event_state_name(TemporalEventState state) {
    switch (state) {
    case TEMPORAL_IDLE: return "IDLE";
    case TEMPORAL_POSSIBLE_FLOW: return "POSSIBLE_FLOW";
    case TEMPORAL_ACTIVE_FLOW: return "ACTIVE_FLOW";
    case TEMPORAL_RECOVERING: return "RECOVERING";
    default: return "UNKNOWN";
    }
}

void temporal_event_reset(TemporalEventTracker *tracker) {
    if (!tracker) return;
    memset(tracker, 0, sizeof(*tracker));
    tracker->state = TEMPORAL_IDLE;
}

void temporal_event_update(TemporalEventTracker *tracker,
                           bool instant_evidence,
                           bool hold_evidence,
                           bool camera_stable) {
    if (!tracker) return;
    if (!camera_stable) { temporal_event_reset(tracker); return; }
    switch (tracker->state) {
    case TEMPORAL_IDLE:
        if (instant_evidence) { tracker->state = TEMPORAL_POSSIBLE_FLOW; tracker->confirm_frames = 1; }
        else tracker->confirm_frames = 0;
        break;
    case TEMPORAL_POSSIBLE_FLOW:
        if (instant_evidence) {
            ++tracker->confirm_frames;
            if (tracker->confirm_frames >= DF_EVENT_CONFIRM_FRAMES) {
                tracker->state = TEMPORAL_ACTIVE_FLOW;
                tracker->miss_frames = 0;
                tracker->recovery_frames = 0;
            }
        } else temporal_event_reset(tracker);
        break;
    case TEMPORAL_ACTIVE_FLOW:
        if (hold_evidence) tracker->miss_frames = 0;
        else if (++tracker->miss_frames >= DF_EVENT_MISS_TO_RECOVER_FRAMES) {
            tracker->state = TEMPORAL_RECOVERING;
            tracker->recovery_frames = 0;
        }
        break;
    case TEMPORAL_RECOVERING:
        if (instant_evidence) {
            tracker->state = TEMPORAL_ACTIVE_FLOW;
            tracker->miss_frames = 0;
            tracker->recovery_frames = 0;
        } else if (++tracker->recovery_frames >= DF_EVENT_RECOVERY_FRAMES) {
            temporal_event_reset(tracker);
        }
        break;
    }
}

static const char *debris_end_reason_name(DebrisEventEndReason reason) {
    switch (reason) {
    case DEBRIS_END_NORMAL: return "NORMAL";
    case DEBRIS_END_CAMERA_UNSTABLE: return "CAMERA_UNSTABLE";
    case DEBRIS_END_PROCESS_STOPPED: return "PROCESS_STOPPED";
    case DEBRIS_END_NONE:
    default: return "NONE";
    }
}

static const char *monitor_message_type_name(MonitorMessageType type) {
    switch (type) {
    case MONITOR_EVENT_START: return "EVENT_START";
    case MONITOR_EVENT_UPDATE: return "EVENT_UPDATE";
    case MONITOR_SNAPSHOT_READY: return "SNAPSHOT_READY";
    case MONITOR_EVENT_END: return "EVENT_END";
    default: return "UNKNOWN";
    }
}

static void reset_event(DebrisEventAccumulator *event) {
    if (event) memset(event, 0, sizeof(*event));
}

static void reset_monitor_queue(MonitorMessageQueue *queue) {
    if (queue) memset(queue, 0, sizeof(*queue));
}

static void reset_snapshot_queue(SnapshotPackageQueue *queue) {
    if (queue) memset(queue, 0, sizeof(*queue));
}

static bool push_snapshot_package(SnapshotPackageQueue *queue, const SnapshotPackage *package) {
    if (queue->count >= DF_SNAPSHOT_PACKAGE_QUEUE_CAPACITY) {
        queue->head = (queue->head + 1U) % DF_SNAPSHOT_PACKAGE_QUEUE_CAPACITY;
        --queue->count;
        ++queue->dropped_packages;
        fprintf(stderr, "[IMAGE-QUEUE] overflow, drop oldest totalDropped=%llu\n",
                (unsigned long long)queue->dropped_packages);
    }
    queue->packages[queue->tail] = *package;
    queue->tail = (queue->tail + 1U) % DF_SNAPSHOT_PACKAGE_QUEUE_CAPACITY;
    ++queue->count;
    return true;
}

static bool pop_snapshot_package(SnapshotPackageQueue *queue, SnapshotPackage *package) {
    if (!queue || !package || queue->count == 0) return false;
    *package = queue->packages[queue->head];
    queue->head = (queue->head + 1U) % DF_SNAPSHOT_PACKAGE_QUEUE_CAPACITY;
    --queue->count;
    return true;
}

static bool push_monitor_message(MonitorMessageQueue *queue, const MonitorMessage *message) {
    if (queue->count >= DF_MONITOR_MESSAGE_QUEUE_CAPACITY) {
        queue->head = (queue->head + 1U) % DF_MONITOR_MESSAGE_QUEUE_CAPACITY;
        --queue->count;
        ++queue->dropped_messages;
        fprintf(stderr, "[QUEUE] overflow, drop oldest totalDropped=%llu\n",
                (unsigned long long)queue->dropped_messages);
    }
    queue->messages[queue->tail] = *message;
    queue->tail = (queue->tail + 1U) % DF_MONITOR_MESSAGE_QUEUE_CAPACITY;
    ++queue->count;
    return true;
}

static bool pop_monitor_message(MonitorMessageQueue *queue, MonitorMessage *message) {
    if (!queue || !message || queue->count == 0) return false;
    *message = queue->messages[queue->head];
    queue->head = (queue->head + 1U) % DF_MONITOR_MESSAGE_QUEUE_CAPACITY;
    --queue->count;
    return true;
}

static CoarseDirection dominant_event_direction(const DebrisEventAccumulator *event, double *consistency) {
    uint64_t best_votes = 0;
    CoarseDirection best = DIR_UNKNOWN;
    int i;
    *consistency = 0.0;
    if (!event || event->direction_samples == 0) return DIR_UNKNOWN;
    for (i = 2; i < DF_COARSE_DIRECTION_COUNT; ++i) {
        if (event->direction_votes[i] > best_votes) {
            best_votes = event->direction_votes[i];
            best = (CoarseDirection)i;
        }
    }
    if (best_votes == 0) return DIR_UNKNOWN;
    *consistency = (double)best_votes / (double)event->direction_samples;
    return best;
}

static DebrisEventSummary build_summary(const DebrisEventAccumulator *event,
                                        uint64_t end_epoch_ms,
                                        double now_monotonic,
                                        DebrisEventEndReason end_reason) {
    DebrisEventSummary summary;
    memset(&summary, 0, sizeof(summary));
    summary.event_id = event->event_id;
    summary.start_epoch_ms = event->start_epoch_ms;
    summary.end_epoch_ms = end_epoch_ms;
    if (now_monotonic >= event->start_monotonic)
        summary.duration_ms = (uint64_t)((now_monotonic - event->start_monotonic) * 1000.0);
    summary.observed_frames = event->observed_frames;
    summary.active_frames = event->active_frames;
    summary.recovering_frames = event->recovering_frames;
    summary.max_gully_ratio = event->max_gully_ratio;
    summary.max_moving_ratio = event->max_moving_ratio;
    summary.max_blob_ratio = event->max_blob_ratio;
    summary.max_blob_area = event->max_blob_area;
    summary.max_valid_blobs = event->max_valid_blobs;
    summary.max_speed_px_s = event->max_speed_px_s;
    if (event->speed_samples > 0)
        summary.avg_speed_px_s = event->speed_sum_px_s / (double)event->speed_samples;
    summary.dominant_direction = dominant_event_direction(event, &summary.direction_consistency);
    summary.direction_samples = event->direction_samples;
    summary.snapshot_count = event->snapshot_count;
    snprintf(summary.latest_snapshot_path, sizeof(summary.latest_snapshot_path), "%s",
             event->latest_snapshot_path);
    summary.end_reason = end_reason;
    return summary;
}

static void push_event_summary(MonitorMessageQueue *queue,
                               MonitorMessageType type,
                               uint64_t *next_sequence,
                               const DebrisEventAccumulator *event,
                               uint64_t now_epoch_ms,
                               double now_monotonic,
                               DebrisEventEndReason end_reason) {
    MonitorMessage message;
    memset(&message, 0, sizeof(message));
    message.type = type;
    message.sequence = (*next_sequence)++;
    message.event = build_summary(event, now_epoch_ms, now_monotonic, end_reason);
    push_monitor_message(queue, &message);
}

static void update_metrics(DebrisEventAccumulator *event,
                           TemporalEventState temporal_state,
                           double gully_ratio,
                           double moving_ratio,
                           double blob_ratio,
                           const BlobFrameStats *blob_stats,
                           const DominantBlobTracker *tracker) {
    int dir_index;
    ++event->observed_frames;
    if (temporal_state == TEMPORAL_ACTIVE_FLOW) ++event->active_frames;
    else if (temporal_state == TEMPORAL_RECOVERING) ++event->recovering_frames;
    if (gully_ratio > event->max_gully_ratio) event->max_gully_ratio = gully_ratio;
    if (moving_ratio > event->max_moving_ratio) event->max_moving_ratio = moving_ratio;
    if (blob_ratio > event->max_blob_ratio) event->max_blob_ratio = blob_ratio;
    if ((uint64_t)blob_stats->largest.area > event->max_blob_area)
        event->max_blob_area = (uint64_t)blob_stats->largest.area;
    if (blob_stats->valid_blob_count > event->max_valid_blobs)
        event->max_valid_blobs = blob_stats->valid_blob_count;
    if (!tracker->valid || tracker->missed_frames != 0) return;
    if (tracker->speed_px_s > event->max_speed_px_s) event->max_speed_px_s = tracker->speed_px_s;
    if (tracker->speed_px_s > 0.0) {
        event->speed_sum_px_s += tracker->speed_px_s;
        ++event->speed_samples;
    }
    dir_index = (int)tracker->direction;
    if (tracker->speed_px_s >= DF_EVENT_DIRECTION_VOTE_MIN_SPEED_PX_S &&
        dir_index >= 2 && dir_index < DF_COARSE_DIRECTION_COUNT) {
        ++event->direction_votes[dir_index];
        ++event->direction_samples;
    }
}

static void start_event(DebrisEventAccumulator *event, uint64_t event_id,
                        uint64_t now_epoch_ms, double now_monotonic) {
    reset_event(event);
    event->active = true;
    event->event_id = event_id;
    event->start_epoch_ms = now_epoch_ms;
    event->start_monotonic = now_monotonic;
    event->last_update_message_monotonic = now_monotonic;
    event->close_pending = false;
    event->close_pending_since = 0.0;
}

static void capture_snapshot(EventManager *manager,
                             const GrayImage *gray,
                             uint64_t now_epoch_ms,
                             double now_monotonic) {
    DebrisEventAccumulator *event = &manager->event;
    SnapshotPackage package;
    char path[128];
    uint64_t slot;
    bool ok;
    if (!event->active) return;
    if (manager->last_global_snapshot_monotonic > 0.0 &&
        now_monotonic - manager->last_global_snapshot_monotonic < DF_EVENT_SNAPSHOT_INTERVAL_SECONDS)
        return;
    slot = manager->snapshot_slot_counter % DF_SNAPSHOT_RING_SLOTS;
    ++manager->snapshot_slot_counter;
    snprintf(path, sizeof(path), "/tmp/debris_snapshot_%02llu.pgm", (unsigned long long)slot);
    ok = write_gray_snapshot_pgm(path, gray);
    manager->last_global_snapshot_monotonic = now_monotonic;
    if (!ok) {
        fprintf(stderr, "[SNAPSHOT] write failed event=%llu path=%s\n",
                (unsigned long long)event->event_id, path);
        return;
    }
    ++event->snapshot_count;
    snprintf(event->latest_snapshot_path, sizeof(event->latest_snapshot_path), "%s", path);
    memset(&package, 0, sizeof(package));
    package.sequence = manager->next_snapshot_sequence++;
    package.event_id = event->event_id;
    package.capture_epoch_ms = now_epoch_ms;
    package.width = gray->width;
    package.height = gray->height;
    snprintf(package.path, sizeof(package.path), "%s", path);
    push_snapshot_package(&manager->snapshot_queue, &package);
}

static void finish_event(EventManager *manager, DebrisEventEndReason reason,
                         uint64_t now_epoch_ms, double now_monotonic) {
    if (!manager->event.active) return;
    push_event_summary(&manager->telemetry_queue, MONITOR_EVENT_END,
                       &manager->next_telemetry_sequence, &manager->event,
                       now_epoch_ms, now_monotonic, reason);
    reset_event(&manager->event);
}

void event_manager_init(EventManager *manager) {
    if (!manager) return;
    memset(manager, 0, sizeof(*manager));
    reset_event(&manager->event);
    reset_monitor_queue(&manager->telemetry_queue);
    reset_snapshot_queue(&manager->snapshot_queue);
    manager->next_event_id = 1;
    manager->next_telemetry_sequence = 1;
    manager->next_snapshot_sequence = 1;
}

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
    double now_monotonic) {
    uint64_t now_epoch_ms;
    DebrisEventAccumulator *event;
    if (!manager || !blob_stats || !tracker || !gray) return;
    now_epoch_ms = wall_clock_epoch_ms();
    event = &manager->event;

    if (!camera_stable) {
        if (event->active) finish_event(manager, DEBRIS_END_CAMERA_UNSTABLE, now_epoch_ms, now_monotonic);
        return;
    }

    if (!event->active && temporal_state == TEMPORAL_ACTIVE_FLOW) {
        start_event(event, manager->next_event_id++, now_epoch_ms, now_monotonic);
        update_metrics(event, temporal_state, gully_ratio, moving_ratio, blob_ratio, blob_stats, tracker);
        push_event_summary(&manager->telemetry_queue, MONITOR_EVENT_START,
                           &manager->next_telemetry_sequence, event,
                           now_epoch_ms, now_monotonic, DEBRIS_END_NONE);
        capture_snapshot(manager, gray, now_epoch_ms, now_monotonic);
        return;
    }

    if (!event->active) return;

    if (temporal_state == TEMPORAL_ACTIVE_FLOW ||
        temporal_state == TEMPORAL_RECOVERING ||
        temporal_state == TEMPORAL_POSSIBLE_FLOW) {
        event->close_pending = false;
        event->close_pending_since = 0.0;
        update_metrics(event, temporal_state, gully_ratio, moving_ratio, blob_ratio, blob_stats, tracker);
    } else {
        if (!event->close_pending) {
            event->close_pending = true;
            event->close_pending_since = now_monotonic;
        }
        if (now_monotonic - event->close_pending_since >= DF_EVENT_CLOSE_GRACE_SECONDS) {
            finish_event(manager, DEBRIS_END_NORMAL, now_epoch_ms, now_monotonic);
            return;
        }
    }

    if (now_monotonic - event->last_update_message_monotonic >= DF_EVENT_SUMMARY_UPDATE_INTERVAL_SECONDS) {
        push_event_summary(&manager->telemetry_queue, MONITOR_EVENT_UPDATE,
                           &manager->next_telemetry_sequence, event,
                           now_epoch_ms, now_monotonic, DEBRIS_END_NONE);
        event->last_update_message_monotonic = now_monotonic;
    }
    capture_snapshot(manager, gray, now_epoch_ms, now_monotonic);
}

void event_manager_finish_process(EventManager *manager) {
    if (!manager || !manager->event.active) return;
    finish_event(manager, DEBRIS_END_PROCESS_STOPPED,
                 wall_clock_epoch_ms(), monotonic_seconds());
}

bool event_manager_pop_telemetry(EventManager *manager, MonitorMessage *message) {
    if (!manager) return false;
    return pop_monitor_message(&manager->telemetry_queue, message);
}

bool event_manager_pop_snapshot(EventManager *manager, SnapshotPackage *package) {
    if (!manager) return false;
    return pop_snapshot_package(&manager->snapshot_queue, package);
}

void monitor_message_log(const MonitorMessage *message, const char *channel) {
    if (!message) return;
    if (!channel) channel = "DATA";
    if (message->type == MONITOR_SNAPSHOT_READY) {
        printf("[QUEUE-%s] seq=%llu type=%s event=%llu snapshotEpochMs=%llu path=%s\n",
               channel,
               (unsigned long long)message->sequence,
               monitor_message_type_name(message->type),
               (unsigned long long)message->event.event_id,
               (unsigned long long)message->snapshot_epoch_ms,
               message->snapshot_path);
        fflush(stdout);
        return;
    }
    printf("[QUEUE-%s] seq=%llu type=%s event=%llu startMs=%llu endMs=%llu durationMs=%llu "
           "frames=%llu activeFrames=%llu recoverFrames=%llu maxGully=%.3f maxMoving=%.3f "
           "maxBlob=%.3f maxBlobArea=%llu maxValidBlobs=%d maxSpeed=%.1f avgSpeed=%.1f "
           "dir=%s dirConsistency=%.3f dirSamples=%llu snapshots=%llu lastSnapshot=%s endReason=%s\n",
           channel,
           (unsigned long long)message->sequence,
           monitor_message_type_name(message->type),
           (unsigned long long)message->event.event_id,
           (unsigned long long)message->event.start_epoch_ms,
           (unsigned long long)message->event.end_epoch_ms,
           (unsigned long long)message->event.duration_ms,
           (unsigned long long)message->event.observed_frames,
           (unsigned long long)message->event.active_frames,
           (unsigned long long)message->event.recovering_frames,
           message->event.max_gully_ratio,
           message->event.max_moving_ratio,
           message->event.max_blob_ratio,
           (unsigned long long)message->event.max_blob_area,
           message->event.max_valid_blobs,
           message->event.max_speed_px_s,
           message->event.avg_speed_px_s,
           coarse_direction_name(message->event.dominant_direction),
           message->event.direction_consistency,
           (unsigned long long)message->event.direction_samples,
           (unsigned long long)message->event.snapshot_count,
           message->event.latest_snapshot_path[0] ? message->event.latest_snapshot_path : "-",
           debris_end_reason_name(message->event.end_reason));
    fflush(stdout);
}

void snapshot_package_log(const SnapshotPackage *package) {
    if (!package) return;
    printf("[QUEUE-IMAGE] seq=%llu event=%llu captureMs=%llu size=%dx%d path=%s format=PGM_GRAY\n",
           (unsigned long long)package->sequence,
           (unsigned long long)package->event_id,
           (unsigned long long)package->capture_epoch_ms,
           package->width, package->height, package->path);
    fflush(stdout);
}

void event_manager_drain_to_log(EventManager *manager) {
    MonitorMessage message;
    SnapshotPackage package;
    if (!manager) return;
    while (event_manager_pop_telemetry(manager, &message)) monitor_message_log(&message, "DATA");
    while (event_manager_pop_snapshot(manager, &package)) snapshot_package_log(&package);
}
