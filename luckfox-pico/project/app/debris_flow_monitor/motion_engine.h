#ifndef MOTION_ENGINE_H
#define MOTION_ENGINE_H

#include "debris_config.h"
#include "image_utils.h"
#include "monitor_queue.h"
#include "motion_blob.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint64_t frame_count;

    U16Image background_q8;
    bool background_ready;
    double background_mean;

    U8Image previous_gray;
    bool previous_ready;
    double previous_mean;

    U8Image gully_mask;
    U8Image static_mask;
    U8Image blob_foreground_mask;
    uint64_t gully_pixels;
    uint64_t static_pixels;
    uint64_t outside_zone_pixels[DF_OUTSIDE_ZONE_COUNT];
    bool roi_ready;
    bool roi_debug_written;

    int *blob_flood_queue;
    size_t blob_flood_queue_capacity;
    MotionBlobList motion_blobs;

    uint64_t settled_new_pose_frames;
    uint64_t rebase_hold_remaining;
    uint64_t rebase_count;
    uint64_t camera_disturbance_hold_remaining;
    bool post_shake_pending;
    uint64_t post_shake_stable_frames;
    uint64_t post_shake_guard_frames;

    TemporalEventTracker event_tracker;
    DominantBlobTracker dominant_tracker;
    int next_track_id;

    EventManager event_manager;

    bool buffers_ready;
} MotionEngine;

void motion_engine_init(MotionEngine *engine);
void motion_engine_print_config(void);
void motion_engine_deinit(MotionEngine *engine);
int motion_engine_process_frame(MotionEngine *engine, const GrayImage *gray);
void motion_engine_drain_queues(MotionEngine *engine);
void motion_engine_finish_process(MotionEngine *engine);

#endif
