#ifndef MOTION_BLOB_H
#define MOTION_BLOB_H

#include "image_utils.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int area;
    int min_x;
    int min_y;
    int max_x;
    int max_y;
    double cx;
    double cy;
} MotionBlob;

typedef struct {
    MotionBlob *items;
    size_t count;
    size_t capacity;
} MotionBlobList;

typedef struct {
    int raw_component_count;
    int valid_blob_count;
    uint64_t total_valid_area;
    double combined_cx;
    double combined_cy;
    MotionBlob largest;
} BlobFrameStats;

typedef enum {
    DIR_UNKNOWN = 0,
    DIR_STILL,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_UP,
    DIR_DOWN,
    DIR_LEFT_UP,
    DIR_LEFT_DOWN,
    DIR_RIGHT_UP,
    DIR_RIGHT_DOWN
} CoarseDirection;

typedef struct {
    bool valid;
    int id;
    uint64_t age_frames;
    uint64_t missed_frames;
    MotionBlob blob;
    double last_seen_time;
    double anchor_cx;
    double anchor_cy;
    double anchor_time;
    uint64_t anchor_frame;
    double dx;
    double dy;
    double vx;
    double vy;
    double speed_px_s;
    CoarseDirection direction;
    uint64_t direction_streak;
} DominantBlobTracker;

int motion_blob_list_alloc(MotionBlobList *list, size_t capacity);
void motion_blob_list_free(MotionBlobList *list);
void motion_blob_list_clear(MotionBlobList *list);

BlobFrameStats motion_blob_extract_in_place(
    U8Image *motion_mask,
    int *flood_queue,
    size_t flood_queue_capacity,
    MotionBlobList *blobs,
    int min_area);

const char *coarse_direction_name(CoarseDirection direction);
const char *coarse_speed_name(double speed_px_s);
CoarseDirection coarse_direction_from_delta(double dx, double dy);

void dominant_blob_tracker_reset(DominantBlobTracker *tracker);
void dominant_blob_tracker_update(
    DominantBlobTracker *tracker,
    const MotionBlobList *blobs,
    uint64_t frame_count,
    double now_seconds,
    bool camera_stable,
    int *next_track_id);

#endif
