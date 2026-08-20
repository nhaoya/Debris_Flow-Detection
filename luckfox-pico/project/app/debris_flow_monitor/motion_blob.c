#include "motion_blob.h"
#include "debris_config.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

int motion_blob_list_alloc(MotionBlobList *list, size_t capacity) {
    if (!list || capacity == 0) return -1;
    free(list->items);
    memset(list, 0, sizeof(*list));
    list->items = (MotionBlob *)calloc(capacity, sizeof(MotionBlob));
    if (!list->items) return -1;
    list->capacity = capacity;
    return 0;
}

void motion_blob_list_free(MotionBlobList *list) {
    if (!list) return;
    free(list->items);
    memset(list, 0, sizeof(*list));
}

void motion_blob_list_clear(MotionBlobList *list) {
    if (list) list->count = 0;
}

BlobFrameStats motion_blob_extract_in_place(
    U8Image *motion_mask,
    int *flood_queue,
    size_t flood_queue_capacity,
    MotionBlobList *blobs,
    int min_area) {
    BlobFrameStats stats;
    uint64_t valid_sum_x = 0;
    uint64_t valid_sum_y = 0;
    int width, height, x, y;

    memset(&stats, 0, sizeof(stats));
    if (blobs) blobs->count = 0;
    if (!motion_mask || !motion_mask->data || !flood_queue || !blobs) return stats;

    width = motion_mask->width;
    height = motion_mask->height;
    if ((size_t)width * (size_t)height > flood_queue_capacity) return stats;

    for (y = 0; y < height; ++y) {
        uint8_t *scan_row = motion_mask->data + (size_t)y * motion_mask->stride;
        for (x = 0; x < width; ++x) {
            size_t queue_head, queue_tail;
            int area, min_x, min_y, max_x, max_y;
            uint64_t sum_x, sum_y;
            MotionBlob blob;

            if (scan_row[x] == 0) continue;
            ++stats.raw_component_count;
            queue_head = 0;
            queue_tail = 0;
            scan_row[x] = 0;
            flood_queue[queue_tail++] = y * width + x;
            area = 0;
            min_x = max_x = x;
            min_y = max_y = y;
            sum_x = sum_y = 0;

            while (queue_head < queue_tail) {
                const int index = flood_queue[queue_head++];
                const int cy = index / width;
                const int cx = index - cy * width;
                int dy, dx;
                ++area;
                sum_x += (uint64_t)cx;
                sum_y += (uint64_t)cy;
                if (cx < min_x) min_x = cx;
                if (cx > max_x) max_x = cx;
                if (cy < min_y) min_y = cy;
                if (cy > max_y) max_y = cy;

                for (dy = -1; dy <= 1; ++dy) {
                    const int ny = cy + dy;
                    uint8_t *neighbor_row;
                    if (ny < 0 || ny >= height) continue;
                    neighbor_row = motion_mask->data + (size_t)ny * motion_mask->stride;
                    for (dx = -1; dx <= 1; ++dx) {
                        const int nx = cx + dx;
                        if (dx == 0 && dy == 0) continue;
                        if (nx < 0 || nx >= width) continue;
                        if (neighbor_row[nx] == 0) continue;
                        neighbor_row[nx] = 0;
                        flood_queue[queue_tail++] = ny * width + nx;
                    }
                }
            }

            if (area < min_area) continue;
            memset(&blob, 0, sizeof(blob));
            blob.area = area;
            blob.min_x = min_x;
            blob.min_y = min_y;
            blob.max_x = max_x;
            blob.max_y = max_y;
            blob.cx = (double)sum_x / (double)area;
            blob.cy = (double)sum_y / (double)area;

            if (blobs->count < blobs->capacity) blobs->items[blobs->count++] = blob;
            ++stats.valid_blob_count;
            stats.total_valid_area += (uint64_t)area;
            valid_sum_x += sum_x;
            valid_sum_y += sum_y;
            if (stats.largest.area == 0 || blob.area > stats.largest.area) stats.largest = blob;
        }
    }

    if (stats.total_valid_area > 0) {
        stats.combined_cx = (double)valid_sum_x / (double)stats.total_valid_area;
        stats.combined_cy = (double)valid_sum_y / (double)stats.total_valid_area;
    }
    return stats;
}

const char *coarse_direction_name(CoarseDirection direction) {
    switch (direction) {
    case DIR_STILL: return "STILL";
    case DIR_LEFT: return "LEFT";
    case DIR_RIGHT: return "RIGHT";
    case DIR_UP: return "UP";
    case DIR_DOWN: return "DOWN";
    case DIR_LEFT_UP: return "LEFT_UP";
    case DIR_LEFT_DOWN: return "LEFT_DOWN";
    case DIR_RIGHT_UP: return "RIGHT_UP";
    case DIR_RIGHT_DOWN: return "RIGHT_DOWN";
    case DIR_UNKNOWN:
    default: return "UNKNOWN";
    }
}

const char *coarse_speed_name(double speed_px_s) {
    if (speed_px_s < 20.0) return "SLOW";
    if (speed_px_s < 100.0) return "MEDIUM";
    return "FAST";
}

CoarseDirection coarse_direction_from_delta(double dx, double dy) {
    const double distance = sqrt(dx * dx + dy * dy);
    const double abs_x = fabs(dx);
    const double abs_y = fabs(dy);
    if (distance < DF_TRACK_DIRECTION_MIN_DISPLACEMENT_PX) return DIR_STILL;
    if (abs_x >= abs_y * 1.5) return dx >= 0.0 ? DIR_RIGHT : DIR_LEFT;
    if (abs_y >= abs_x * 1.5) return dy >= 0.0 ? DIR_DOWN : DIR_UP;
    if (dx >= 0.0 && dy >= 0.0) return DIR_RIGHT_DOWN;
    if (dx >= 0.0 && dy < 0.0) return DIR_RIGHT_UP;
    if (dx < 0.0 && dy >= 0.0) return DIR_LEFT_DOWN;
    return DIR_LEFT_UP;
}

static double blob_box_iou(const MotionBlob *a, const MotionBlob *b) {
    int ix0, iy0, ix1, iy1;
    double inter_area, a_area, b_area, union_area;
    ix0 = a->min_x > b->min_x ? a->min_x : b->min_x;
    iy0 = a->min_y > b->min_y ? a->min_y : b->min_y;
    ix1 = a->max_x < b->max_x ? a->max_x : b->max_x;
    iy1 = a->max_y < b->max_y ? a->max_y : b->max_y;
    if (ix1 < ix0 || iy1 < iy0) return 0.0;
    inter_area = (double)((ix1 - ix0 + 1) * (iy1 - iy0 + 1));
    a_area = (double)((a->max_x - a->min_x + 1) * (a->max_y - a->min_y + 1));
    b_area = (double)((b->max_x - b->min_x + 1) * (b->max_y - b->min_y + 1));
    union_area = a_area + b_area - inter_area;
    return union_area > 0.0 ? inter_area / union_area : 0.0;
}

void dominant_blob_tracker_reset(DominantBlobTracker *tracker) {
    if (!tracker) return;
    memset(tracker, 0, sizeof(*tracker));
    tracker->direction = DIR_UNKNOWN;
}

static const MotionBlob *find_largest_trackable_blob(const MotionBlobList *blobs) {
    const MotionBlob *best = NULL;
    size_t i;
    if (!blobs) return NULL;
    for (i = 0; i < blobs->count; ++i) {
        const MotionBlob *blob = &blobs->items[i];
        if (blob->area < DF_TRACK_MIN_BLOB_AREA) continue;
        if (!best || blob->area > best->area) best = blob;
    }
    return best;
}

static const MotionBlob *find_best_blob_match(const DominantBlobTracker *tracker,
                                               const MotionBlobList *blobs) {
    const MotionBlob *best = NULL;
    double best_score = 1.0e30;
    size_t i;
    if (!tracker || !blobs) return NULL;
    for (i = 0; i < blobs->count; ++i) {
        const MotionBlob *blob = &blobs->items[i];
        const double dx = blob->cx - tracker->blob.cx;
        const double dy = blob->cy - tracker->blob.cy;
        const double distance = sqrt(dx * dx + dy * dy);
        const int min_area = blob->area < tracker->blob.area ? blob->area : tracker->blob.area;
        const int max_area = blob->area > tracker->blob.area ? blob->area : tracker->blob.area;
        double area_ratio, iou, area_change, score;
        if (blob->area < DF_TRACK_MIN_BLOB_AREA || min_area <= 0) continue;
        area_ratio = (double)max_area / (double)min_area;
        iou = blob_box_iou(&tracker->blob, blob);
        if (distance > DF_TRACK_MAX_MATCH_DISTANCE_PX && iou < 0.03) continue;
        if (area_ratio > DF_TRACK_MAX_AREA_RATIO) continue;
        area_change = fabs((double)(blob->area - tracker->blob.area)) / (double)max_area;
        score = distance + 50.0 * area_change - 40.0 * iou;
        if (score < best_score) { best_score = score; best = blob; }
    }
    return best;
}

static void start_track(DominantBlobTracker *tracker, const MotionBlob *blob,
                        int track_id, uint64_t frame_count, double now_seconds) {
    dominant_blob_tracker_reset(tracker);
    tracker->valid = true;
    tracker->id = track_id;
    tracker->age_frames = 1;
    tracker->missed_frames = 0;
    tracker->blob = *blob;
    tracker->last_seen_time = now_seconds;
    tracker->anchor_cx = blob->cx;
    tracker->anchor_cy = blob->cy;
    tracker->anchor_time = now_seconds;
    tracker->anchor_frame = frame_count;
}

void dominant_blob_tracker_update(
    DominantBlobTracker *tracker,
    const MotionBlobList *blobs,
    uint64_t frame_count,
    double now_seconds,
    bool camera_stable,
    int *next_track_id) {
    const MotionBlob *largest;
    const MotionBlob *matched;
    if (!tracker || !blobs || !next_track_id) return;
    if (!camera_stable) { dominant_blob_tracker_reset(tracker); return; }

    if (!tracker->valid) {
        const MotionBlob *first = find_largest_trackable_blob(blobs);
        if (!first) return;
        start_track(tracker, first, (*next_track_id)++, frame_count, now_seconds);
        return;
    }

    largest = find_largest_trackable_blob(blobs);
    matched = find_best_blob_match(tracker, blobs);

    if (largest && matched && largest != matched && matched->area > 0 &&
        (double)largest->area >= (double)matched->area * DF_TRACK_DOMINANT_REPLACEMENT_AREA_FACTOR) {
        start_track(tracker, largest, (*next_track_id)++, frame_count, now_seconds);
        return;
    }

    if (!matched && largest && tracker->blob.area > 0 &&
        (double)largest->area >= (double)tracker->blob.area * DF_TRACK_DOMINANT_REPLACEMENT_AREA_FACTOR) {
        start_track(tracker, largest, (*next_track_id)++, frame_count, now_seconds);
        return;
    }

    if (!matched) {
        ++tracker->missed_frames;
        if (tracker->missed_frames > DF_TRACK_MAX_MISSED_FRAMES) {
            dominant_blob_tracker_reset(tracker);
            if (largest) start_track(tracker, largest, (*next_track_id)++, frame_count, now_seconds);
        }
        return;
    }

    tracker->missed_frames = 0;
    tracker->blob = *matched;
    ++tracker->age_frames;
    tracker->last_seen_time = now_seconds;

    if (frame_count - tracker->anchor_frame >= DF_TRACK_DIRECTION_WINDOW_FRAMES) {
        const double elapsed = now_seconds - tracker->anchor_time;
        const CoarseDirection new_direction = coarse_direction_from_delta(
            tracker->blob.cx - tracker->anchor_cx,
            tracker->blob.cy - tracker->anchor_cy);
        tracker->dx = tracker->blob.cx - tracker->anchor_cx;
        tracker->dy = tracker->blob.cy - tracker->anchor_cy;
        if (elapsed > 0.001) {
            tracker->vx = tracker->dx / elapsed;
            tracker->vy = tracker->dy / elapsed;
            tracker->speed_px_s = sqrt(tracker->vx * tracker->vx + tracker->vy * tracker->vy);
        }
        if (new_direction == tracker->direction) ++tracker->direction_streak;
        else { tracker->direction = new_direction; tracker->direction_streak = 1; }
        tracker->anchor_cx = tracker->blob.cx;
        tracker->anchor_cy = tracker->blob.cy;
        tracker->anchor_time = now_seconds;
        tracker->anchor_frame = frame_count;
    }
}
