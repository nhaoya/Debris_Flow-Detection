#include "motion_engine.h"
#include "debris_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const RoiPoint k_gully_polygon[DF_GULLY_POINT_COUNT] = {
    {225, 132}, {365, 128}, {590, 325}, {155, 355}
};

static const RoiPoint k_static_polygon[DF_STATIC_POINT_COUNT] = {
    {430, 65}, {635, 50}, {635, 175}, {470, 155}
};

/* V1.4.3: 2x2 spatial zones over the full frame. Gully pixels are excluded
 * from the zone population, so each ratio measures only outside-gully motion.
 *   0 = top-left, 1 = top-right, 2 = bottom-left, 3 = bottom-right. */
static unsigned outside_zone_index(int x, int y, int width, int height) {
    const unsigned right = x >= width / 2 ? 1U : 0U;
    const unsigned bottom = y >= height / 2 ? 1U : 0U;
    return bottom * 2U + right;
}

static bool outside_zone_consensus_motion(const double zone_frame_ratio[DF_OUTSIDE_ZONE_COUNT],
                                          unsigned *active_count_out) {
    unsigned active_count = 0U;
    unsigned i;
    for (i = 0U; i < DF_OUTSIDE_ZONE_COUNT; ++i) {
        if (zone_frame_ratio[i] >= DF_OUTSIDE_ZONE_FRAME_THRESHOLD) ++active_count;
    }
    if (active_count_out) *active_count_out = active_count;
    return active_count >= DF_OUTSIDE_ZONE_MIN_ACTIVE_COUNT;
}

static int clamp_brightness_shift(double shift) {
    int applied = shift >= 0.0 ? (int)(shift + 0.5) : (int)(shift - 0.5);
    if (applied > DF_BRIGHTNESS_COMPENSATION_LIMIT) applied = DF_BRIGHTNESS_COMPENSATION_LIMIT;
    if (applied < -DF_BRIGHTNESS_COMPENSATION_LIMIT) applied = -DF_BRIGHTNESS_COMPENSATION_LIMIT;
    return applied;
}

static double initialize_background_q8(const GrayImage *gray, U16Image *background_q8) {
    uint64_t sum_q8 = 0;
    uint64_t pixels;
    int x, y;
    if (u16_image_alloc(background_q8, gray->width, gray->height) != 0) return 0.0;
    for (y = 0; y < gray->height; ++y) {
        const uint8_t *cur = gray->data + (size_t)y * gray->stride;
        uint16_t *bg = background_q8->data + (size_t)y * background_q8->stride;
        for (x = 0; x < gray->width; ++x) {
            const uint16_t value = (uint16_t)((uint16_t)cur[x] << DF_BACKGROUND_FRACTION_BITS);
            bg[x] = value;
            sum_q8 += (uint64_t)value;
        }
    }
    pixels = (uint64_t)gray->width * (uint64_t)gray->height;
    return pixels ? (double)sum_q8 / ((double)pixels * (double)DF_BACKGROUND_SCALE) : 0.0;
}

static int ensure_buffers(MotionEngine *engine, const GrayImage *gray) {
    size_t pixel_count;
    size_t max_valid_blobs;
    bool gully_ok, static_ok;
    int x, y;
    if (engine->buffers_ready && engine->gully_mask.width == gray->width && engine->gully_mask.height == gray->height)
        return 0;

    u8_image_free(&engine->previous_gray);
    u16_image_free(&engine->background_q8);
    u8_image_free(&engine->gully_mask);
    u8_image_free(&engine->static_mask);
    u8_image_free(&engine->blob_foreground_mask);
    free(engine->blob_flood_queue);
    engine->blob_flood_queue = NULL;
    engine->blob_flood_queue_capacity = 0;
    motion_blob_list_free(&engine->motion_blobs);

    engine->gully_pixels = build_polygon_mask(&engine->gully_mask,
        gray->width, gray->height, k_gully_polygon, DF_GULLY_POINT_COUNT, DF_WIDTH, DF_HEIGHT);
    engine->static_pixels = build_polygon_mask(&engine->static_mask,
        gray->width, gray->height, k_static_polygon, DF_STATIC_POINT_COUNT, DF_WIDTH, DF_HEIGHT);
    engine->roi_ready = engine->gully_pixels > 0 && engine->static_pixels > 0;
    if (!engine->roi_ready) return -1;

    memset(engine->outside_zone_pixels, 0, sizeof(engine->outside_zone_pixels));
    for (y = 0; y < gray->height; ++y) {
        const uint8_t *gully_row = engine->gully_mask.data + (size_t)y * engine->gully_mask.stride;
        for (x = 0; x < gray->width; ++x) {
            if (!gully_row[x]) {
                const unsigned zone = outside_zone_index(x, y, gray->width, gray->height);
                ++engine->outside_zone_pixels[zone];
            }
        }
    }

    if (u8_image_alloc(&engine->blob_foreground_mask, gray->width, gray->height) != 0) return -1;

    pixel_count = (size_t)gray->width * (size_t)gray->height;
    engine->blob_flood_queue = (int *)malloc(pixel_count * sizeof(int));
    if (!engine->blob_flood_queue) return -1;
    engine->blob_flood_queue_capacity = pixel_count;

    max_valid_blobs = pixel_count / (size_t)DF_BLOB_MIN_AREA + 32U;
    if (motion_blob_list_alloc(&engine->motion_blobs, max_valid_blobs) != 0) return -1;

    printf("[ROI] initialized size=%ux%u gully_pixels=%llu static_pixels=%llu\n",
           (unsigned)gray->width, (unsigned)gray->height,
           (unsigned long long)engine->gully_pixels,
           (unsigned long long)engine->static_pixels);
    gully_ok = write_mask_pgm("/tmp/debris_gully_mask.pgm", &engine->gully_mask);
    static_ok = write_mask_pgm("/tmp/debris_static_mask.pgm", &engine->static_mask);
    printf("[ROI] masks: gully_pgm=%d static_pgm=%d\n", gully_ok ? 1 : 0, static_ok ? 1 : 0);
    printf("[ROI] outside-zone pixels TL/TR/BL/BR=%llu/%llu/%llu/%llu\n",
           (unsigned long long)engine->outside_zone_pixels[0],
           (unsigned long long)engine->outside_zone_pixels[1],
           (unsigned long long)engine->outside_zone_pixels[2],
           (unsigned long long)engine->outside_zone_pixels[3]);
    fflush(stdout);

    engine->buffers_ready = true;
    return 0;
}

void motion_engine_init(MotionEngine *engine) {
    if (!engine) return;
    memset(engine, 0, sizeof(*engine));
    u8_image_reset(&engine->previous_gray);
    u16_image_reset(&engine->background_q8);
    u8_image_reset(&engine->gully_mask);
    u8_image_reset(&engine->static_mask);
    u8_image_reset(&engine->blob_foreground_mask);
    temporal_event_reset(&engine->event_tracker);
    dominant_blob_tracker_reset(&engine->dominant_tracker);
    engine->next_track_id = 1;
    event_manager_init(&engine->event_manager);
}

void motion_engine_deinit(MotionEngine *engine) {
    if (!engine) return;
    u8_image_free(&engine->previous_gray);
    u16_image_free(&engine->background_q8);
    u8_image_free(&engine->gully_mask);
    u8_image_free(&engine->static_mask);
    u8_image_free(&engine->blob_foreground_mask);
    free(engine->blob_flood_queue);
    engine->blob_flood_queue = NULL;
    engine->blob_flood_queue_capacity = 0;
    motion_blob_list_free(&engine->motion_blobs);
    engine->buffers_ready = false;
}

void motion_engine_print_config(void) {
    printf("[MOTION] V1.4.3 visual config (V1.4.4 transport): warmup=%llu bg_diff=%d frame_diff=%d brightness_limit=+-%d "
           "gully_threshold=%.3f static_threshold=%.3f shake_static_bg=%.3f shake_static_frame=%.3f "
           "rebase_static_bg=%.3f rebase_static_frame<=%.3f rebase_frames=%llu blob_min_area=%d "
           "track_min_area=%d fast_shake_static_frame=%.3f event_update=%.1fs event_merge_grace=%.1fs "
           "snapshot_interval=%.1fs outside_zone>=%.3f zone_consensus=%u/%u shake_hold=%llu snapshot_slots=%llu data_queue=%llu image_queue=%llu\n",
           (unsigned long long)DF_WARMUP_FRAMES,
           DF_BACKGROUND_DIFF_THRESHOLD, DF_FRAME_DIFF_THRESHOLD, DF_BRIGHTNESS_COMPENSATION_LIMIT,
           DF_GULLY_MOTION_THRESHOLD, DF_STATIC_MOTION_THRESHOLD,
           DF_CAMERA_SHAKE_STATIC_BG_THRESHOLD, DF_CAMERA_SHAKE_STATIC_FRAME_THRESHOLD,
           DF_REBASE_STATIC_BG_THRESHOLD, DF_REBASE_STATIC_FRAME_STABLE_THRESHOLD,
           (unsigned long long)DF_REBASE_STABLE_FRAMES,
           DF_BLOB_MIN_AREA, DF_TRACK_MIN_BLOB_AREA, DF_FAST_CAMERA_SHAKE_STATIC_FRAME_THRESHOLD,
           DF_EVENT_SUMMARY_UPDATE_INTERVAL_SECONDS, DF_EVENT_CLOSE_GRACE_SECONDS,
           DF_EVENT_SNAPSHOT_INTERVAL_SECONDS, DF_OUTSIDE_ZONE_FRAME_THRESHOLD,
           (unsigned)DF_OUTSIDE_ZONE_MIN_ACTIVE_COUNT, (unsigned)DF_OUTSIDE_ZONE_COUNT,
           (unsigned long long)DF_CAMERA_DISTURBANCE_HOLD_FRAMES,
           (unsigned long long)DF_SNAPSHOT_RING_SLOTS,
           (unsigned long long)DF_MONITOR_MESSAGE_QUEUE_CAPACITY,
           (unsigned long long)DF_SNAPSHOT_PACKAGE_QUEUE_CAPACITY);
    fflush(stdout);
}

int motion_engine_process_frame(MotionEngine *engine, const GrayImage *gray) {
    double current_mean;
    uint64_t total_pixels;
    int bg_applied_shift, frame_applied_shift;
    double bg_shift, frame_shift;
    uint64_t global_bg_changed = 0, gully_bg_changed = 0, static_bg_changed = 0;
    uint64_t global_frame_changed = 0, gully_frame_changed = 0, static_frame_changed = 0;
    uint64_t bg_sum_diff = 0, frame_sum_diff = 0, moving_overlap_pixels = 0;
    double global_bg_ratio, gully_bg_ratio, static_bg_ratio;
    double global_frame_ratio, gully_frame_ratio, static_frame_ratio;
    uint64_t outside_gully_pixels, outside_gully_frame_changed, outside_gully_bg_changed;
    uint64_t outside_zone_frame_changed[DF_OUTSIDE_ZONE_COUNT] = {0, 0, 0, 0};
    double outside_zone_frame_ratio[DF_OUTSIDE_ZONE_COUNT] = {0.0, 0.0, 0.0, 0.0};
    double outside_gully_frame_ratio, outside_gully_bg_ratio, bg_mean_diff, frame_mean_diff;
    unsigned outside_zone_active_count = 0U;
    bool fast_camera_shake, static_camera_shake, broad_camera_motion, camera_shaking;
    bool possible_new_pose, post_shake_stable, post_shake_rebase_candidate;
    bool rebased_this_frame = false, rebase_hold, static_clean;
    bool instant_event_evidence, hold_event_evidence, camera_stable_for_event;
    BlobFrameStats blob_stats;
    bool blob_analyzed;
    double blob_total_area_ratio, largest_blob_ratio, moving_overlap_ratio, tracking_time;
    bool temporal_candidate, allow_selective_background_update;
    uint64_t learned_pixels = 0, background_sum_q8 = 0;
    double learned_ratio;
    const char *camera_state = "STABLE";
    int x, y;

    if (!engine || !gray || !gray->data || gray->width <= 0 || gray->height <= 0) return -1;
    ++engine->frame_count;
    if (ensure_buffers(engine, gray) != 0) return -1;

    current_mean = gray_mean(gray);

    if (engine->frame_count <= DF_WARMUP_FRAMES) {
        if (engine->frame_count == 1 || engine->frame_count % 30ULL == 0) {
            printf("[WARMUP] frame=%llu/%llu meanY=%.2f\n",
                   (unsigned long long)engine->frame_count,
                   (unsigned long long)DF_WARMUP_FRAMES,
                   current_mean);
        }
        if (engine->frame_count == DF_WARMUP_FRAMES) {
            engine->background_mean = initialize_background_q8(gray, &engine->background_q8);
            engine->background_ready = true;
            if (gray_copy_to_u8(gray, &engine->previous_gray) != 0) return -1;
            engine->previous_mean = current_mean;
            engine->previous_ready = true;
            printf("[MOTION] Q8 background initialized frame=%llu meanY=%.2f\n",
                   (unsigned long long)engine->frame_count, engine->background_mean);
            if (engine->roi_ready && !engine->roi_debug_written) {
                const bool ok = write_roi_debug_ppm("/tmp/debris_roi_debug.ppm", gray,
                                                    &engine->gully_mask, &engine->static_mask);
                engine->roi_debug_written = ok;
                printf("[ROI] debug image: /tmp/debris_roi_debug.ppm write=%d\n", ok ? 1 : 0);
            }
            fflush(stdout);
        }
        return 0;
    }

    if (!engine->background_ready || !engine->previous_ready || !engine->roi_ready) return 0;

    bg_shift = current_mean - engine->background_mean;
    bg_applied_shift = clamp_brightness_shift(bg_shift);
    frame_shift = current_mean - engine->previous_mean;
    frame_applied_shift = clamp_brightness_shift(frame_shift);
    total_pixels = (uint64_t)gray->width * (uint64_t)gray->height;
    u8_image_zero(&engine->blob_foreground_mask);

    for (y = 0; y < gray->height; ++y) {
        const uint8_t *cur = gray->data + (size_t)y * gray->stride;
        const uint8_t *prev = engine->previous_gray.data + (size_t)y * engine->previous_gray.stride;
        const uint16_t *bg_q8 = engine->background_q8.data + (size_t)y * engine->background_q8.stride;
        const uint8_t *gully_row = engine->gully_mask.data + (size_t)y * engine->gully_mask.stride;
        const uint8_t *static_row = engine->static_mask.data + (size_t)y * engine->static_mask.stride;
        uint8_t *blob_row = engine->blob_foreground_mask.data + (size_t)y * engine->blob_foreground_mask.stride;
        for (x = 0; x < gray->width; ++x) {
            const int current = (int)cur[x];
            const int background = ((int)bg_q8[x] + DF_BACKGROUND_SCALE / 2) >> DF_BACKGROUND_FRACTION_BITS;
            int corrected_bg = current - bg_applied_shift;
            int corrected_frame = current - frame_applied_shift;
            int bg_diff, frame_diff;
            if (corrected_bg < 0) corrected_bg = 0; else if (corrected_bg > 255) corrected_bg = 255;
            bg_diff = corrected_bg - background; if (bg_diff < 0) bg_diff = -bg_diff;
            bg_sum_diff += (uint64_t)bg_diff;
            if (bg_diff > DF_BACKGROUND_DIFF_THRESHOLD) {
                ++global_bg_changed;
                if (gully_row[x]) { ++gully_bg_changed; blob_row[x] = 255; }
                if (static_row[x]) ++static_bg_changed;
            }
            if (corrected_frame < 0) corrected_frame = 0; else if (corrected_frame > 255) corrected_frame = 255;
            frame_diff = corrected_frame - (int)prev[x]; if (frame_diff < 0) frame_diff = -frame_diff;
            frame_sum_diff += (uint64_t)frame_diff;
            if (frame_diff > DF_FRAME_DIFF_THRESHOLD) {
                ++global_frame_changed;
                if (gully_row[x]) {
                    ++gully_frame_changed;
                } else {
                    const unsigned zone = outside_zone_index(x, y, gray->width, gray->height);
                    ++outside_zone_frame_changed[zone];
                }
                if (static_row[x]) ++static_frame_changed;
            }
            if (gully_row[x] && bg_diff > DF_BACKGROUND_DIFF_THRESHOLD && frame_diff > DF_FRAME_DIFF_THRESHOLD)
                ++moving_overlap_pixels;
        }
    }

    global_bg_ratio = total_pixels ? (double)global_bg_changed / (double)total_pixels : 0.0;
    gully_bg_ratio = engine->gully_pixels ? (double)gully_bg_changed / (double)engine->gully_pixels : 0.0;
    static_bg_ratio = engine->static_pixels ? (double)static_bg_changed / (double)engine->static_pixels : 0.0;
    global_frame_ratio = total_pixels ? (double)global_frame_changed / (double)total_pixels : 0.0;
    gully_frame_ratio = engine->gully_pixels ? (double)gully_frame_changed / (double)engine->gully_pixels : 0.0;
    static_frame_ratio = engine->static_pixels ? (double)static_frame_changed / (double)engine->static_pixels : 0.0;

    outside_gully_pixels = total_pixels > engine->gully_pixels ? total_pixels - engine->gully_pixels : 0;
    outside_gully_frame_changed = global_frame_changed >= gully_frame_changed ? global_frame_changed - gully_frame_changed : 0;
    outside_gully_bg_changed = global_bg_changed >= gully_bg_changed ? global_bg_changed - gully_bg_changed : 0;
    outside_gully_frame_ratio = outside_gully_pixels ? (double)outside_gully_frame_changed / (double)outside_gully_pixels : 0.0;
    outside_gully_bg_ratio = outside_gully_pixels ? (double)outside_gully_bg_changed / (double)outside_gully_pixels : 0.0;
    bg_mean_diff = total_pixels ? (double)bg_sum_diff / (double)total_pixels : 0.0;
    frame_mean_diff = total_pixels ? (double)frame_sum_diff / (double)total_pixels : 0.0;

    for (unsigned zone = 0U; zone < DF_OUTSIDE_ZONE_COUNT; ++zone) {
        outside_zone_frame_ratio[zone] = engine->outside_zone_pixels[zone]
            ? (double)outside_zone_frame_changed[zone] / (double)engine->outside_zone_pixels[zone]
            : 0.0;
    }

    fast_camera_shake = static_frame_ratio >= DF_FAST_CAMERA_SHAKE_STATIC_FRAME_THRESHOLD;
    static_camera_shake = static_bg_ratio >= DF_CAMERA_SHAKE_STATIC_BG_THRESHOLD &&
                          static_frame_ratio >= DF_CAMERA_SHAKE_STATIC_FRAME_THRESHOLD;
    broad_camera_motion = outside_zone_consensus_motion(outside_zone_frame_ratio,
                                                        &outside_zone_active_count);
    camera_shaking = fast_camera_shake || broad_camera_motion || static_camera_shake;

    if (camera_shaking) {
        engine->camera_disturbance_hold_remaining = DF_CAMERA_DISTURBANCE_HOLD_FRAMES;
        engine->post_shake_pending = true;
        engine->post_shake_stable_frames = 0;
        engine->post_shake_guard_frames = 0;
    } else if (engine->camera_disturbance_hold_remaining > 0) {
        --engine->camera_disturbance_hold_remaining;
    }

    possible_new_pose =
        global_bg_ratio >= DF_REBASE_GLOBAL_BG_THRESHOLD &&
        gully_bg_ratio >= DF_REBASE_GULLY_BG_THRESHOLD &&
        static_bg_ratio >= DF_REBASE_STATIC_BG_THRESHOLD &&
        global_frame_ratio <= DF_REBASE_GLOBAL_FRAME_STABLE_THRESHOLD &&
        gully_frame_ratio <= DF_REBASE_GULLY_FRAME_STABLE_THRESHOLD &&
        static_frame_ratio <= DF_REBASE_STATIC_FRAME_STABLE_THRESHOLD;

    post_shake_stable = global_frame_ratio <= DF_POST_SHAKE_STABLE_FRAME_THRESHOLD &&
                        outside_gully_frame_ratio <= DF_POST_SHAKE_STABLE_FRAME_THRESHOLD;
    post_shake_rebase_candidate = engine->post_shake_pending &&
        engine->camera_disturbance_hold_remaining == 0 && !camera_shaking && post_shake_stable &&
        global_bg_ratio >= DF_POST_SHAKE_REBASE_GLOBAL_BG_THRESHOLD &&
        outside_gully_bg_ratio >= DF_POST_SHAKE_REBASE_OUTSIDE_BG_THRESHOLD;

    if (engine->post_shake_pending && !camera_shaking) {
        ++engine->post_shake_guard_frames;
        if (post_shake_rebase_candidate) ++engine->post_shake_stable_frames;
        else engine->post_shake_stable_frames = 0;
        if (post_shake_stable && global_bg_ratio < 0.05 && outside_gully_bg_ratio < 0.03) {
            engine->post_shake_pending = false;
            engine->post_shake_stable_frames = 0;
            engine->post_shake_guard_frames = 0;
        }
        if (engine->post_shake_guard_frames >= DF_POST_SHAKE_GUARD_MAX_FRAMES && !post_shake_rebase_candidate) {
            engine->post_shake_pending = false;
            engine->post_shake_stable_frames = 0;
            engine->post_shake_guard_frames = 0;
        }
    }

    if (camera_shaking) engine->settled_new_pose_frames = 0;
    else if (possible_new_pose) ++engine->settled_new_pose_frames;
    else engine->settled_new_pose_frames = 0;

    if (engine->settled_new_pose_frames >= DF_REBASE_STABLE_FRAMES ||
        engine->post_shake_stable_frames >= DF_POST_SHAKE_REBASE_STABLE_FRAMES) {
        engine->background_mean = initialize_background_q8(gray, &engine->background_q8);
        engine->settled_new_pose_frames = 0;
        engine->rebase_hold_remaining = DF_REBASE_HOLD_FRAMES;
        engine->camera_disturbance_hold_remaining = 0;
        engine->post_shake_pending = false;
        engine->post_shake_stable_frames = 0;
        engine->post_shake_guard_frames = 0;
        ++engine->rebase_count;
        rebased_this_frame = true;
        temporal_event_reset(&engine->event_tracker);
        dominant_blob_tracker_reset(&engine->dominant_tracker);
        printf("[REBASE] camera settled at new pose, background reset frame=%llu newMean=%.2f count=%llu\n",
               (unsigned long long)engine->frame_count,
               engine->background_mean,
               (unsigned long long)engine->rebase_count);
        fflush(stdout);
    }

    rebase_hold = engine->rebase_hold_remaining > 0;
    static_clean = static_bg_ratio < DF_EVENT_STATIC_BG_VETO_THRESHOLD &&
                   static_frame_ratio < DF_EVENT_STATIC_FRAME_VETO_THRESHOLD;
    instant_event_evidence = gully_bg_ratio >= DF_EVENT_ENTER_GULLY_BG_THRESHOLD &&
                             gully_frame_ratio >= DF_EVENT_ENTER_GULLY_FRAME_THRESHOLD && static_clean;
    hold_event_evidence = gully_bg_ratio >= DF_EVENT_HOLD_GULLY_BG_THRESHOLD &&
                          gully_frame_ratio >= DF_EVENT_HOLD_GULLY_FRAME_THRESHOLD && static_clean;
    camera_stable_for_event = !camera_shaking &&
        engine->camera_disturbance_hold_remaining == 0 &&
        !engine->post_shake_pending && !possible_new_pose && !rebase_hold && !rebased_this_frame;

    memset(&blob_stats, 0, sizeof(blob_stats));
    blob_analyzed = camera_stable_for_event && gully_bg_changed > 0 && gully_frame_changed > 0;
    if (blob_analyzed) {
        blob_stats = motion_blob_extract_in_place(&engine->blob_foreground_mask,
            engine->blob_flood_queue, engine->blob_flood_queue_capacity,
            &engine->motion_blobs, DF_BLOB_MIN_AREA);
    } else motion_blob_list_clear(&engine->motion_blobs);

    blob_total_area_ratio = engine->gully_pixels ? (double)blob_stats.total_valid_area / (double)engine->gully_pixels : 0.0;
    largest_blob_ratio = engine->gully_pixels ? (double)blob_stats.largest.area / (double)engine->gully_pixels : 0.0;
    moving_overlap_ratio = engine->gully_pixels ? (double)moving_overlap_pixels / (double)engine->gully_pixels : 0.0;

    tracking_time = monotonic_seconds();
    dominant_blob_tracker_update(&engine->dominant_tracker, &engine->motion_blobs,
                                 engine->frame_count, tracking_time,
                                 camera_stable_for_event, &engine->next_track_id);
    temporal_event_update(&engine->event_tracker, instant_event_evidence,
                          hold_event_evidence, camera_stable_for_event);
    temporal_candidate = engine->event_tracker.state == TEMPORAL_ACTIVE_FLOW;

    event_manager_update(&engine->event_manager, engine->event_tracker.state,
                         camera_stable_for_event, gully_bg_ratio,
                         moving_overlap_ratio, largest_blob_ratio,
                         &blob_stats, &engine->dominant_tracker, gray, tracking_time);

    if (camera_shaking) camera_state = "SHAKING";
    else if (rebased_this_frame) camera_state = "REBASED";
    else if (rebase_hold) camera_state = "REBASE_HOLD";
    else if (engine->camera_disturbance_hold_remaining > 0) camera_state = "SHAKE_HOLD";
    else if (engine->post_shake_pending) camera_state = "POST_SHAKE_GUARD";
    else if (possible_new_pose) camera_state = "SETTLING_NEW_POSE";

    allow_selective_background_update = !camera_shaking &&
        engine->camera_disturbance_hold_remaining == 0 &&
        !engine->post_shake_pending && !possible_new_pose && !rebase_hold && !rebased_this_frame;

    for (y = 0; y < gray->height; ++y) {
        const uint8_t *cur = gray->data + (size_t)y * gray->stride;
        const uint8_t *prev = engine->previous_gray.data + (size_t)y * engine->previous_gray.stride;
        uint16_t *bg = engine->background_q8.data + (size_t)y * engine->background_q8.stride;
        for (x = 0; x < gray->width; ++x) {
            int old_q8 = (int)bg[x];
            const int background_value = (old_q8 + DF_BACKGROUND_SCALE / 2) >> DF_BACKGROUND_FRACTION_BITS;
            int corrected_bg_current = (int)cur[x] - bg_applied_shift;
            int corrected_frame_current = (int)cur[x] - frame_applied_shift;
            int bg_diff, frame_diff;
            bool pixel_stable;
            if (corrected_bg_current < 0) corrected_bg_current = 0; else if (corrected_bg_current > 255) corrected_bg_current = 255;
            bg_diff = corrected_bg_current - background_value; if (bg_diff < 0) bg_diff = -bg_diff;
            if (corrected_frame_current < 0) corrected_frame_current = 0; else if (corrected_frame_current > 255) corrected_frame_current = 255;
            frame_diff = corrected_frame_current - (int)prev[x]; if (frame_diff < 0) frame_diff = -frame_diff;
            pixel_stable = bg_diff <= DF_SELECTIVE_BG_LEARN_DIFF_THRESHOLD &&
                           frame_diff <= DF_SELECTIVE_FRAME_STABLE_THRESHOLD;
            if (allow_selective_background_update && pixel_stable) {
                const int target_q8 = (int)cur[x] << DF_BACKGROUND_FRACTION_BITS;
                const int delta_q8 = target_q8 - old_q8;
                int updated_q8 = old_q8 + delta_q8 / DF_BACKGROUND_UPDATE_DIVISOR;
                const int max_q8 = 255 << DF_BACKGROUND_FRACTION_BITS;
                if (updated_q8 < 0) updated_q8 = 0;
                if (updated_q8 > max_q8) updated_q8 = max_q8;
                bg[x] = (uint16_t)updated_q8;
                ++learned_pixels;
            }
            background_sum_q8 += (uint64_t)bg[x];
        }
    }

    if (total_pixels > 0)
        engine->background_mean = (double)background_sum_q8 / ((double)total_pixels * (double)DF_BACKGROUND_SCALE);
    learned_ratio = total_pixels ? (double)learned_pixels / (double)total_pixels : 0.0;

    if (engine->frame_count % DF_MOTION_LOG_INTERVAL == 1) {
        // printf("[MOTION] frame=%llu bgG=%.3f bgGu=%.3f bgSt=%.3f frmG=%.3f frmGu=%.3f frmSt=%.3f "
        //        "meanCur=%.2f meanBg=%.2f bgShift=%+.2f/%+d frmShift=%+.2f/%+d bgDiff=%.2f frmDiff=%.2f "
        //        "cam=%s fastShake=%d staticShake=%d outsideFrm=%.3f outsideBg=%.3f "
        //        "zFrm=%.3f/%.3f/%.3f/%.3f zAct=%u/%u camHold=%llu postShake=%d "
        //        "settle=%llu/%llu hold=%llu learn=%.3f inst=%d event=%s confirm=%llu miss=%llu recover=%llu active=%d\n",
        //        (unsigned long long)engine->frame_count,
        //        global_bg_ratio, gully_bg_ratio, static_bg_ratio,
        //        global_frame_ratio, gully_frame_ratio, static_frame_ratio,
        //        current_mean, engine->background_mean,
        //        bg_shift, bg_applied_shift, frame_shift, frame_applied_shift,
        //        bg_mean_diff, frame_mean_diff, camera_state,
        //        fast_camera_shake ? 1 : 0,
        //        static_camera_shake ? 1 : 0,
        //        outside_gully_frame_ratio, outside_gully_bg_ratio,
        //        outside_zone_frame_ratio[0], outside_zone_frame_ratio[1],
        //        outside_zone_frame_ratio[2], outside_zone_frame_ratio[3],
        //        outside_zone_active_count, (unsigned)DF_OUTSIDE_ZONE_COUNT,
        //        (unsigned long long)engine->camera_disturbance_hold_remaining,
        //        engine->post_shake_pending ? 1 : 0,
        //        (unsigned long long)engine->settled_new_pose_frames,
        //        (unsigned long long)DF_REBASE_STABLE_FRAMES,
        //        (unsigned long long)engine->rebase_hold_remaining,
        //        learned_ratio, instant_event_evidence ? 1 : 0,
        //        temporal_event_state_name(engine->event_tracker.state),
        //        (unsigned long long)engine->event_tracker.confirm_frames,
        //        (unsigned long long)engine->event_tracker.miss_frames,
        //        (unsigned long long)engine->event_tracker.recovery_frames,
        //        temporal_candidate ? 1 : 0);
        fflush(stdout);

        // printf("[BLOB] frame=%llu analyzed=%d fgPix=%llu fgRatio=%.3f movingPix=%llu movingRatio=%.3f "
        //        "raw=%d valid=%d totalArea=%llu totalRatio=%.3f largestArea=%d largestRatio=%.3f "
        //        "largestCx=%.1f largestCy=%.1f largestBox=%d,%d,%d,%d allCx=%.1f allCy=%.1f\n",
        //        (unsigned long long)engine->frame_count, blob_analyzed ? 1 : 0,
        //        (unsigned long long)gully_bg_changed, gully_bg_ratio,
        //        (unsigned long long)moving_overlap_pixels, moving_overlap_ratio,
        //        blob_stats.raw_component_count, blob_stats.valid_blob_count,
        //        (unsigned long long)blob_stats.total_valid_area, blob_total_area_ratio,
        //        blob_stats.largest.area, largest_blob_ratio,
        //        blob_stats.largest.cx, blob_stats.largest.cy,
        //        blob_stats.largest.min_x, blob_stats.largest.min_y,
        //        blob_stats.largest.max_x, blob_stats.largest.max_y,
        //        blob_stats.combined_cx, blob_stats.combined_cy);
        fflush(stdout);

        // printf("[TRACK] frame=%llu valid=%d id=%d age=%llu missed=%llu area=%d cx=%.1f cy=%.1f "
        //        "dx=%+.1f dy=%+.1f vx=%+.1f vy=%+.1f speed=%.1f speedClass=%s dir=%s dirStreak=%llu\n",
        //        (unsigned long long)engine->frame_count,
        //        engine->dominant_tracker.valid ? 1 : 0,
        //        engine->dominant_tracker.id,
        //        (unsigned long long)engine->dominant_tracker.age_frames,
        //        (unsigned long long)engine->dominant_tracker.missed_frames,
        //        engine->dominant_tracker.valid ? engine->dominant_tracker.blob.area : 0,
        //        engine->dominant_tracker.valid ? engine->dominant_tracker.blob.cx : 0.0,
        //        engine->dominant_tracker.valid ? engine->dominant_tracker.blob.cy : 0.0,
        //        engine->dominant_tracker.dx, engine->dominant_tracker.dy,
        //        engine->dominant_tracker.vx, engine->dominant_tracker.vy,
        //        engine->dominant_tracker.speed_px_s,
        //        coarse_speed_name(engine->dominant_tracker.speed_px_s),
        //        coarse_direction_name(engine->dominant_tracker.direction),
        //        (unsigned long long)engine->dominant_tracker.direction_streak);
        fflush(stdout);
    }

    if (gray_copy_to_u8(gray, &engine->previous_gray) != 0) return -1;
    engine->previous_mean = current_mean;
    if (engine->rebase_hold_remaining > 0 && !rebased_this_frame) --engine->rebase_hold_remaining;
    return 0;
}

void motion_engine_drain_queues(MotionEngine *engine) {
    if (engine) event_manager_drain_to_log(&engine->event_manager);
}

void motion_engine_finish_process(MotionEngine *engine) {
    if (!engine) return;
    /* Producer only. The caller dispatches/logs after this, so PROCESS_STOPPED
     * can use the same LoRa telemetry path as normal EVENT_END messages. */
    event_manager_finish_process(&engine->event_manager);
}
