#include "image_utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

void u8_image_reset(U8Image *img) {
    if (img) memset(img, 0, sizeof(*img));
}

void u16_image_reset(U16Image *img) {
    if (img) memset(img, 0, sizeof(*img));
}

int u8_image_alloc(U8Image *img, int width, int height) {
    size_t bytes;
    uint8_t *p;
    if (!img || width <= 0 || height <= 0) return -1;
    if (img->data && img->width == width && img->height == height && img->stride == width) return 0;
    free(img->data);
    memset(img, 0, sizeof(*img));
    bytes = (size_t)width * (size_t)height;
    p = (uint8_t *)calloc(bytes, 1);
    if (!p) return -1;
    img->data = p;
    img->width = width;
    img->height = height;
    img->stride = width;
    return 0;
}

int u16_image_alloc(U16Image *img, int width, int height) {
    size_t count;
    uint16_t *p;
    if (!img || width <= 0 || height <= 0) return -1;
    if (img->data && img->width == width && img->height == height && img->stride == width) return 0;
    free(img->data);
    memset(img, 0, sizeof(*img));
    count = (size_t)width * (size_t)height;
    p = (uint16_t *)calloc(count, sizeof(uint16_t));
    if (!p) return -1;
    img->data = p;
    img->width = width;
    img->height = height;
    img->stride = width;
    return 0;
}

void u8_image_free(U8Image *img) {
    if (!img) return;
    free(img->data);
    memset(img, 0, sizeof(*img));
}

void u16_image_free(U16Image *img) {
    if (!img) return;
    free(img->data);
    memset(img, 0, sizeof(*img));
}

void u8_image_zero(U8Image *img) {
    if (!img || !img->data || img->width <= 0 || img->height <= 0) return;
    memset(img->data, 0, (size_t)img->stride * (size_t)img->height);
}

int gray_copy_to_u8(const GrayImage *src, U8Image *dst) {
    int y;
    if (!src || !src->data || !dst) return -1;
    if (u8_image_alloc(dst, src->width, src->height) != 0) return -1;
    for (y = 0; y < src->height; ++y) {
        memcpy(dst->data + (size_t)y * dst->stride,
               src->data + (size_t)y * src->stride,
               (size_t)src->width);
    }
    return 0;
}

int gray_resize_nearest_to_u8(const GrayImage *src, U8Image *dst, int out_width, int out_height) {
    int x, y;
    if (!src || !src->data || !dst || out_width <= 0 || out_height <= 0) return -1;
    if (u8_image_alloc(dst, out_width, out_height) != 0) return -1;
    for (y = 0; y < out_height; ++y) {
        int sy = (int)(((int64_t)y * src->height) / out_height);
        uint8_t *out_row = dst->data + (size_t)y * dst->stride;
        const uint8_t *in_row;
        if (sy >= src->height) sy = src->height - 1;
        in_row = src->data + (size_t)sy * src->stride;
        for (x = 0; x < out_width; ++x) {
            int sx = (int)(((int64_t)x * src->width) / out_width);
            if (sx >= src->width) sx = src->width - 1;
            out_row[x] = in_row[sx];
        }
    }
    return 0;
}

double gray_mean(const GrayImage *gray) {
    uint64_t sum = 0;
    uint64_t count;
    int x, y;
    if (!gray || !gray->data || gray->width <= 0 || gray->height <= 0) return 0.0;
    for (y = 0; y < gray->height; ++y) {
        const uint8_t *row = gray->data + (size_t)y * gray->stride;
        for (x = 0; x < gray->width; ++x) sum += row[x];
    }
    count = (uint64_t)gray->width * (uint64_t)gray->height;
    return count ? (double)sum / (double)count : 0.0;
}

static bool point_in_polygon(double x, double y, const RoiPoint *polygon, size_t n) {
    bool inside = false;
    size_t i, j;
    if (!polygon || n < 3) return false;
    j = n - 1;
    for (i = 0; i < n; ++i) {
        const double xi = (double)polygon[i].x;
        const double yi = (double)polygon[i].y;
        const double xj = (double)polygon[j].x;
        const double yj = (double)polygon[j].y;
        if ((yi > y) != (yj > y)) {
            const double x_intersect = (xj - xi) * (y - yi) / (yj - yi) + xi;
            if (x < x_intersect) inside = !inside;
        }
        j = i;
    }
    return inside;
}

uint64_t build_polygon_mask(
    U8Image *mask, int width, int height,
    const RoiPoint *base_polygon, size_t point_count,
    int base_width, int base_height) {
    RoiPoint *scaled;
    uint64_t count = 0;
    size_t i;
    int x, y;
    if (!mask || !base_polygon || point_count < 3 || base_width <= 0 || base_height <= 0) return 0;
    if (u8_image_alloc(mask, width, height) != 0) return 0;
    u8_image_zero(mask);
    scaled = (RoiPoint *)malloc(point_count * sizeof(RoiPoint));
    if (!scaled) return 0;
    for (i = 0; i < point_count; ++i) {
        scaled[i].x = (base_polygon[i].x * width + base_width / 2) / base_width;
        scaled[i].y = (base_polygon[i].y * height + base_height / 2) / base_height;
    }
    for (y = 0; y < height; ++y) {
        uint8_t *row = mask->data + (size_t)y * mask->stride;
        for (x = 0; x < width; ++x) {
            if (point_in_polygon((double)x + 0.5, (double)y + 0.5, scaled, point_count)) {
                row[x] = 1;
                ++count;
            }
        }
    }
    free(scaled);
    return count;
}

bool write_mask_pgm(const char *path, const U8Image *mask) {
    FILE *fp;
    int x, y;
    if (!path || !mask || !mask->data) return false;
    fp = fopen(path, "wb");
    if (!fp) return false;
    fprintf(fp, "P5\n%d %d\n255\n", mask->width, mask->height);
    for (y = 0; y < mask->height; ++y) {
        const uint8_t *row = mask->data + (size_t)y * mask->stride;
        for (x = 0; x < mask->width; ++x) {
            const uint8_t value = row[x] ? 255 : 0;
            fwrite(&value, 1, 1, fp);
        }
    }
    fclose(fp);
    return true;
}

bool write_gray_snapshot_pgm(const char *path, const GrayImage *gray) {
    FILE *fp;
    int y;
    bool ok = true;
    if (!path || !gray || !gray->data) return false;
    fp = fopen(path, "wb");
    if (!fp) return false;
    fprintf(fp, "P5\n%d %d\n255\n", gray->width, gray->height);
    for (y = 0; y < gray->height; ++y) {
        const uint8_t *row = gray->data + (size_t)y * gray->stride;
        if (fwrite(row, 1, (size_t)gray->width, fp) != (size_t)gray->width) {
            ok = false;
            break;
        }
    }
    fclose(fp);
    return ok;
}

static bool is_mask_boundary(const U8Image *mask, int x, int y) {
    static const int dx[4] = {-1, 1, 0, 0};
    static const int dy[4] = {0, 0, -1, 1};
    int i;
    if (!mask || !mask->data || !mask->data[(size_t)y * mask->stride + x]) return false;
    for (i = 0; i < 4; ++i) {
        const int nx = x + dx[i];
        const int ny = y + dy[i];
        if (nx < 0 || ny < 0 || nx >= mask->width || ny >= mask->height) return true;
        if (!mask->data[(size_t)ny * mask->stride + nx]) return true;
    }
    return false;
}

bool write_roi_debug_ppm(const char *path, const GrayImage *gray,
                         const U8Image *gully_mask, const U8Image *static_mask) {
    FILE *fp;
    int x, y;
    if (!path || !gray || !gray->data || !gully_mask || !static_mask) return false;
    fp = fopen(path, "wb");
    if (!fp) return false;
    fprintf(fp, "P6\n%d %d\n255\n", gray->width, gray->height);
    for (y = 0; y < gray->height; ++y) {
        const uint8_t *src = gray->data + (size_t)y * gray->stride;
        for (x = 0; x < gray->width; ++x) {
            uint8_t rgb[3] = {src[x], src[x], src[x]};
            const bool gb = is_mask_boundary(gully_mask, x, y);
            const bool sb = is_mask_boundary(static_mask, x, y);
            if (gb && sb) { rgb[0] = 255; rgb[1] = 255; rgb[2] = 0; }
            else if (gb) { rgb[0] = 255; rgb[1] = 0; rgb[2] = 0; }
            else if (sb) { rgb[0] = 0; rgb[1] = 255; rgb[2] = 0; }
            fwrite(rgb, 1, 3, fp);
        }
    }
    fclose(fp);
    return true;
}

double monotonic_seconds(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0.0;
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

uint64_t wall_clock_epoch_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) return 0;
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}
