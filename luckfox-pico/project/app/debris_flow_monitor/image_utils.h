#ifndef IMAGE_UTILS_H
#define IMAGE_UTILS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t *data;
    int width;
    int height;
    int stride;
} GrayImage;

typedef struct {
    uint8_t *data;
    int width;
    int height;
    int stride;
} U8Image;

typedef struct {
    uint16_t *data;
    int width;
    int height;
    int stride;
} U16Image;

typedef struct {
    int x;
    int y;
} RoiPoint;

void u8_image_reset(U8Image *img);
void u16_image_reset(U16Image *img);
int u8_image_alloc(U8Image *img, int width, int height);
int u16_image_alloc(U16Image *img, int width, int height);
void u8_image_free(U8Image *img);
void u16_image_free(U16Image *img);
void u8_image_zero(U8Image *img);
int gray_copy_to_u8(const GrayImage *src, U8Image *dst);
int gray_resize_nearest_to_u8(const GrayImage *src, U8Image *dst, int out_width, int out_height);
double gray_mean(const GrayImage *gray);

uint64_t build_polygon_mask(
    U8Image *mask,
    int width,
    int height,
    const RoiPoint *base_polygon,
    size_t point_count,
    int base_width,
    int base_height);

bool write_mask_pgm(const char *path, const U8Image *mask);
bool write_gray_snapshot_pgm(const char *path, const GrayImage *gray);
bool write_roi_debug_ppm(
    const char *path,
    const GrayImage *gray,
    const U8Image *gully_mask,
    const U8Image *static_mask);

double monotonic_seconds(void);
uint64_t wall_clock_epoch_ms(void);

#endif
