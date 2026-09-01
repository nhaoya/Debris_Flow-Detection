#ifndef JPEG_GRAY_H
#define JPEG_GRAY_H

#include "image_utils.h"

#include <stddef.h>
#include <stdint.h>

/*
 * Small baseline grayscale JPEG encoder for deployment/event thumbnails.
 * Input is the Y plane already produced by the VI NV12 channel.
 */
int jpeg_gray_encode_scaled(const GrayImage *src,
                            int out_width,
                            int out_height,
                            int quality,
                            uint8_t *out,
                            size_t out_capacity,
                            size_t *out_size);

/* Read a P5 PGM file, scale and encode it as baseline grayscale JPEG. */
int jpeg_gray_encode_pgm_scaled(const char *pgm_path,
                                int out_width,
                                int out_height,
                                int quality,
                                uint8_t *out,
                                size_t out_capacity,
                                size_t *out_size);

#endif
