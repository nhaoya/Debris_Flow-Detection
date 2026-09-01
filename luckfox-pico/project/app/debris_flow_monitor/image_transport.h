#ifndef IMAGE_TRANSPORT_H
#define IMAGE_TRANSPORT_H

#include "image_utils.h"
#include "telemetry_protocol.h"

#include <stddef.h>
#include <stdint.h>

int image_transport_encode_gray(const GrayImage *gray,
                                DfImagePurpose purpose,
                                uint8_t *jpeg,
                                size_t jpeg_capacity,
                                size_t *jpeg_size,
                                uint8_t *quality_used);

int image_transport_encode_pgm(const char *pgm_path,
                               DfImagePurpose purpose,
                               uint8_t *jpeg,
                               size_t jpeg_capacity,
                               size_t *jpeg_size,
                               uint8_t *quality_used);

const char *df_image_purpose_name(DfImagePurpose purpose);

#endif
