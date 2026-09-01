#include "image_transport.h"
#include "debris_config.h"
#include "jpeg_gray.h"

#include <stdio.h>

const char *df_image_purpose_name(DfImagePurpose purpose) {
    switch (purpose) {
    case DF_IMAGE_PURPOSE_EVENT: return "EVENT";
    case DF_IMAGE_PURPOSE_DEPLOY_PREVIEW: return "DEPLOY_PREVIEW";
    default: return "UNKNOWN";
    }
}

static int quality_plan(DfImagePurpose purpose,
                        int qualities[4],
                        size_t *count,
                        size_t *target_bytes) {
    if (!qualities || !count || !target_bytes) return -1;
    if (purpose == DF_IMAGE_PURPOSE_DEPLOY_PREVIEW) {
        qualities[0] = DF_IMAGE_PREVIEW_JPEG_QUALITY;
        qualities[1] = DF_IMAGE_PREVIEW_FALLBACK_QUALITY_1;
        qualities[2] = DF_IMAGE_PREVIEW_FALLBACK_QUALITY_2;
        *count = 3U;
        *target_bytes = DF_IMAGE_PREVIEW_TARGET_BYTES;
        return 0;
    }
    if (purpose == DF_IMAGE_PURPOSE_EVENT) {
        qualities[0] = DF_IMAGE_EVENT_JPEG_QUALITY;
        qualities[1] = DF_IMAGE_EVENT_FALLBACK_QUALITY_1;
        qualities[2] = DF_IMAGE_EVENT_FALLBACK_QUALITY_2;
        qualities[3] = DF_IMAGE_EVENT_FALLBACK_QUALITY_3;
        *count = 4U;
        *target_bytes = DF_IMAGE_EVENT_TARGET_BYTES;
        return 0;
    }
    return -1;
}

static int encode_gray_adaptive(const GrayImage *gray,
                                DfImagePurpose purpose,
                                uint8_t *jpeg,
                                size_t jpeg_capacity,
                                size_t *jpeg_size,
                                uint8_t *quality_used) {
    int qualities[4];
    size_t count = 0U, target = 0U, i;
    size_t size = 0U;
    int ret = -1;
    if (quality_plan(purpose, qualities, &count, &target) != 0) return -1;
    for (i = 0U; i < count; ++i) {
        ret = jpeg_gray_encode_scaled(gray,
                                      DF_IMAGE_JPEG_WIDTH,
                                      DF_IMAGE_JPEG_HEIGHT,
                                      qualities[i],
                                      jpeg,
                                      jpeg_capacity,
                                      &size);
        if (ret != 0) continue;
        if (size <= target || i + 1U == count) {
            *jpeg_size = size;
            *quality_used = (uint8_t)qualities[i];
            return 0;
        }
    }
    return ret;
}

int image_transport_encode_gray(const GrayImage *gray,
                                DfImagePurpose purpose,
                                uint8_t *jpeg,
                                size_t jpeg_capacity,
                                size_t *jpeg_size,
                                uint8_t *quality_used) {
    if (!gray || !jpeg || !jpeg_size || !quality_used) return -1;
    return encode_gray_adaptive(gray, purpose, jpeg, jpeg_capacity, jpeg_size, quality_used);
}

int image_transport_encode_pgm(const char *pgm_path,
                               DfImagePurpose purpose,
                               uint8_t *jpeg,
                               size_t jpeg_capacity,
                               size_t *jpeg_size,
                               uint8_t *quality_used) {
    int qualities[4];
    size_t count = 0U, target = 0U, i;
    size_t size = 0U;
    int ret = -1;
    if (!pgm_path || !jpeg || !jpeg_size || !quality_used) return -1;
    if (quality_plan(purpose, qualities, &count, &target) != 0) return -1;
    for (i = 0U; i < count; ++i) {
        ret = jpeg_gray_encode_pgm_scaled(pgm_path,
                                          DF_IMAGE_JPEG_WIDTH,
                                          DF_IMAGE_JPEG_HEIGHT,
                                          qualities[i],
                                          jpeg,
                                          jpeg_capacity,
                                          &size);
        if (ret != 0) continue;
        if (size <= target || i + 1U == count) {
            *jpeg_size = size;
            *quality_used = (uint8_t)qualities[i];
            return 0;
        }
    }
    return ret;
}
