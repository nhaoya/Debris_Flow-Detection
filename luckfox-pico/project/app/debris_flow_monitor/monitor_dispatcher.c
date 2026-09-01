#include "monitor_dispatcher.h"
#include "debris_config.h"
#include "image_transport.h"

#include <stdio.h>
#include <stdlib.h>

void monitor_dispatch_event_manager(EventManager *manager, LoraUartLink *lora) {
    MonitorMessage message;
    SnapshotPackage snapshot;
    if (!manager) return;

    while (event_manager_pop_telemetry(manager, &message)) {
        /* Keep the original V1.4.2 log line unchanged. */
        monitor_message_log(&message, "DATA");
        if (lora) lora_uart_enqueue(lora, &message);
    }

    while (event_manager_pop_snapshot(manager, &snapshot)) {
        uint8_t *jpeg = NULL;
        size_t jpeg_size = 0U;
        uint8_t quality = 0U;
        uint32_t image_id = 0U;
        int enc_ret;

        snapshot_package_log(&snapshot);
        if (!lora) continue;

        jpeg = (uint8_t *)malloc(DF_IMAGE_MAX_JPEG_BYTES);
        if (!jpeg) {
            fprintf(stderr, "[IMAGE] malloc failed for event=%llu snapshot=%s\n",
                    (unsigned long long)snapshot.event_id, snapshot.path);
            continue;
        }
        enc_ret = image_transport_encode_pgm(snapshot.path,
                                             DF_IMAGE_PURPOSE_EVENT,
                                             jpeg,
                                             DF_IMAGE_MAX_JPEG_BYTES,
                                             &jpeg_size,
                                             &quality);
        if (enc_ret != 0) {
            fprintf(stderr, "[IMAGE] JPEG encode failed event=%llu path=%s ret=%d\n",
                    (unsigned long long)snapshot.event_id, snapshot.path, enc_ret);
            free(jpeg);
            continue;
        }
        printf("[IMAGE] event snapshot compressed event=%llu pgm=%s jpeg=%zu bytes %dx%d q=%u\n",
               (unsigned long long)snapshot.event_id, snapshot.path, jpeg_size,
               DF_IMAGE_JPEG_WIDTH, DF_IMAGE_JPEG_HEIGHT, (unsigned)quality);
        fflush(stdout);
        if (!lora_uart_enqueue_image(lora,
                                     snapshot.event_id,
                                     snapshot.capture_epoch_ms,
                                     DF_IMAGE_PURPOSE_EVENT,
                                     (uint16_t)DF_IMAGE_JPEG_WIDTH,
                                     (uint16_t)DF_IMAGE_JPEG_HEIGHT,
                                     quality,
                                     jpeg,
                                     jpeg_size,
                                     &image_id)) {
            fprintf(stderr, "[IMAGE] LoRa image enqueue failed event=%llu bytes=%zu\n",
                    (unsigned long long)snapshot.event_id, jpeg_size);
        } else {
            printf("[IMAGE] event snapshot queued image=%u event=%llu\n",
                   (unsigned)image_id, (unsigned long long)snapshot.event_id);
            fflush(stdout);
        }
        free(jpeg);
    }
}
