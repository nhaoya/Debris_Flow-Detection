#include "camera_pipeline.h"
#include "debris_config.h"
#include "isp_min.h"
#include "lora_uart.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rk_mpi_sys.h>

static volatile sig_atomic_t g_running = 1;

static void on_signal(int signo) {
    (void)signo;
    g_running = 0;
}

static void print_usage(const char *prog) {
    printf("Usage: %s [--lora-device /dev/ttyS3] [--lora-baud 9600] "
           "[--device-id 1] [--heartbeat-seconds 600] [--lora-off] "
           "[--deploy-preview-seconds 600] [--deploy-preview-interval 5] "
           "[--chain-first-id N --chain-last-id M]\n", prog);
    printf("       Backward aliases: --uart-device, --uart-baud, --uart-off\n");
    printf("       V1.4.4 deployment preview: first camera frame immediately, then every 5s for 10min.\n");
}

static int parse_int(const char *s, int min_value, int max_value, int *out) {
    char *end = NULL;
    long value;
    if (!s || !out) return -1;
    errno = 0;
    value = strtol(s, &end, 10);
    if (errno != 0 || !end || *end != '\0' || value < min_value || value > max_value) return -1;
    *out = (int)value;
    return 0;
}

int main(int argc, char **argv) {
    CameraPipeline camera;
    LoraUartLink lora;
    LoraUartLink *lora_ptr = NULL;
    const char *lora_device = DF_LORA_DEFAULT_DEVICE;
    int lora_baud = DF_LORA_DEFAULT_BAUD;
    int device_id_int = (int)DF_LORA_DEFAULT_DEVICE_ID;
    int heartbeat_seconds = (int)DF_LORA_HEARTBEAT_INTERVAL_SECONDS;
    int deploy_preview_seconds = (int)DF_BOOT_PREVIEW_DURATION_SECONDS;
    int deploy_preview_interval = (int)DF_BOOT_PREVIEW_INTERVAL_SECONDS;
    int chain_first_id = 0;
    int chain_last_id = 0;
    bool lora_enabled = true;
    int lora_inited = 0;
    int isp_started = 0;
    int mpi_started = 0;
    int camera_ready = 0;
    int ret = 1;
    int i;

    for (i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "--lora-device") == 0 || strcmp(argv[i], "--uart-device") == 0) && i + 1 < argc) {
            lora_device = argv[++i];
        } else if ((strcmp(argv[i], "--lora-baud") == 0 || strcmp(argv[i], "--uart-baud") == 0) && i + 1 < argc) {
            if (parse_int(argv[++i], 1200, 1000000, &lora_baud) != 0) {
                fprintf(stderr, "[MAIN] invalid LoRa UART baud\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--device-id") == 0 && i + 1 < argc) {
            if (parse_int(argv[++i], 1, 65535, &device_id_int) != 0) {
                fprintf(stderr, "[MAIN] invalid --device-id\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--heartbeat-seconds") == 0 && i + 1 < argc) {
            if (parse_int(argv[++i], 0, 86400, &heartbeat_seconds) != 0) {
                fprintf(stderr, "[MAIN] invalid --heartbeat-seconds\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--deploy-preview-seconds") == 0 && i + 1 < argc) {
            if (parse_int(argv[++i], 0, 3600, &deploy_preview_seconds) != 0) {
                fprintf(stderr, "[MAIN] invalid --deploy-preview-seconds\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--deploy-preview-interval") == 0 && i + 1 < argc) {
            if (parse_int(argv[++i], 1, 300, &deploy_preview_interval) != 0) {
                fprintf(stderr, "[MAIN] invalid --deploy-preview-interval\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--chain-first-id") == 0 && i + 1 < argc) {
            if (parse_int(argv[++i], 1, 65535, &chain_first_id) != 0) {
                fprintf(stderr, "[MAIN] invalid --chain-first-id\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--chain-last-id") == 0 && i + 1 < argc) {
            if (parse_int(argv[++i], 1, 65535, &chain_last_id) != 0) {
                fprintf(stderr, "[MAIN] invalid --chain-last-id\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--lora-off") == 0 || strcmp(argv[i], "--uart-off") == 0) {
            lora_enabled = false;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "[MAIN] unknown argument: %s\n", argv[i]);
            print_usage(argv[0]);
            return 2;
        }
    }

    if ((chain_first_id == 0) != (chain_last_id == 0)) {
        fprintf(stderr, "[MAIN] --chain-first-id and --chain-last-id must be used together\n");
        return 2;
    }
    if (chain_first_id != 0) {
        if (chain_first_id > chain_last_id || device_id_int < chain_first_id || device_id_int > chain_last_id) {
            fprintf(stderr, "[MAIN] invalid chain range: first=%d last=%d local=%d\n",
                    chain_first_id, chain_last_id, device_id_int);
            return 2;
        }
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    printf("=== debris_flow_monitor Pure-C V1.4.4 Communication/Image Reliability Chain Relay fix3 UART-RAW ===\n");
    printf("[MAIN] startup order: ISP -> MPI -> camera VI -> LoRa -> capture\n");
    printf("[MAIN] image: JPEG_GRAY %dx%d, event>=30s, deploy-preview=%ds for first %ds\n",
           DF_IMAGE_JPEG_WIDTH, DF_IMAGE_JPEG_HEIGHT,
           deploy_preview_interval, deploy_preview_seconds);
    printf("[MAIN] LoRa reliability: 40B chunks + AUX flow + hop-by-hop IMAGE_NACK/ACK + EVENT_START/END ACK\n");
    if (chain_first_id != 0)
        printf("[MAIN] chain relay enabled: IDs %d..%d, local=%d, high->low follows increasing device-id\n",
               chain_first_id, chain_last_id, device_id_int);
    else
        printf("[MAIN] chain relay disabled; local-origin traffic only\n");

    /* Camera is intentionally opened before the LoRa worker for deployment positioning. */
    if (isp_min_start("/oem/usr/share/iqfiles") != 0) {
        fprintf(stderr, "[MAIN] ISP init failed\n");
        goto cleanup;
    }
    isp_started = 1;

    if (RK_MPI_SYS_Init() != RK_SUCCESS) {
        fprintf(stderr, "[MAIN] MPI init failed\n");
        goto cleanup;
    }
    mpi_started = 1;
    printf("[MAIN] MPI initialized\n");

    camera_pipeline_reset(&camera);
    if (camera_pipeline_init(&camera) != RK_SUCCESS) {
        fprintf(stderr, "[MAIN] camera init failed\n");
        camera_pipeline_deinit(&camera);
        goto cleanup;
    }
    camera_ready = 1;
    printf("[MAIN] camera opened; starting LoRa transport next\n");

    if (lora_enabled) {
        if (lora_uart_init(&lora, lora_device, lora_baud, (uint16_t)device_id_int,
                           DF_LORA_GPIO_M0, DF_LORA_GPIO_M1, DF_LORA_GPIO_AUX,
                           (unsigned)heartbeat_seconds,
                           (uint16_t)chain_first_id, (uint16_t)chain_last_id) != 0) {
            fprintf(stderr, "[MAIN] LoRa UART init failed; continue camera in log-only mode\n");
        } else {
            lora_inited = 1;
            if (lora_uart_start(&lora) != 0) {
                fprintf(stderr, "[MAIN] LoRa worker start failed; continue camera in log-only mode\n");
            } else {
                lora_ptr = &lora;
            }
        }
    } else {
        printf("[MAIN] LoRa disabled by --lora-off\n");
    }

    ret = camera_pipeline_capture_loop(&camera, &g_running, lora_ptr,
                                       (unsigned)deploy_preview_seconds,
                                       (unsigned)deploy_preview_interval);

cleanup:
    if (camera_ready) {
        camera_pipeline_deinit(&camera);
        camera_ready = 0;
    }
    if (lora_ptr) {
        if (!lora_uart_wait_queue_empty(lora_ptr, 1500U))
            fprintf(stderr, "[MAIN] LoRa queue not empty after shutdown grace\n");
    }
    if (lora_inited) lora_uart_deinit(&lora);
    if (mpi_started) RK_MPI_SYS_Exit();
    if (isp_started) isp_min_stop();
    return ret == 0 ? 0 : 1;
}
