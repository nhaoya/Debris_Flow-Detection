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
           "[--device-id 1] [--heartbeat-seconds 600] [--lora-off]\n", prog);
    printf("       Backward aliases: --uart-device, --uart-baud, --uart-off\n");
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
    bool lora_enabled = true;
    int lora_inited = 0;
    int isp_started = 0;
    int mpi_started = 0;
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

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    printf("=== debris_flow_monitor Pure-C V1.4.3 LoRa-UART3 HB10M ===\n");

    if (lora_enabled) {
        if (lora_uart_init(&lora, lora_device, lora_baud, (uint16_t)device_id_int,
                           DF_LORA_GPIO_M0, DF_LORA_GPIO_M1, DF_LORA_GPIO_AUX,
                           (unsigned)heartbeat_seconds) != 0) {
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

    ret = camera_pipeline_capture_loop(&camera, &g_running, lora_ptr);
    camera_pipeline_deinit(&camera);

cleanup:
    if (lora_ptr) {
        if (!lora_uart_wait_queue_empty(lora_ptr, 1500U))
            fprintf(stderr, "[MAIN] LoRa queue not empty after shutdown grace\n");
    }
    if (lora_inited) lora_uart_deinit(&lora);
    if (mpi_started) RK_MPI_SYS_Exit();
    if (isp_started) isp_min_stop();
    return ret == 0 ? 0 : 1;
}
