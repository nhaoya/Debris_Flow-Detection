#include "camera_pipeline.h"
#include "debris_config.h"
#include "isp_min.h"
#include "uart2_link.h"

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
    printf("Usage: %s [--uart-device /dev/ttyS2] [--uart-baud 115200] "
           "[--device-id 1] [--uart-off]\n", prog);
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
    Uart2Link uart2;
    Uart2Link *uart2_ptr = NULL;
    const char *uart_device = DF_UART2_DEFAULT_DEVICE;
    int uart_baud = DF_UART2_DEFAULT_BAUD;
    int device_id_int = (int)DF_UART2_DEFAULT_DEVICE_ID;
    bool uart_enabled = true;
    int uart_inited = 0;
    int isp_started = 0;
    int mpi_started = 0;
    int ret = 1;
    int i;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--uart-device") == 0 && i + 1 < argc) {
            uart_device = argv[++i];
        } else if (strcmp(argv[i], "--uart-baud") == 0 && i + 1 < argc) {
            if (parse_int(argv[++i], 1200, 1000000, &uart_baud) != 0) {
                fprintf(stderr, "[MAIN] invalid --uart-baud\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--device-id") == 0 && i + 1 < argc) {
            if (parse_int(argv[++i], 1, 65535, &device_id_int) != 0) {
                fprintf(stderr, "[MAIN] invalid --device-id\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--uart-off") == 0) {
            uart_enabled = false;
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

    printf("=== debris_flow_monitor Pure-C V1.4.2 UART2 ===\n");

    if (uart_enabled) {
        if (uart2_link_init(&uart2, uart_device, uart_baud, (uint16_t)device_id_int) != 0) {
            fprintf(stderr, "[MAIN] UART2 init failed; continue camera in log-only mode\n");
        } else {
            uart_inited = 1;
            if (uart2_link_start(&uart2) != 0) {
                fprintf(stderr, "[MAIN] UART2 worker start failed; continue camera in log-only mode\n");
            } else {
                uart2_ptr = &uart2;
            }
        }
    } else {
        printf("[MAIN] UART2 disabled by --uart-off\n");
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

    ret = camera_pipeline_capture_loop(&camera, &g_running, uart2_ptr);
    camera_pipeline_deinit(&camera);

cleanup:
    if (uart2_ptr) {
        if (!uart2_link_wait_queue_empty(uart2_ptr, 1000U))
            fprintf(stderr, "[MAIN] UART2 queue not empty after 1s shutdown grace\n");
    }
    if (uart_inited) uart2_link_deinit(&uart2);
    if (mpi_started) RK_MPI_SYS_Exit();
    if (isp_started) isp_min_stop();
    return ret == 0 ? 0 : 1;
}
