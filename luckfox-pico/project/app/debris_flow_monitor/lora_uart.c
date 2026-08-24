#include "lora_uart.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define DF_LORA_AUX_POLL_MS 10U
#define DF_LORA_AUX_PRE_TX_TIMEOUT_MS 500U
#define DF_LORA_AUX_POST_TX_TIMEOUT_MS 1000U
#define DF_LORA_RECONNECT_MS 1000U

static speed_t baud_to_termios(int baud_rate) {
    switch (baud_rate) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
    default: return (speed_t)0;
    }
}

static void sleep_ms(unsigned ms) {
    struct timespec ts;
    ts.tv_sec = (time_t)(ms / 1000U);
    ts.tv_nsec = (long)(ms % 1000U) * 1000000L;
    while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {
    }
}

static uint64_t monotonic_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0U;
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

/* ----- sysfs GPIO helpers: copied in behavior from the validated standalone Lora.c ----- */
static int gpio_export_one(int pin) {
    char buf[32];
    int fd;
    int len;
    ssize_t written;

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return -1;
    len = snprintf(buf, sizeof(buf), "%d", pin);
    written = write(fd, buf, (size_t)len);
    if (written < 0 && errno != EBUSY) {
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

static int gpio_wait_path(int pin, unsigned timeout_ms) {
    char path[64];
    unsigned waited = 0;
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    while (waited <= timeout_ms) {
        if (access(path, F_OK) == 0) return 0;
        sleep_ms(10U);
        waited += 10U;
    }
    return -1;
}

static int gpio_set_direction_one(int pin, const char *direction) {
    char path[64];
    int fd;
    ssize_t written;
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    written = write(fd, direction, strlen(direction));
    close(fd);
    return written < 0 ? -1 : 0;
}

static int gpio_write_one(int pin, int value) {
    char path[64];
    char ch = value ? '1' : '0';
    int fd;
    ssize_t written;
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    written = write(fd, &ch, 1U);
    close(fd);
    return written == 1 ? 0 : -1;
}

static int gpio_read_one(int pin) {
    char path[64];
    char ch = 0;
    int fd;
    ssize_t n;
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_RDONLY);
    if (fd < 0) return -1;
    n = read(fd, &ch, 1U);
    close(fd);
    if (n != 1) return -1;
    return ch == '1' ? 1 : 0;
}

static int lora_gpio_setup(LoraUartLink *link) {
    int aux;
    if (!link) return -1;

    if (gpio_export_one(link->gpio_m0) != 0 ||
        gpio_export_one(link->gpio_m1) != 0 ||
        gpio_export_one(link->gpio_aux) != 0) {
        fprintf(stderr, "[LORA] GPIO export failed M0=%d M1=%d AUX=%d: %s\n",
                link->gpio_m0, link->gpio_m1, link->gpio_aux, strerror(errno));
        return -1;
    }

    if (gpio_wait_path(link->gpio_m0, 500U) != 0 ||
        gpio_wait_path(link->gpio_m1, 500U) != 0 ||
        gpio_wait_path(link->gpio_aux, 500U) != 0) {
        fprintf(stderr, "[LORA] GPIO sysfs nodes did not appear\n");
        return -1;
    }

    if (gpio_set_direction_one(link->gpio_m0, "out") != 0 ||
        gpio_set_direction_one(link->gpio_m1, "out") != 0 ||
        gpio_set_direction_one(link->gpio_aux, "in") != 0) {
        fprintf(stderr, "[LORA] GPIO direction setup failed: %s\n", strerror(errno));
        return -1;
    }

    /* Normal transparent mode from the validated Lora.c: M0=0, M1=0. */
    if (gpio_write_one(link->gpio_m0, 0) != 0 ||
        gpio_write_one(link->gpio_m1, 0) != 0) {
        fprintf(stderr, "[LORA] failed to set Normal mode M0=0 M1=0: %s\n", strerror(errno));
        return -1;
    }
    sleep_ms(100U);

    link->gpio_ready = true;
    aux = gpio_read_one(link->gpio_aux);
    if (aux >= 0) {
        link->aux_ready = true;
        printf("[LORA] mode=Normal M0=0 M1=0 AUX=%d (1=idle,0=busy) GPIO=%d/%d/%d\n",
               aux, link->gpio_m0, link->gpio_m1, link->gpio_aux);
    } else {
        link->aux_ready = false;
        fprintf(stderr, "[LORA] AUX read unavailable; continue without AUX gating\n");
    }
    fflush(stdout);
    return 0;
}

static int wait_aux_idle(LoraUartLink *link, unsigned timeout_ms) {
    unsigned waited = 0;
    if (!link || !link->aux_ready || link->aux_advisory_only) return 0;

    while (waited <= timeout_ms) {
        int value = gpio_read_one(link->gpio_aux);
        if (value == 1) return 0;
        if (value < 0) {
            link->aux_ready = false;
            fprintf(stderr, "[LORA] AUX read failed; disable AUX gating for this run\n");
            return 0;
        }
        sleep_ms(DF_LORA_AUX_POLL_MS);
        waited += DF_LORA_AUX_POLL_MS;
    }
    ++link->aux_wait_timeouts;
    link->aux_advisory_only = true;
    fprintf(stderr, "[LORA] AUX stayed low for %ums; switch to advisory mode and continue TX\n", timeout_ms);
    return 0;
}

/* ----- UART ----- */
static int configure_uart(int fd, int baud_rate) {
    struct termios options;
    speed_t speed = baud_to_termios(baud_rate);
    if (speed == (speed_t)0) return -1;
    if (tcgetattr(fd, &options) != 0) return -1;

    /* Match the standalone Lora.c: 8N1, CLOCAL/CREAD, raw-ish, no SW flow control. */
    if (cfsetispeed(&options, speed) != 0) return -1;
    if (cfsetospeed(&options, speed) != 0) return -1;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
#ifdef CRTSCTS
    options.c_cflag &= ~CRTSCTS;
#endif
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) return -1;
    return 0;
}

static int open_uart(LoraUartLink *link) {
    int fd;
    if (!link) return -1;
    fd = open(link->device_path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) return -1;
    if (configure_uart(fd, link->baud_rate) != 0) {
        int saved_errno = errno;
        close(fd);
        errno = saved_errno;
        return -1;
    }
    link->fd = fd;
    printf("[LORA] UART opened device=%s baud=%d deviceId=%u\n",
           link->device_path, link->baud_rate, (unsigned)link->device_id);
    fflush(stdout);
    return 0;
}

static void close_uart(LoraUartLink *link) {
    if (!link) return;
    if (link->fd >= 0) {
        close(link->fd);
        link->fd = -1;
        printf("[LORA] UART closed\n");
        fflush(stdout);
    }
}

static ssize_t write_all(int fd, const uint8_t *data, size_t len) {
    size_t offset = 0;
    while (offset < len) {
        ssize_t n = write(fd, data + offset, len - offset);
        if (n > 0) {
            offset += (size_t)n;
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd pfd;
            int pr;
            pfd.fd = fd;
            pfd.events = POLLOUT;
            pfd.revents = 0;
            pr = poll(&pfd, 1, 500);
            if (pr > 0) continue;
            return -1;
        }
        return -1;
    }
    return (ssize_t)offset;
}

/* ----- queue/state ----- */
static bool queue_pop(LoraUartLink *link, MonitorMessage *out) {
    bool ok = false;
    pthread_mutex_lock(&link->queue_mutex);
    if (link->tx_count > 0U) {
        *out = link->tx_queue[link->tx_head];
        link->tx_head = (link->tx_head + 1U) % DF_LORA_TX_QUEUE_CAPACITY;
        --link->tx_count;
        ok = true;
    }
    pthread_mutex_unlock(&link->queue_mutex);
    return ok;
}

static bool link_should_stop(LoraUartLink *link) {
    bool stop;
    pthread_mutex_lock(&link->queue_mutex);
    stop = link->stop_requested;
    pthread_mutex_unlock(&link->queue_mutex);
    return stop;
}

static void set_in_flight(LoraUartLink *link, bool value) {
    pthread_mutex_lock(&link->queue_mutex);
    link->tx_in_flight = value;
    pthread_mutex_unlock(&link->queue_mutex);
}

static bool link_is_idle(LoraUartLink *link) {
    bool idle;
    pthread_mutex_lock(&link->queue_mutex);
    idle = (link->tx_count == 0U && !link->tx_in_flight);
    pthread_mutex_unlock(&link->queue_mutex);
    return idle;
}

/* ----- binary RX ----- */
static void remove_rx_prefix(LoraUartLink *link, size_t count) {
    if (count >= link->rx_size) {
        link->rx_size = 0U;
        return;
    }
    memmove(link->rx_buffer, link->rx_buffer + count, link->rx_size - count);
    link->rx_size -= count;
}

static void parse_rx_frames(LoraUartLink *link) {
    while (link->rx_size >= 2U) {
        size_t i;
        size_t frame_len = 0U;
        int peek_ret;
        DfTelemetryDecoded d;
        int decode_ret;

        for (i = 0U; i + 1U < link->rx_size; ++i) {
            if (link->rx_buffer[i] == DF_WIRE_MAGIC0 &&
                link->rx_buffer[i + 1U] == DF_WIRE_MAGIC1)
                break;
        }
        if (i > 0U) remove_rx_prefix(link, i);
        if (link->rx_size < 2U) return;
        if (link->rx_buffer[0] != DF_WIRE_MAGIC0 || link->rx_buffer[1] != DF_WIRE_MAGIC1) {
            remove_rx_prefix(link, 1U);
            continue;
        }

        peek_ret = df_wire_peek_frame_length(link->rx_buffer, link->rx_size, &frame_len);
        if (peek_ret == 1) return;
        if (peek_ret != 0 || frame_len > sizeof(link->rx_buffer)) {
            ++link->rx_bad;
            remove_rx_prefix(link, 1U);
            continue;
        }

        if (link->rx_buffer[3] == (uint8_t)DF_WIRE_HEARTBEAT) {
            DfHeartbeatDecoded hb;
            decode_ret = df_heartbeat_decode(link->rx_buffer, frame_len, &hb);
            if (decode_ret == 0) {
                ++link->rx_ok;
                printf("[LORA-RX] dev=%u seq=%u type=HEARTBEAT uptime=%us txOK=%u "
                       "uart=%u auxHigh=%u auxAdvisory=%u\n",
                       (unsigned)hb.source_device_id, (unsigned)hb.sequence,
                       (unsigned)hb.uptime_s, (unsigned)hb.tx_ok,
                       (unsigned)((hb.flags & DF_HEARTBEAT_FLAG_UART_OPEN) ? 1U : 0U),
                       (unsigned)((hb.flags & DF_HEARTBEAT_FLAG_AUX_HIGH) ? 1U : 0U),
                       (unsigned)((hb.flags & DF_HEARTBEAT_FLAG_AUX_ADVISORY) ? 1U : 0U));
                fflush(stdout);
                remove_rx_prefix(link, frame_len);
                continue;
            }
        } else {
            decode_ret = df_telemetry_decode(link->rx_buffer, frame_len, &d);
            if (decode_ret == 0) {
                ++link->rx_ok;
                printf("[LORA-RX] dev=%u seq=%u type=%s event=%u durationMs=%u "
                       "gully=%.3f moving=%.3f blob=%.3f blobArea=%u speed=%.1f avg=%.1f "
                       "dir=%u dirConsistency=%.3f snapshots=%u endReason=%u hop=%u clockValid=%u\n",
                       (unsigned)d.source_device_id,
                       (unsigned)d.sequence,
                       df_wire_packet_type_name(d.type),
                       (unsigned)d.event_id,
                       (unsigned)d.duration_ms,
                       (double)d.max_gully_x10000 / 10000.0,
                       (double)d.max_moving_x10000 / 10000.0,
                       (double)d.max_blob_x10000 / 10000.0,
                       (unsigned)d.max_blob_area,
                       (double)d.max_speed_x10 / 10.0,
                       (double)d.avg_speed_x10 / 10.0,
                       (unsigned)d.direction,
                       (double)d.direction_consistency_255 / 255.0,
                       (unsigned)d.snapshot_count,
                       (unsigned)d.end_reason,
                       (unsigned)d.hop_count,
                       (unsigned)((d.flags & DF_WIRE_FLAG_CLOCK_VALID) ? 1U : 0U));
                fflush(stdout);
                remove_rx_prefix(link, frame_len);
                continue;
            }
        }
        ++link->rx_bad;
        fprintf(stderr, "[LORA-RX] bad frame decode=%d buffered=%zu\n", decode_ret, link->rx_size);
        remove_rx_prefix(link, 1U);
    }
}

static void receive_uart(LoraUartLink *link) {
    uint8_t temp[128];
    for (;;) {
        ssize_t n = read(link->fd, temp, sizeof(temp));
        if (n > 0) {
            size_t copy_len = (size_t)n;
            if (copy_len > sizeof(link->rx_buffer) - link->rx_size) {
                ++link->rx_bad;
                link->rx_size = 0U;
            }
            memcpy(link->rx_buffer + link->rx_size, temp, copy_len);
            link->rx_size += copy_len;
            parse_rx_frames(link);
            continue;
        }
        if (n == 0) return;
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        fprintf(stderr, "[LORA] UART read error: %s\n", strerror(errno));
        close_uart(link);
        return;
    }
}

static void *lora_worker(void *arg) {
    LoraUartLink *link = (LoraUartLink *)arg;
    MonitorMessage pending;
    bool pending_valid = false;
    bool gpio_attempted = false;
    const uint64_t worker_start_ms = monotonic_ms();
    uint64_t next_heartbeat_ms = link->heartbeat_interval_s > 0U
        ? worker_start_ms + (uint64_t)link->heartbeat_interval_s * 1000ULL : 0ULL;

    while (!link_should_stop(link)) {
        uint8_t frame[DF_WIRE_MAX_FRAME_SIZE];
        size_t frame_len;

        if (!gpio_attempted) {
            /* GPIO failure should not kill camera/event processing. */
            if (lora_gpio_setup(link) != 0) {
                fprintf(stderr, "[LORA] GPIO setup failed; retry in %ums\n", DF_LORA_RECONNECT_MS);
                sleep_ms(DF_LORA_RECONNECT_MS);
                continue;
            }
            gpio_attempted = true;
        }

        if (link->fd < 0) {
            if (open_uart(link) != 0) {
                fprintf(stderr, "[LORA] open %s failed: %s; retrying\n",
                        link->device_path, strerror(errno));
                sleep_ms(DF_LORA_RECONNECT_MS);
                continue;
            }
        }

        if (!pending_valid) {
            pending_valid = queue_pop(link, &pending);
            if (pending_valid) set_in_flight(link, true);
        }

        if (pending_valid) {
            (void)wait_aux_idle(link, DF_LORA_AUX_PRE_TX_TIMEOUT_MS);

            frame_len = df_telemetry_encode(&pending, link->device_id, frame, sizeof(frame));
            if (frame_len == 0U) {
                fprintf(stderr, "[LORA-TX] unsupported message seq=%llu type=%d\n",
                        (unsigned long long)pending.sequence, (int)pending.type);
                pending_valid = false;
                set_in_flight(link, false);
            } else if (write_all(link->fd, frame, frame_len) == (ssize_t)frame_len) {
                /* Match standalone Lora.c behavior: block until UART bytes leave the driver. */
                if (tcdrain(link->fd) != 0)
                    fprintf(stderr, "[LORA-TX] tcdrain warning seq=%llu: %s\n",
                            (unsigned long long)pending.sequence, strerror(errno));

                ++link->tx_ok;
                printf("[LORA-TX] dev=%u seq=%llu type=%s event=%llu bytes=%zu status=OK\n",
                       (unsigned)link->device_id,
                       (unsigned long long)pending.sequence,
                       df_wire_packet_type_name((DfWirePacketType)frame[3]),
                       (unsigned long long)pending.event.event_id,
                       frame_len);
                fflush(stdout);
                pending_valid = false;
                set_in_flight(link, false);

                (void)wait_aux_idle(link, DF_LORA_AUX_POST_TX_TIMEOUT_MS);
            } else {
                ++link->tx_fail;
                fprintf(stderr, "[LORA-TX] UART write failed seq=%llu: %s; keep packet for retry\n",
                        (unsigned long long)pending.sequence, strerror(errno));
                close_uart(link);
                continue;
            }
        }

        if (!pending_valid && link->fd >= 0 && link->heartbeat_interval_s > 0U &&
            monotonic_ms() >= next_heartbeat_ms) {
            uint8_t hb_flags = DF_HEARTBEAT_FLAG_UART_OPEN;
            int aux_value = link->aux_ready ? gpio_read_one(link->gpio_aux) : -1;
            if (aux_value == 1) hb_flags |= DF_HEARTBEAT_FLAG_AUX_HIGH;
            if (link->aux_advisory_only) hb_flags |= DF_HEARTBEAT_FLAG_AUX_ADVISORY;
            ++link->heartbeat_sequence;
            frame_len = df_heartbeat_encode(link->device_id, link->heartbeat_sequence,
                                            (uint32_t)((monotonic_ms() - worker_start_ms) / 1000ULL),
                                            (uint32_t)(link->tx_ok > 0xffffffffULL ? 0xffffffffULL : link->tx_ok),
                                            hb_flags, frame, sizeof(frame));
            if (frame_len > 0U) {
                (void)wait_aux_idle(link, DF_LORA_AUX_PRE_TX_TIMEOUT_MS);
                if (write_all(link->fd, frame, frame_len) == (ssize_t)frame_len) {
                    (void)tcdrain(link->fd);
                    ++link->tx_ok;
                    printf("[LORA-TX] dev=%u hbSeq=%u type=HEARTBEAT uptime=%llus bytes=%zu status=OK\n",
                           (unsigned)link->device_id, (unsigned)link->heartbeat_sequence,
                           (unsigned long long)((monotonic_ms() - worker_start_ms) / 1000ULL), frame_len);
                    fflush(stdout);
                } else {
                    ++link->tx_fail;
                    fprintf(stderr, "[LORA-TX] HEARTBEAT UART write failed: %s\n", strerror(errno));
                    close_uart(link);
                }
            }
            next_heartbeat_ms = monotonic_ms() + (uint64_t)link->heartbeat_interval_s * 1000ULL;
        }

        if (link->fd >= 0) {
            struct pollfd pfd;
            int pr;
            pfd.fd = link->fd;
            pfd.events = POLLIN;
            pfd.revents = 0;
            pr = poll(&pfd, 1, 50);
            if (pr > 0 && (pfd.revents & POLLIN)) receive_uart(link);
            else if (pr < 0 && errno != EINTR) {
                fprintf(stderr, "[LORA] UART poll error: %s\n", strerror(errno));
                close_uart(link);
            }
        } else {
            sleep_ms(20U);
        }
    }

    set_in_flight(link, false);
    close_uart(link);
    return NULL;
}

int lora_uart_init(LoraUartLink *link,
                   const char *device_path,
                   int baud_rate,
                   uint16_t device_id,
                   int gpio_m0,
                   int gpio_m1,
                   int gpio_aux,
                   unsigned heartbeat_interval_s) {
    if (!link || !device_path || !device_path[0] || baud_to_termios(baud_rate) == (speed_t)0)
        return -1;
    memset(link, 0, sizeof(*link));
    snprintf(link->device_path, sizeof(link->device_path), "%s", device_path);
    link->baud_rate = baud_rate;
    link->device_id = device_id;
    link->gpio_m0 = gpio_m0;
    link->gpio_m1 = gpio_m1;
    link->gpio_aux = gpio_aux;
    link->heartbeat_interval_s = heartbeat_interval_s;
    link->fd = -1;
    if (pthread_mutex_init(&link->queue_mutex, NULL) != 0) return -1;
    link->mutex_ready = true;
    return 0;
}

int lora_uart_start(LoraUartLink *link) {
    if (!link || !link->mutex_ready || link->worker_started) return -1;
    link->stop_requested = false;
    if (pthread_create(&link->worker_thread, NULL, lora_worker, link) != 0) return -1;
    link->worker_started = true;
    printf("[LORA] worker started device=%s baud=%d deviceId=%u txQueue=%u "
           "M0=%d M1=%d AUX=%d heartbeat=%us\n",
           link->device_path, link->baud_rate, (unsigned)link->device_id,
           (unsigned)DF_LORA_TX_QUEUE_CAPACITY,
           link->gpio_m0, link->gpio_m1, link->gpio_aux, link->heartbeat_interval_s);
    fflush(stdout);
    return 0;
}

bool lora_uart_enqueue(LoraUartLink *link, const MonitorMessage *message) {
    size_t index;
    if (!link || !message || !link->mutex_ready) return false;
    if (message->type != MONITOR_EVENT_START &&
        message->type != MONITOR_EVENT_UPDATE &&
        message->type != MONITOR_EVENT_END)
        return false;

    pthread_mutex_lock(&link->queue_mutex);
    if (link->tx_count >= DF_LORA_TX_QUEUE_CAPACITY) {
        /* Preserve START/END semantics as much as possible: discard an old UPDATE first. */
        size_t i;
        bool removed = false;
        for (i = 0U; i < link->tx_count; ++i) {
            size_t pos = (link->tx_head + i) % DF_LORA_TX_QUEUE_CAPACITY;
            if (link->tx_queue[pos].type == MONITOR_EVENT_UPDATE) {
                size_t j;
                for (j = i; j + 1U < link->tx_count; ++j) {
                    size_t from = (link->tx_head + j + 1U) % DF_LORA_TX_QUEUE_CAPACITY;
                    size_t to = (link->tx_head + j) % DF_LORA_TX_QUEUE_CAPACITY;
                    link->tx_queue[to] = link->tx_queue[from];
                }
                link->tx_tail = (link->tx_tail + DF_LORA_TX_QUEUE_CAPACITY - 1U) % DF_LORA_TX_QUEUE_CAPACITY;
                --link->tx_count;
                removed = true;
                break;
            }
        }
        if (!removed) {
            link->tx_head = (link->tx_head + 1U) % DF_LORA_TX_QUEUE_CAPACITY;
            --link->tx_count;
        }
        ++link->dropped_messages;
        fprintf(stderr, "[LORA-QUEUE] overflow drop=%llu\n",
                (unsigned long long)link->dropped_messages);
    }

    index = link->tx_tail;
    link->tx_queue[index] = *message;
    link->tx_tail = (link->tx_tail + 1U) % DF_LORA_TX_QUEUE_CAPACITY;
    ++link->tx_count;
    pthread_mutex_unlock(&link->queue_mutex);
    return true;
}

bool lora_uart_wait_queue_empty(LoraUartLink *link, unsigned timeout_ms) {
    unsigned waited = 0U;
    if (!link || !link->mutex_ready) return true;
    while (waited < timeout_ms) {
        if (link_is_idle(link)) return true;
        sleep_ms(20U);
        waited += 20U;
    }
    return link_is_idle(link);
}

void lora_uart_stop(LoraUartLink *link) {
    if (!link || !link->worker_started) return;
    pthread_mutex_lock(&link->queue_mutex);
    link->stop_requested = true;
    pthread_mutex_unlock(&link->queue_mutex);
    pthread_join(link->worker_thread, NULL);
    link->worker_started = false;
    printf("[LORA] worker stopped txOK=%llu txFail=%llu rxOK=%llu rxBad=%llu dropped=%llu auxTimeout=%llu\n",
           (unsigned long long)link->tx_ok,
           (unsigned long long)link->tx_fail,
           (unsigned long long)link->rx_ok,
           (unsigned long long)link->rx_bad,
           (unsigned long long)link->dropped_messages,
           (unsigned long long)link->aux_wait_timeouts);
    fflush(stdout);
}

void lora_uart_deinit(LoraUartLink *link) {
    if (!link) return;
    lora_uart_stop(link);
    close_uart(link);
    if (link->mutex_ready) {
        pthread_mutex_destroy(&link->queue_mutex);
        link->mutex_ready = false;
    }
}
