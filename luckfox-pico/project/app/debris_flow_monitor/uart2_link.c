#include "uart2_link.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static speed_t baud_to_termios(int baud_rate) {
    switch (baud_rate) {
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
    nanosleep(&ts, NULL);
}

static int configure_uart(int fd, int baud_rate) {
    struct termios tty;
    speed_t speed = baud_to_termios(baud_rate);
    if (speed == (speed_t)0) return -1;
    if (tcgetattr(fd, &tty) != 0) return -1;

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    tty.c_cflag |= (CS8 | CLOCAL | CREAD);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (cfsetispeed(&tty, speed) != 0) return -1;
    if (cfsetospeed(&tty, speed) != 0) return -1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
    tcflush(fd, TCIOFLUSH);
    return 0;
}

static int open_uart(Uart2Link *link) {
    int fd;
    if (!link) return -1;
    fd = open(link->device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    if (configure_uart(fd, link->baud_rate) != 0) {
        close(fd);
        return -1;
    }
    link->fd = fd;
    printf("[UART2] opened device=%s baud=%d deviceId=%u\n",
           link->device_path, link->baud_rate, (unsigned)link->device_id);
    fflush(stdout);
    return 0;
}

static void close_uart(Uart2Link *link) {
    if (!link) return;
    if (link->fd >= 0) {
        close(link->fd);
        link->fd = -1;
        printf("[UART2] closed\n");
        fflush(stdout);
    }
}

static bool queue_pop(Uart2Link *link, MonitorMessage *out) {
    bool ok = false;
    pthread_mutex_lock(&link->queue_mutex);
    if (link->tx_count > 0) {
        *out = link->tx_queue[link->tx_head];
        link->tx_head = (link->tx_head + 1U) % DF_UART2_TX_QUEUE_CAPACITY;
        --link->tx_count;
        ok = true;
    }
    pthread_mutex_unlock(&link->queue_mutex);
    return ok;
}

static bool link_should_stop(Uart2Link *link) {
    bool stop;
    pthread_mutex_lock(&link->queue_mutex);
    stop = link->stop_requested;
    pthread_mutex_unlock(&link->queue_mutex);
    return stop;
}

static void set_in_flight(Uart2Link *link, bool value) {
    pthread_mutex_lock(&link->queue_mutex);
    link->tx_in_flight = value;
    pthread_mutex_unlock(&link->queue_mutex);
}

static bool link_is_idle(Uart2Link *link) {
    bool idle;
    pthread_mutex_lock(&link->queue_mutex);
    idle = (link->tx_count == 0U && !link->tx_in_flight);
    pthread_mutex_unlock(&link->queue_mutex);
    return idle;
}

static ssize_t write_all_nonblocking(int fd, const uint8_t *data, size_t len) {
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

static void remove_rx_prefix(Uart2Link *link, size_t count) {
    if (count >= link->rx_size) {
        link->rx_size = 0;
        return;
    }
    memmove(link->rx_buffer, link->rx_buffer + count, link->rx_size - count);
    link->rx_size -= count;
}

static void parse_rx_frames(Uart2Link *link) {
    while (link->rx_size >= 2U) {
        size_t i;
        size_t frame_len = 0;
        int peek_ret;
        DfTelemetryDecoded d;
        int decode_ret;

        for (i = 0; i + 1U < link->rx_size; ++i) {
            if (link->rx_buffer[i] == DF_WIRE_MAGIC0 &&
                link->rx_buffer[i + 1U] == DF_WIRE_MAGIC1)
                break;
        }
        if (i > 0) remove_rx_prefix(link, i);
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

        decode_ret = df_telemetry_decode(link->rx_buffer, frame_len, &d);
        if (decode_ret == 0) {
            ++link->rx_ok;
            printf("[UART2-RX] dev=%u seq=%u type=%s event=%u durationMs=%u "
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
        } else {
            ++link->rx_bad;
            fprintf(stderr, "[UART2-RX] bad frame decode=%d buffered=%zu\n", decode_ret, link->rx_size);
            remove_rx_prefix(link, 1U);
        }
    }
}

static void receive_uart(Uart2Link *link) {
    uint8_t temp[128];
    for (;;) {
        ssize_t n = read(link->fd, temp, sizeof(temp));
        if (n > 0) {
            size_t copy_len = (size_t)n;
            if (copy_len > sizeof(link->rx_buffer) - link->rx_size) {
                /* Drop a stale/incomplete stream fragment; one read is <= temp[128]. */
                ++link->rx_bad;
                link->rx_size = 0;
            }
            memcpy(link->rx_buffer + link->rx_size, temp, copy_len);
            link->rx_size += copy_len;
            parse_rx_frames(link);
            continue;
        }
        if (n == 0) return;
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        fprintf(stderr, "[UART2] read error: %s\n", strerror(errno));
        close_uart(link);
        return;
    }
}

static void *uart_worker(void *arg) {
    Uart2Link *link = (Uart2Link *)arg;
    MonitorMessage pending;
    bool pending_valid = false;
    unsigned reconnect_wait_ms = 0;

    while (!link_should_stop(link)) {
        uint8_t frame[DF_WIRE_MAX_FRAME_SIZE];
        size_t frame_len;

        if (link->fd < 0) {
            if (reconnect_wait_ms == 0U) {
                if (open_uart(link) == 0) reconnect_wait_ms = 0U;
                else {
                    fprintf(stderr, "[UART2] open %s failed: %s; retrying\n",
                            link->device_path, strerror(errno));
                    reconnect_wait_ms = 1000U;
                }
            }
            if (link->fd < 0) {
                sleep_ms(reconnect_wait_ms ? reconnect_wait_ms : 100U);
                reconnect_wait_ms = 0U;
                continue;
            }
        }

        if (!pending_valid) {
            pending_valid = queue_pop(link, &pending);
            if (pending_valid) set_in_flight(link, true);
        }
        if (pending_valid) {
            frame_len = df_telemetry_encode(&pending, link->device_id, frame, sizeof(frame));
            if (frame_len == 0) {
                fprintf(stderr, "[UART2-TX] unsupported message seq=%llu type=%d\n",
                        (unsigned long long)pending.sequence, (int)pending.type);
                pending_valid = false;
                set_in_flight(link, false);
            } else if (write_all_nonblocking(link->fd, frame, frame_len) == (ssize_t)frame_len) {
                ++link->tx_ok;
                printf("[UART2-TX] dev=%u seq=%llu type=%s event=%llu bytes=%zu status=OK\n",
                       (unsigned)link->device_id,
                       (unsigned long long)pending.sequence,
                       df_wire_packet_type_name((DfWirePacketType)frame[3]),
                       (unsigned long long)pending.event.event_id,
                       frame_len);
                fflush(stdout);
                pending_valid = false;
                set_in_flight(link, false);
            } else {
                ++link->tx_fail;
                fprintf(stderr, "[UART2-TX] write failed seq=%llu: %s; keep packet for retry\n",
                        (unsigned long long)pending.sequence, strerror(errno));
                close_uart(link);
                continue;
            }
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
                fprintf(stderr, "[UART2] poll error: %s\n", strerror(errno));
                close_uart(link);
            }
        }
    }

    set_in_flight(link, false);
    close_uart(link);
    return NULL;
}

int uart2_link_init(Uart2Link *link,
                    const char *device_path,
                    int baud_rate,
                    uint16_t device_id) {
    if (!link || !device_path || !device_path[0] || baud_to_termios(baud_rate) == (speed_t)0)
        return -1;
    memset(link, 0, sizeof(*link));
    snprintf(link->device_path, sizeof(link->device_path), "%s", device_path);
    link->baud_rate = baud_rate;
    link->device_id = device_id;
    link->fd = -1;
    if (pthread_mutex_init(&link->queue_mutex, NULL) != 0) return -1;
    link->mutex_ready = true;
    return 0;
}

int uart2_link_start(Uart2Link *link) {
    if (!link || !link->mutex_ready || link->worker_started) return -1;
    link->stop_requested = false;
    if (pthread_create(&link->worker_thread, NULL, uart_worker, link) != 0) return -1;
    link->worker_started = true;
    printf("[UART2] worker started device=%s baud=%d deviceId=%u txQueue=%u\n",
           link->device_path, link->baud_rate, (unsigned)link->device_id,
           (unsigned)DF_UART2_TX_QUEUE_CAPACITY);
    fflush(stdout);
    return 0;
}

bool uart2_link_enqueue(Uart2Link *link, const MonitorMessage *message) {
    size_t index;
    if (!link || !message || !link->mutex_ready) return false;
    if (message->type != MONITOR_EVENT_START &&
        message->type != MONITOR_EVENT_UPDATE &&
        message->type != MONITOR_EVENT_END)
        return false;

    pthread_mutex_lock(&link->queue_mutex);
    if (link->tx_count >= DF_UART2_TX_QUEUE_CAPACITY) {
        /* Prefer dropping an old UPDATE before dropping START/END semantics. */
        size_t i;
        bool removed = false;
        for (i = 0; i < link->tx_count; ++i) {
            size_t pos = (link->tx_head + i) % DF_UART2_TX_QUEUE_CAPACITY;
            if (link->tx_queue[pos].type == MONITOR_EVENT_UPDATE) {
                size_t j;
                for (j = i; j + 1U < link->tx_count; ++j) {
                    size_t from = (link->tx_head + j + 1U) % DF_UART2_TX_QUEUE_CAPACITY;
                    size_t to = (link->tx_head + j) % DF_UART2_TX_QUEUE_CAPACITY;
                    link->tx_queue[to] = link->tx_queue[from];
                }
                link->tx_tail = (link->tx_tail + DF_UART2_TX_QUEUE_CAPACITY - 1U) % DF_UART2_TX_QUEUE_CAPACITY;
                --link->tx_count;
                removed = true;
                break;
            }
        }
        if (!removed) {
            link->tx_head = (link->tx_head + 1U) % DF_UART2_TX_QUEUE_CAPACITY;
            --link->tx_count;
        }
        ++link->dropped_messages;
        fprintf(stderr, "[UART2-QUEUE] overflow drop=%llu\n",
                (unsigned long long)link->dropped_messages);
    }
    index = link->tx_tail;
    link->tx_queue[index] = *message;
    link->tx_tail = (link->tx_tail + 1U) % DF_UART2_TX_QUEUE_CAPACITY;
    ++link->tx_count;
    pthread_mutex_unlock(&link->queue_mutex);
    return true;
}

bool uart2_link_wait_queue_empty(Uart2Link *link, unsigned timeout_ms) {
    unsigned waited = 0;
    if (!link || !link->mutex_ready) return true;
    while (waited < timeout_ms) {
        if (link_is_idle(link)) return true;
        sleep_ms(20U);
        waited += 20U;
    }
    return link_is_idle(link);
}

void uart2_link_stop(Uart2Link *link) {
    if (!link || !link->worker_started) return;
    pthread_mutex_lock(&link->queue_mutex);
    link->stop_requested = true;
    pthread_mutex_unlock(&link->queue_mutex);
    pthread_join(link->worker_thread, NULL);
    link->worker_started = false;
    printf("[UART2] worker stopped txOK=%llu txFail=%llu rxOK=%llu rxBad=%llu dropped=%llu\n",
           (unsigned long long)link->tx_ok,
           (unsigned long long)link->tx_fail,
           (unsigned long long)link->rx_ok,
           (unsigned long long)link->rx_bad,
           (unsigned long long)link->dropped_messages);
    fflush(stdout);
}

void uart2_link_deinit(Uart2Link *link) {
    if (!link) return;
    uart2_link_stop(link);
    close_uart(link);
    if (link->mutex_ready) {
        pthread_mutex_destroy(&link->queue_mutex);
        link->mutex_ready = false;
    }
}
