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

/* DX-LR32-433T22D V2.0 AUX semantics:
 *   HIGH = radio busy (TX/RX/mode switching)
 *   LOW  = radio idle / operation complete
 * Use AUX as the primary pacing signal instead of fixed per-chunk sleeps. */
#define DF_LORA_AUX_POLL_MS 2U
#define DF_LORA_AUX_IDLE_STABLE_SAMPLES 2U
#define DF_LORA_AUX_PRE_TX_TIMEOUT_MS 5000U
#define DF_LORA_AUX_POST_TX_TIMEOUT_MS 5000U
#define DF_LORA_AUX_POST_ASSERT_WINDOW_MS 20U
#define DF_LORA_RECONNECT_MS 1000U

typedef enum {
    IMAGE_TX_PHASE_IDLE = 0,
    IMAGE_TX_PHASE_META,
    IMAGE_TX_PHASE_INITIAL,
    IMAGE_TX_PHASE_REPAIR,
    IMAGE_TX_PHASE_DONE,
    IMAGE_TX_PHASE_WAIT_FEEDBACK
} ImageTxPhase;

typedef struct {
    bool active;
    LoraImageJob image;
    ImageTxPhase phase;
    uint16_t total_chunks;
    uint16_t next_chunk;
    uint8_t round;
    uint8_t no_feedback_retries;
    bool feedback_seen;
    bool ack_seen;
    bool was_deferred;
    bool meta_refresh_needed;
    uint8_t missing_union[DF_LORA_IMAGE_BITMAP_BYTES];
    uint8_t repair_bitmap[DF_LORA_IMAGE_BITMAP_BYTES];
    uint64_t feedback_deadline_ms;
    uint64_t feedback_quiet_until_ms;
} LoraImageTxSession;

static int send_frame_bytes(LoraUartLink *link,
                            const uint8_t *frame,
                            size_t frame_len,
                            const char *tag);
static uint16_t chain_next_id(const LoraUartLink *link);

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

    /* DX-LR32-433T22D: M0=0, M1=0 selects high-efficiency mode.
     * Transparent UART transport still works in this mode. */
    if (gpio_write_one(link->gpio_m0, 0) != 0 ||
        gpio_write_one(link->gpio_m1, 0) != 0) {
        fprintf(stderr, "[LORA] failed to set high-efficiency mode M0=0 M1=0: %s\n", strerror(errno));
        return -1;
    }
    sleep_ms(100U);

    link->gpio_ready = true;
    aux = gpio_read_one(link->gpio_aux);
    if (aux >= 0) {
        link->aux_ready = true;
        printf("[LORA] mode=HIGH_EFFICIENCY M0=0 M1=0 AUX=%d "
               "(1=busy,0=idle/complete) GPIO=%d/%d/%d\n",
               aux, link->gpio_m0, link->gpio_m1, link->gpio_aux);
    } else {
        link->aux_ready = false;
        fprintf(stderr, "[LORA] AUX read unavailable; continue without AUX gating\n");
    }
    fflush(stdout);
    return 0;
}

static int aux_read(LoraUartLink *link) {
    int value;
    if (!link || !link->aux_ready || link->aux_advisory_only) return -2;
    value = gpio_read_one(link->gpio_aux);
    if (value >= 0) return value;
    link->aux_ready = false;
    link->aux_advisory_only = true;
    fprintf(stderr, "[LORA] AUX read failed; disable AUX gating for this run\n");
    return -2;
}

/* Wait until AUX is LOW and remains LOW for a few polls.  Per the module
 * manual LOW means the RF operation has completed and the module is idle.
 * A timeout does NOT switch to advisory mode: sending while AUX is HIGH would
 * deliberately overrun/collide with an active RF operation. */
static int wait_aux_idle_low(LoraUartLink *link, unsigned timeout_ms, const char *phase) {
    unsigned waited = 0U;
    unsigned low_samples = 0U;
    if (!link || !link->aux_ready || link->aux_advisory_only) return 0;

    while (waited <= timeout_ms) {
        int value = aux_read(link);
        if (value == -2) return 0;
        if (value == 0) {
            ++low_samples;
            if (low_samples >= DF_LORA_AUX_IDLE_STABLE_SAMPLES) return 0;
        } else {
            low_samples = 0U;
        }
        sleep_ms(DF_LORA_AUX_POLL_MS);
        waited += DF_LORA_AUX_POLL_MS;
    }

    ++link->aux_wait_timeouts;
    fprintf(stderr,
            "[LORA] AUX busy(HIGH) timeout phase=%s waited=%ums; defer next TX\n",
            phase ? phase : "unknown", timeout_ms);
    return 1;
}

/* After the UART frame has drained, AUX may already be LOW (RF finished while
 * UART was still shifting bytes), or it may transition HIGH while the module is
 * transmitting.  Watch a short assertion window; if HIGH is seen, wait for the
 * subsequent stable LOW before allowing the next frame. */
static void wait_aux_tx_complete(LoraUartLink *link, const char *tag) {
    unsigned waited = 0U;
    bool saw_busy = false;
    if (!link || !link->aux_ready || link->aux_advisory_only) return;

    while (waited <= DF_LORA_AUX_POST_ASSERT_WINDOW_MS) {
        int value = aux_read(link);
        if (value == -2) return;
        if (value == 1) {
            saw_busy = true;
            ++link->aux_busy_cycles;
            break;
        }
        sleep_ms(DF_LORA_AUX_POLL_MS);
        waited += DF_LORA_AUX_POLL_MS;
    }

    if (!saw_busy) {
        /* LOW means complete.  This is valid at high air rates where the BUSY
         * pulse can finish before tcdrain() returns. */
        ++link->aux_no_busy_observed;
        return;
    }

    (void)wait_aux_idle_low(link, DF_LORA_AUX_POST_TX_TIMEOUT_MS, tag ? tag : "post-tx");
}

/* ----- UART ----- */
static int configure_uart(int fd, int baud_rate) {
    struct termios options;
    struct termios applied;
    speed_t speed = baud_to_termios(baud_rate);
    if (speed == (speed_t)0) return -1;
    if (tcgetattr(fd, &options) != 0) return -1;

    if (cfsetispeed(&options, speed) != 0) return -1;
    if (cfsetospeed(&options, speed) != 0) return -1;

    /*
     * Binary-transparent UART for the LoRa wire protocol.
     *
     * The DF frames contain arbitrary byte values (including 0x0A, 0x0D,
     * 0x11 and 0x13), therefore Linux tty text processing must not alter
     * any received/transmitted byte.  In particular ICRNL must be disabled;
     * otherwise an incoming 0x0D can be translated to 0x0A before the CRC
     * checker sees the frame.
     */
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;

    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
#ifdef CRTSCTS
    options.c_cflag &= ~CRTSCTS;
#endif

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;

    if (tcflush(fd, TCIFLUSH) != 0) return -1;
    if (tcsetattr(fd, TCSANOW, &options) != 0) return -1;

    /* Read back the applied flags so deployment logs can prove that no tty
     * input/output/local translation survived the tcsetattr(). */
    if (tcgetattr(fd, &applied) != 0) return -1;
    if (applied.c_iflag != 0 || applied.c_oflag != 0 || applied.c_lflag != 0) {
        errno = EPROTO;
        return -1;
    }

    printf("[LORA] UART raw-binary iflag=0x%lx oflag=0x%lx lflag=0x%lx 8N1=1\n",
           (unsigned long)applied.c_iflag,
           (unsigned long)applied.c_oflag,
           (unsigned long)applied.c_lflag);
    fflush(stdout);
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

/* ----- queue/state ----- */
static bool queue_pop_prioritized(LoraUartLink *link,
                                  MonitorMessage *out,
                                  bool allow_update) {
    bool ok = false;
    size_t i, chosen = SIZE_MAX;
    pthread_mutex_lock(&link->queue_mutex);
    for (i = 0U; i < link->tx_count; ++i) {
        size_t pos = (link->tx_head + i) % DF_LORA_TX_QUEUE_CAPACITY;
        MonitorMessageType type = link->tx_queue[pos].type;
        if (type == MONITOR_EVENT_START || type == MONITOR_EVENT_END) {
            chosen = i;
            break;
        }
    }
    if (chosen == SIZE_MAX && allow_update && link->tx_count > 0U) chosen = 0U;
    if (chosen != SIZE_MAX) {
        size_t pos = (link->tx_head + chosen) % DF_LORA_TX_QUEUE_CAPACITY;
        size_t j;
        *out = link->tx_queue[pos];
        for (j = chosen; j + 1U < link->tx_count; ++j) {
            size_t from = (link->tx_head + j + 1U) % DF_LORA_TX_QUEUE_CAPACITY;
            size_t to = (link->tx_head + j) % DF_LORA_TX_QUEUE_CAPACITY;
            link->tx_queue[to] = link->tx_queue[from];
        }
        link->tx_tail = (link->tx_tail + DF_LORA_TX_QUEUE_CAPACITY - 1U) % DF_LORA_TX_QUEUE_CAPACITY;
        --link->tx_count;
        ok = true;
    }
    pthread_mutex_unlock(&link->queue_mutex);
    return ok;
}

static bool image_queue_pop(LoraUartLink *link, LoraImageJob *out) {
    bool ok = false;
    size_t i, chosen = SIZE_MAX;
    uint64_t now = monotonic_ms();
    pthread_mutex_lock(&link->queue_mutex);
    for (i = 0U; i < link->image_count; ++i) {
        const LoraImageJob *job = &link->image_queue[i];
        if (job->not_before_ms > now) continue;
        if (chosen == SIZE_MAX) {
            chosen = i;
            continue;
        }
        /* EVENT evidence outranks deployment preview; relay/local are otherwise FIFO. */
        if (job->purpose == (uint8_t)DF_IMAGE_PURPOSE_EVENT &&
            link->image_queue[chosen].purpose != (uint8_t)DF_IMAGE_PURPOSE_EVENT)
            chosen = i;
    }
    if (chosen != SIZE_MAX) {
        *out = link->image_queue[chosen];
        for (i = chosen + 1U; i < link->image_count; ++i)
            link->image_queue[i - 1U] = link->image_queue[i];
        --link->image_count;
        ok = true;
    }
    pthread_mutex_unlock(&link->queue_mutex);
    return ok;
}

static void image_queue_remove_at_locked(LoraUartLink *link, size_t index) {
    size_t i;
    if (!link || index >= link->image_count) return;
    for (i = index + 1U; i < link->image_count; ++i)
        link->image_queue[i - 1U] = link->image_queue[i];
    --link->image_count;
}

static bool image_queue_push_relay_from_slot(LoraUartLink *link,
                                             LoraImageRxSlot *slot,
                                             uint64_t not_before_ms) {
    LoraImageJob job;
    size_t i;
    if (!link || !slot || !slot->active || !slot->complete || slot->relay_queued ||
        !link->chain_enabled) return false;
    if (slot->hop_count == 0xffU) return false;

    memset(&job, 0, sizeof(job));
    job.source_device_id = slot->source_device_id;
    job.hop_count = (uint8_t)(slot->hop_count + 1U);
    job.relayed = true;
    job.expect_feedback = chain_next_id(link) != 0U;
    job.not_before_ms = not_before_ms;
    job.image_id = slot->image_id;
    job.event_id = slot->event_id;
    job.capture_epoch_ms = (uint64_t)slot->capture_epoch_s * 1000ULL;
    job.width = slot->width;
    job.height = slot->height;
    job.quality = slot->quality;
    job.purpose = slot->purpose;
    job.jpeg_size = slot->total_size;
    memcpy(job.jpeg, slot->data, slot->total_size);

    pthread_mutex_lock(&link->queue_mutex);
    if (link->image_count >= DF_LORA_IMAGE_QUEUE_CAPACITY) {
        for (i = 0U; i < link->image_count; ++i) {
            if (link->image_queue[i].purpose == (uint8_t)DF_IMAGE_PURPOSE_DEPLOY_PREVIEW) {
                image_queue_remove_at_locked(link, i);
                ++link->dropped_images;
                break;
            }
        }
    }
    if (link->image_count >= DF_LORA_IMAGE_QUEUE_CAPACITY) {
        ++link->dropped_images;
        pthread_mutex_unlock(&link->queue_mutex);
        fprintf(stderr,
                "[LORA-CHAIN] drop relay image origin=%u image=%u queue full dropped=%llu\n",
                (unsigned)slot->source_device_id, (unsigned)slot->image_id,
                (unsigned long long)link->dropped_images);
        return false;
    }
    link->image_queue[link->image_count++] = job;
    slot->relay_queued = true;
    ++link->chain_images_queued;
    pthread_mutex_unlock(&link->queue_mutex);

    printf("[LORA-CHAIN] store-forward queued origin=%u image=%u hop=%u->%u size=%u notBefore=%llums expectFeedback=%u\n",
           (unsigned)slot->source_device_id, (unsigned)slot->image_id,
           (unsigned)slot->hop_count, (unsigned)job.hop_count,
           (unsigned)slot->total_size,
           (unsigned long long)not_before_ms,
           (unsigned)(job.expect_feedback ? 1U : 0U));
    fflush(stdout);
    return true;
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

static void set_image_in_flight(LoraUartLink *link, bool value) {
    pthread_mutex_lock(&link->queue_mutex);
    link->image_tx_in_flight = value;
    pthread_mutex_unlock(&link->queue_mutex);
}

static bool link_is_idle(LoraUartLink *link) {
    bool idle;
    pthread_mutex_lock(&link->queue_mutex);
    idle = (link->tx_count == 0U && link->image_count == 0U &&
            !link->tx_in_flight && !link->image_tx_in_flight);
    pthread_mutex_unlock(&link->queue_mutex);
    return idle;
}

static const char *image_purpose_name(uint8_t purpose) {
    if (purpose == (uint8_t)DF_IMAGE_PURPOSE_EVENT) return "EVENT";
    if (purpose == (uint8_t)DF_IMAGE_PURPOSE_DEPLOY_PREVIEW) return "DEPLOY_PREVIEW";
    return "UNKNOWN";
}

static void bitmap_zero(uint8_t *bitmap, size_t bytes) {
    if (bitmap && bytes > 0U) memset(bitmap, 0, bytes);
}

static bool bitmap_test_raw(const uint8_t *bitmap, uint16_t index) {
    return bitmap && (bitmap[index / 8U] & (uint8_t)(1U << (index % 8U))) != 0U;
}

static void bitmap_set_raw(uint8_t *bitmap, uint16_t index) {
    if (bitmap) bitmap[index / 8U] |= (uint8_t)(1U << (index % 8U));
}

static bool bitmap_any_raw(const uint8_t *bitmap, size_t bytes) {
    size_t i;
    if (!bitmap) return false;
    for (i = 0U; i < bytes; ++i) if (bitmap[i] != 0U) return true;
    return false;
}

static uint16_t bitmap_count_raw(const uint8_t *bitmap, uint16_t total_chunks) {
    uint16_t i, count = 0U;
    for (i = 0U; i < total_chunks; ++i) if (bitmap_test_raw(bitmap, i)) ++count;
    return count;
}

static void bitmap_or_raw(uint8_t *dst, const uint8_t *src, size_t bytes) {
    size_t i;
    if (!dst || !src) return;
    for (i = 0U; i < bytes; ++i) dst[i] |= src[i];
}

static uint32_t reliability_hash(uint16_t a, uint16_t b, uint32_t c, uint8_t d) {
    uint32_t x = 2166136261U;
    x = (x ^ (uint32_t)a) * 16777619U;
    x = (x ^ (uint32_t)b) * 16777619U;
    x = (x ^ c) * 16777619U;
    x = (x ^ (uint32_t)d) * 16777619U;
    return x;
}

static uint16_t chain_prev_id(const LoraUartLink *link) {
    if (!link || !link->chain_enabled || link->device_id <= link->chain_first_id) return 0U;
    return (uint16_t)(link->device_id - 1U);
}

static uint16_t chain_next_id(const LoraUartLink *link) {
    if (!link || !link->chain_enabled || link->device_id >= link->chain_last_id) return 0U;
    return (uint16_t)(link->device_id + 1U);
}

static bool chain_accept_downstream_origin_hop(const LoraUartLink *link,
                                               uint16_t source_device_id,
                                               uint8_t hop_count) {
    uint16_t prev;
    uint32_t logical_sender;
    if (!link || !link->chain_enabled) return true;
    prev = chain_prev_id(link);
    if (prev == 0U) return false;
    logical_sender = (uint32_t)source_device_id + (uint32_t)hop_count;
    return logical_sender == (uint32_t)prev;
}

static bool chain_feedback_from_next(const LoraUartLink *link, uint16_t responder_device_id) {
    uint16_t next;
    if (!link || !link->chain_enabled) return true;
    next = chain_next_id(link);
    return next != 0U && responder_device_id == next;
}


/* ----- controlled broadcast relay -----
 *
 * Frames are forwarded byte-for-byte so source_device_id / sequence / image_id
 * remain the original monitor's identity.  There is deliberately no rewritten
 * "last sender" field: the 4G terminal can keep using the existing payload IDs.
 *
 * Loop prevention uses a short-lived frame hash cache.  A repeated copy heard
 * within DF_LORA_RELAY_DUP_SUPPRESS_MS cancels this node's pending relay of the
 * same frame.  The window is shorter than EVENT retry and IMAGE repair timing,
 * so later reliability retransmissions are allowed to propagate again.
 */
static uint64_t relay_frame_hash64(const uint8_t *data, size_t len) {
    uint64_t h = 1469598103934665603ULL;
    size_t i;
    if (!data) return 0ULL;
    for (i = 0U; i < len; ++i) {
        h ^= (uint64_t)data[i];
        h *= 1099511628211ULL;
    }
    return h;
}

static uint16_t relay_frame_emitter(const uint8_t *frame, size_t frame_len) {
    /* All V1 wire payloads currently start with source_device_id or
     * responder_device_id in little-endian form. */
    if (!frame || frame_len < DF_WIRE_HEADER_SIZE + 2U + DF_WIRE_CRC_SIZE) return 0U;
    return (uint16_t)((uint16_t)frame[DF_WIRE_HEADER_SIZE] |
                      ((uint16_t)frame[DF_WIRE_HEADER_SIZE + 1U] << 8));
}

static uint8_t relay_priority_for_type(uint8_t packet_type) {
    if (packet_type == (uint8_t)DF_WIRE_IMAGE_FEEDBACK ||
        packet_type == (uint8_t)DF_WIRE_EVENT_ACK)
        return (uint8_t)LORA_RELAY_PRIORITY_CONTROL;
    if (packet_type == (uint8_t)DF_WIRE_EVENT_START ||
        packet_type == (uint8_t)DF_WIRE_EVENT_END ||
        packet_type == (uint8_t)DF_WIRE_IMAGE_DONE ||
        packet_type == (uint8_t)DF_WIRE_IMAGE_META)
        return (uint8_t)LORA_RELAY_PRIORITY_EVENT;
    return (uint8_t)LORA_RELAY_PRIORITY_BULK;
}

static LoraRelayCacheEntry *relay_cache_find(LoraUartLink *link,
                                             uint64_t hash,
                                             size_t frame_len,
                                             uint8_t packet_type) {
    size_t i;
    if (!link) return NULL;
    for (i = 0U; i < DF_LORA_RELAY_CACHE_CAPACITY; ++i) {
        LoraRelayCacheEntry *e = &link->relay_cache[i];
        if (e->valid && e->frame_hash == hash && e->frame_len == frame_len &&
            e->packet_type == packet_type)
            return e;
    }
    return NULL;
}

static LoraRelayCacheEntry *relay_cache_allocate(LoraUartLink *link) {
    size_t i, oldest = 0U;
    uint64_t oldest_ms = UINT64_MAX;
    const uint64_t now = monotonic_ms();
    for (i = 0U; i < DF_LORA_RELAY_CACHE_CAPACITY; ++i) {
        LoraRelayCacheEntry *e = &link->relay_cache[i];
        if (!e->valid || (now > e->last_seen_ms &&
            now - e->last_seen_ms > DF_LORA_RELAY_CACHE_EXPIRE_MS))
            return e;
        if (e->last_seen_ms < oldest_ms) {
            oldest_ms = e->last_seen_ms;
            oldest = i;
        }
    }
    return &link->relay_cache[oldest];
}

static bool relay_cancel_pending(LoraUartLink *link,
                                 uint64_t hash,
                                 size_t frame_len,
                                 uint8_t packet_type) {
    size_t i;
    bool cancelled = false;
    for (i = 0U; i < DF_LORA_RELAY_QUEUE_CAPACITY; ++i) {
        LoraRelayFrame *r = &link->relay_queue[i];
        if (r->active && r->frame_hash == hash && r->frame_len == frame_len &&
            r->packet_type == packet_type) {
            r->active = false;
            cancelled = true;
        }
    }
    return cancelled;
}

static LoraRelayFrame *relay_queue_allocate(LoraUartLink *link, uint8_t new_priority) {
    size_t i, replace = SIZE_MAX;
    uint8_t worst_priority = 0U;
    uint64_t latest_due = 0U;
    for (i = 0U; i < DF_LORA_RELAY_QUEUE_CAPACITY; ++i) {
        LoraRelayFrame *r = &link->relay_queue[i];
        if (!r->active) return r;
        if (replace == SIZE_MAX || r->priority > worst_priority ||
            (r->priority == worst_priority && r->due_ms > latest_due)) {
            replace = i;
            worst_priority = r->priority;
            latest_due = r->due_ms;
        }
    }
    if (replace != SIZE_MAX && worst_priority > new_priority) {
        ++link->relay_dropped;
        return &link->relay_queue[replace];
    }
    return NULL;
}

static void relay_consider_valid_frame(LoraUartLink *link,
                                       const uint8_t *frame,
                                       size_t frame_len) {
    uint8_t packet_type;
    uint8_t priority;
    uint16_t emitter;
    uint64_t hash;
    uint64_t now;
    LoraRelayCacheEntry *cache;
    LoraRelayFrame *relay;
    uint8_t new_hop = 0U;
    if (!link || !link->chain_enabled || !frame ||
        frame_len < DF_WIRE_HEADER_SIZE + DF_WIRE_CRC_SIZE) return;
    packet_type = frame[3];
    if (packet_type < (uint8_t)DF_WIRE_EVENT_START ||
        packet_type > (uint8_t)DF_WIRE_EVENT_END) return;

    emitter = relay_frame_emitter(frame, frame_len);
    if (emitter == 0U || emitter == link->device_id) {
        if (emitter == link->device_id) ++link->relay_own_ignored;
        return;
    }

    hash = relay_frame_hash64(frame, frame_len);
    now = monotonic_ms();
    cache = relay_cache_find(link, hash, frame_len, packet_type);
    if (cache && now >= cache->last_seen_ms &&
        now - cache->last_seen_ms < DF_LORA_RELAY_DUP_SUPPRESS_MS) {
        (void)relay_cancel_pending(link, hash, frame_len, packet_type);
        cache->last_seen_ms = now;
        ++link->relay_suppressed;
        return;
    }
    if (!cache) cache = relay_cache_allocate(link);
    memset(cache, 0, sizeof(*cache));
    cache->valid = true;
    cache->frame_hash = hash;
    cache->frame_len = frame_len;
    cache->packet_type = packet_type;
    cache->emitter_device_id = emitter;
    cache->last_seen_ms = now;

    priority = relay_priority_for_type(packet_type);
    relay = relay_queue_allocate(link, priority);
    if (!relay) {
        ++link->relay_dropped;
        fprintf(stderr, "[LORA-CHAIN] relay queue full drop origin=%u type=%s\n",
                (unsigned)emitter,
                df_wire_packet_type_name((DfWirePacketType)packet_type));
        return;
    }
    memset(relay, 0, sizeof(*relay));
    relay->active = true;
    relay->frame_hash = hash;
    relay->emitter_device_id = emitter;
    relay->packet_type = packet_type;
    relay->priority = priority;
    relay->frame_len = frame_len;
    memcpy(relay->frame, frame, frame_len);
    if (df_wire_increment_hop_count(relay->frame, relay->frame_len, &new_hop) != 0) {
        relay->active = false;
        ++link->relay_dropped;
        return;
    }
    relay->due_ms = now + DF_LORA_CHAIN_EVENT_RELAY_DELAY_MS;
    ++link->relay_queued;
    printf("[LORA-CHAIN] queue EVENT origin=%u type=%s hop=%u->%u delay=%ums\n",
           (unsigned)emitter,
           df_wire_packet_type_name((DfWirePacketType)packet_type),
           (unsigned)(new_hop - 1U), (unsigned)new_hop,
           (unsigned)DF_LORA_CHAIN_EVENT_RELAY_DELAY_MS);
    fflush(stdout);
}

static void register_critical_event_tx(LoraUartLink *link,
                                       const uint8_t *frame,
                                       size_t frame_len,
                                       uint16_t source_device_id,
                                       uint32_t sequence,
                                       uint32_t event_id,
                                       uint8_t event_type);

static bool send_due_relay(LoraUartLink *link, uint8_t max_priority) {
    size_t i, chosen = SIZE_MAX;
    uint8_t best_priority = 0xffU;
    uint64_t best_due = UINT64_MAX;
    uint64_t now = monotonic_ms();
    int ret;
    for (i = 0U; i < DF_LORA_RELAY_QUEUE_CAPACITY; ++i) {
        LoraRelayFrame *r = &link->relay_queue[i];
        if (!r->active || r->priority > max_priority || r->due_ms > now) continue;
        if (chosen == SIZE_MAX || r->priority < best_priority ||
            (r->priority == best_priority && r->due_ms < best_due)) {
            chosen = i;
            best_priority = r->priority;
            best_due = r->due_ms;
        }
    }
    if (chosen == SIZE_MAX) return false;
    ret = send_frame_bytes(link, link->relay_queue[chosen].frame,
                           link->relay_queue[chosen].frame_len, "CHAIN_EVENT_RELAY");
    if (ret == 0) {
        LoraRelayFrame *r = &link->relay_queue[chosen];
        DfTelemetryDecoded d;
        ++link->relay_tx_ok;
        if (df_telemetry_decode(r->frame, r->frame_len, &d) == 0) {
            printf("[LORA-CHAIN-TX] origin=%u via=%u type=%s seq=%u event=%u hop=%u bytes=%zu status=OK\n",
                   (unsigned)d.source_device_id, (unsigned)link->device_id,
                   df_wire_packet_type_name(d.type), (unsigned)d.sequence,
                   (unsigned)d.event_id, (unsigned)d.hop_count, r->frame_len);
            fflush(stdout);
            if (chain_next_id(link) != 0U &&
                (d.type == DF_WIRE_EVENT_START || d.type == DF_WIRE_EVENT_END)) {
                register_critical_event_tx(link, r->frame, r->frame_len,
                                           d.source_device_id, d.sequence, d.event_id,
                                           (uint8_t)d.type);
            }
        }
        r->active = false;
    } else if (ret > 0) {
        link->relay_queue[chosen].due_ms = monotonic_ms() + 50U;
    } else {
        link->relay_queue[chosen].due_ms = monotonic_ms() + DF_LORA_RECONNECT_MS;
    }
    return true;
}

static uint64_t image_feedback_due_ms(const LoraUartLink *link,
                                      uint16_t source_device_id,
                                      uint32_t image_id,
                                      uint8_t round) {
    if (link && link->chain_enabled)
        return monotonic_ms() + DF_LORA_CHAIN_IMAGE_FEEDBACK_TURNAROUND_MS;
    {
        uint32_t slot = reliability_hash(link->device_id, source_device_id, image_id, round) %
                        DF_LORA_IMAGE_FEEDBACK_SLOT_COUNT;
        return monotonic_ms() + DF_LORA_IMAGE_FEEDBACK_BASE_MS +
               (uint64_t)slot * DF_LORA_IMAGE_FEEDBACK_SLOT_MS;
    }
}

static uint64_t event_ack_due_ms(const LoraUartLink *link,
                                 uint16_t source_device_id,
                                 uint32_t sequence,
                                 uint8_t event_type) {
    if (link && link->chain_enabled)
        return monotonic_ms() + DF_LORA_CHAIN_EVENT_ACK_TURNAROUND_MS;
    {
        uint32_t slot = reliability_hash(link->device_id, source_device_id, sequence, event_type) %
                        DF_LORA_EVENT_ACK_SLOT_COUNT;
        return monotonic_ms() + DF_LORA_EVENT_ACK_BASE_MS +
               (uint64_t)slot * DF_LORA_EVENT_ACK_SLOT_MS;
    }
}

static LoraScheduledControl *allocate_control_slot(LoraUartLink *link) {
    size_t i, oldest = 0U;
    uint64_t oldest_due = UINT64_MAX;
    for (i = 0U; i < DF_LORA_RELIABILITY_CONTROL_QUEUE_CAPACITY; ++i) {
        if (!link->control_tx[i].active) return &link->control_tx[i];
        if (link->control_tx[i].due_ms < oldest_due) {
            oldest_due = link->control_tx[i].due_ms;
            oldest = i;
        }
    }
    fprintf(stderr, "[LORA-REL] control queue full; replace oldest kind=%d\n",
            (int)link->control_tx[oldest].kind);
    return &link->control_tx[oldest];
}

static bool schedule_control_frame(LoraUartLink *link,
                                   LoraControlKind kind,
                                   uint64_t due_ms,
                                   const uint8_t *frame,
                                   size_t frame_len,
                                   uint16_t source_device_id,
                                   uint32_t sequence,
                                   uint32_t image_id,
                                   uint8_t event_type,
                                   uint8_t round,
                                   uint8_t copies_total,
                                   uint32_t repeat_interval_ms) {
    size_t i;
    LoraScheduledControl *slot;
    if (!link || !frame || frame_len == 0U || frame_len > DF_WIRE_MAX_FRAME_SIZE) return false;
    for (i = 0U; i < DF_LORA_RELIABILITY_CONTROL_QUEUE_CAPACITY; ++i) {
        LoraScheduledControl *c = &link->control_tx[i];
        if (!c->active || c->kind != kind) continue;
        if (kind == LORA_CONTROL_EVENT_ACK && c->source_device_id == source_device_id &&
            c->sequence == sequence && c->event_type == event_type) {
            c->due_ms = due_ms;
            c->frame_len = frame_len;
            memcpy(c->frame, frame, frame_len);
            c->copies_total = copies_total > 0U ? copies_total : 1U;
            c->copies_sent = 0U;
            c->repeat_interval_ms = repeat_interval_ms;
            return true;
        }
        if (kind == LORA_CONTROL_IMAGE_FEEDBACK && c->source_device_id == source_device_id &&
            c->image_id == image_id && c->round == round) {
            c->due_ms = due_ms;
            c->frame_len = frame_len;
            memcpy(c->frame, frame, frame_len);
            c->copies_total = copies_total > 0U ? copies_total : 1U;
            c->copies_sent = 0U;
            c->repeat_interval_ms = repeat_interval_ms;
            return true;
        }
    }
    slot = allocate_control_slot(link);
    memset(slot, 0, sizeof(*slot));
    slot->active = true;
    slot->kind = kind;
    slot->due_ms = due_ms;
    slot->frame_len = frame_len;
    memcpy(slot->frame, frame, frame_len);
    slot->source_device_id = source_device_id;
    slot->sequence = sequence;
    slot->image_id = image_id;
    slot->event_type = event_type;
    slot->round = round;
    slot->copies_total = copies_total > 0U ? copies_total : 1U;
    slot->copies_sent = 0U;
    slot->repeat_interval_ms = repeat_interval_ms;
    return true;
}

static void cancel_scheduled_event_ack(LoraUartLink *link,
                                       uint16_t source_device_id,
                                       uint32_t sequence,
                                       uint8_t event_type) {
    size_t i;
    for (i = 0U; i < DF_LORA_RELIABILITY_CONTROL_QUEUE_CAPACITY; ++i) {
        LoraScheduledControl *c = &link->control_tx[i];
        if (c->active && c->kind == LORA_CONTROL_EVENT_ACK &&
            c->source_device_id == source_device_id && c->sequence == sequence &&
            c->event_type == event_type) {
            c->active = false;
        }
    }
}

static bool feedback_inbox_push(LoraUartLink *link, const DfImageFeedbackDecoded *fb) {
    if (link->image_feedback_rx_count >= DF_LORA_RELIABILITY_CONTROL_QUEUE_CAPACITY) {
        link->image_feedback_rx_head = (link->image_feedback_rx_head + 1U) %
                                       DF_LORA_RELIABILITY_CONTROL_QUEUE_CAPACITY;
        --link->image_feedback_rx_count;
    }
    link->image_feedback_rx[link->image_feedback_rx_tail] = *fb;
    link->image_feedback_rx_tail = (link->image_feedback_rx_tail + 1U) %
                                   DF_LORA_RELIABILITY_CONTROL_QUEUE_CAPACITY;
    ++link->image_feedback_rx_count;
    return true;
}

static bool feedback_inbox_pop(LoraUartLink *link, DfImageFeedbackDecoded *out) {
    if (!link || !out || link->image_feedback_rx_count == 0U) return false;
    *out = link->image_feedback_rx[link->image_feedback_rx_head];
    link->image_feedback_rx_head = (link->image_feedback_rx_head + 1U) %
                                   DF_LORA_RELIABILITY_CONTROL_QUEUE_CAPACITY;
    --link->image_feedback_rx_count;
    return true;
}

static void critical_event_ack_received(LoraUartLink *link, const DfEventAckDecoded *ack) {
    size_t i;
    if (!link || !ack) return;
    if (!chain_feedback_from_next(link, ack->responder_device_id)) {
        if (link->chain_enabled) ++link->chain_filtered_frames;
        return;
    }
    cancel_scheduled_event_ack(link, ack->source_device_id, ack->sequence, ack->event_type);
    for (i = 0U; i < DF_LORA_CRITICAL_EVENT_PENDING_CAPACITY; ++i) {
        LoraCriticalEventPending *p = &link->critical_event_tx[i];
        if (p->active && p->source_device_id == ack->source_device_id &&
            p->sequence == ack->sequence && p->event_id == ack->event_id &&
            p->event_type == ack->event_type) {
            p->active = false;
            ++link->event_ack_rx;
            printf("[LORA-REL] EVENT_ACK from=%u forOrigin=%u seq=%u event=%u type=%s status=CONFIRMED\n",
                   (unsigned)ack->responder_device_id, (unsigned)ack->source_device_id,
                   (unsigned)ack->sequence, (unsigned)ack->event_id,
                   df_wire_packet_type_name((DfWirePacketType)ack->event_type));
            fflush(stdout);
            return;
        }
    }
}

static void register_critical_event_tx(LoraUartLink *link,
                                       const uint8_t *frame,
                                       size_t frame_len,
                                       uint16_t source_device_id,
                                       uint32_t sequence,
                                       uint32_t event_id,
                                       uint8_t event_type) {
    size_t i, replace = 0U;
    uint64_t oldest = UINT64_MAX;
    if (!link || !frame || frame_len == 0U ||
        (event_type != (uint8_t)DF_WIRE_EVENT_START && event_type != (uint8_t)DF_WIRE_EVENT_END))
        return;
    for (i = 0U; i < DF_LORA_CRITICAL_EVENT_PENDING_CAPACITY; ++i) {
        LoraCriticalEventPending *p = &link->critical_event_tx[i];
        if (p->active && p->source_device_id == source_device_id &&
            p->sequence == sequence && p->event_type == event_type) return;
        if (!p->active) { replace = i; break; }
        if (p->next_retry_ms < oldest) { oldest = p->next_retry_ms; replace = i; }
    }
    if (link->critical_event_tx[replace].active) ++link->critical_event_unconfirmed;
    memset(&link->critical_event_tx[replace], 0, sizeof(link->critical_event_tx[replace]));
    link->critical_event_tx[replace].active = true;
    link->critical_event_tx[replace].source_device_id = source_device_id;
    link->critical_event_tx[replace].sequence = sequence;
    link->critical_event_tx[replace].event_id = event_id;
    link->critical_event_tx[replace].event_type = event_type;
    link->critical_event_tx[replace].frame_len = frame_len;
    memcpy(link->critical_event_tx[replace].frame, frame, frame_len);
    {
        uint64_t now = monotonic_ms();
        link->critical_event_tx[replace].next_retry_ms = now + DF_LORA_EVENT_ACK_RETRY_MS;
        link->critical_event_tx[replace].quiet_until_ms =
            (link->chain_enabled && chain_next_id(link) != 0U)
            ? now + DF_LORA_CHAIN_EVENT_SENDER_QUIET_MS : 0U;
    }
}

static bool critical_event_quiet_active(const LoraUartLink *link) {
    size_t i;
    uint64_t now;
    if (!link || !link->chain_enabled) return false;
    now = monotonic_ms();
    for (i = 0U; i < DF_LORA_CRITICAL_EVENT_PENDING_CAPACITY; ++i) {
        const LoraCriticalEventPending *p = &link->critical_event_tx[i];
        if (p->active && p->quiet_until_ms > now) return true;
    }
    return false;
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

static bool image_bitmap_test(const LoraImageRxSlot *slot, uint16_t index) {
    return (slot->received_bitmap[index / 8U] & (uint8_t)(1U << (index % 8U))) != 0U;
}

static void image_bitmap_set(LoraImageRxSlot *slot, uint16_t index) {
    slot->received_bitmap[index / 8U] |= (uint8_t)(1U << (index % 8U));
}

static LoraImageRxSlot *find_image_rx_slot(LoraUartLink *link,
                                            uint16_t source_device_id,
                                            uint32_t image_id) {
    size_t i;
    for (i = 0U; i < DF_LORA_IMAGE_RX_SLOT_COUNT; ++i) {
        LoraImageRxSlot *slot = &link->image_rx[i];
        if (slot->active && slot->source_device_id == source_device_id && slot->image_id == image_id)
            return slot;
    }
    return NULL;
}

static LoraImageRxSlot *allocate_image_rx_slot(LoraUartLink *link,
                                                uint16_t source_device_id,
                                                uint32_t image_id) {
    size_t i, oldest = 0U;
    uint64_t oldest_ms = UINT64_MAX;
    /* A sender serializes images; a newer meta from the same source means the old
     * incomplete image can no longer receive chunks and should be replaced first. */
    for (i = 0U; i < DF_LORA_IMAGE_RX_SLOT_COUNT; ++i) {
        if (link->image_rx[i].active && link->image_rx[i].source_device_id == source_device_id) {
            if (!link->image_rx[i].complete) {
                ++link->rx_images_incomplete;
                fprintf(stderr,
                        "[LORA-RX-IMAGE] replace same-source incomplete dev=%u image=%u received=%u/%u with image=%u\n",
                        (unsigned)source_device_id, (unsigned)link->image_rx[i].image_id,
                        (unsigned)link->image_rx[i].received_chunks,
                        (unsigned)link->image_rx[i].total_chunks, (unsigned)image_id);
            }
            return &link->image_rx[i];
        }
    }
    for (i = 0U; i < DF_LORA_IMAGE_RX_SLOT_COUNT; ++i) {
        if (!link->image_rx[i].active) return &link->image_rx[i];
        if (link->image_rx[i].last_activity_ms < oldest_ms) {
            oldest_ms = link->image_rx[i].last_activity_ms;
            oldest = i;
        }
    }
    if (link->image_rx[oldest].active && !link->image_rx[oldest].complete) {
        ++link->rx_images_incomplete;
        fprintf(stderr,
                "[LORA-RX-IMAGE] replace incomplete dev=%u image=%u received=%u/%u for dev=%u image=%u\n",
                (unsigned)link->image_rx[oldest].source_device_id,
                (unsigned)link->image_rx[oldest].image_id,
                (unsigned)link->image_rx[oldest].received_chunks,
                (unsigned)link->image_rx[oldest].total_chunks,
                (unsigned)source_device_id, (unsigned)image_id);
    }
    return &link->image_rx[oldest];
}

static void finish_received_image(LoraUartLink *link, LoraImageRxSlot *slot) {
    char path[160];
    FILE *fp;
    size_t written;
    if (slot->purpose == (uint8_t)DF_IMAGE_PURPOSE_DEPLOY_PREVIEW) {
        snprintf(path, sizeof(path), "/tmp/debris_rx_dev%u_latest.jpg",
                 (unsigned)slot->source_device_id);
    } else {
        snprintf(path, sizeof(path), "/tmp/debris_rx_dev%u_event%u_img%u.jpg",
                 (unsigned)slot->source_device_id,
                 (unsigned)slot->event_id,
                 (unsigned)slot->image_id);
    }
    fp = fopen(path, "wb");
    if (!fp) {
        fprintf(stderr, "[LORA-RX-IMAGE] fopen failed path=%s: %s\n", path, strerror(errno));
        slot->active = false;
        return;
    }
    written = fwrite(slot->data, 1U, slot->total_size, fp);
    fclose(fp);
    if (written != slot->total_size) {
        fprintf(stderr, "[LORA-RX-IMAGE] short write path=%s %zu/%u\n",
                path, written, (unsigned)slot->total_size);
        slot->active = false;
        return;
    }
    if (!slot->file_saved) ++link->rx_images_ok;
    slot->complete = true;
    slot->file_saved = true;
    printf("[LORA-RX-IMAGE] dev=%u image=%u event=%u purpose=%s size=%u %ux%u q=%u "
           "chunks=%u status=COMPLETE path=%s\n",
           (unsigned)slot->source_device_id,
           (unsigned)slot->image_id,
           (unsigned)slot->event_id,
           image_purpose_name(slot->purpose),
           (unsigned)slot->total_size,
           (unsigned)slot->width, (unsigned)slot->height,
           (unsigned)slot->quality,
           (unsigned)slot->total_chunks,
           path);
    fflush(stdout);
    slot->last_activity_ms = monotonic_ms();
}

static int handle_image_meta(LoraUartLink *link, const uint8_t *frame, size_t frame_len) {
    DfImageMetaDecoded meta;
    LoraImageRxSlot *slot;
    uint16_t expected_chunks;
    int ret = df_image_meta_decode(frame, frame_len, &meta);
    if (ret != 0) return ret;
    if (meta.source_device_id == link->device_id) return 0;
    if (!chain_accept_downstream_origin_hop(link, meta.source_device_id, meta.hop_count)) {
        if (link->chain_enabled) ++link->chain_filtered_frames;
        return 0;
    }
    expected_chunks = (uint16_t)((meta.total_size + DF_IMAGE_CHUNK_DATA_MAX - 1U) /
                                 DF_IMAGE_CHUNK_DATA_MAX);
    if (meta.format != DF_IMAGE_FORMAT_JPEG_GRAY ||
        meta.total_size == 0U || meta.total_size > DF_IMAGE_MAX_JPEG_BYTES ||
        meta.total_chunks == 0U || meta.total_chunks > DF_LORA_IMAGE_MAX_CHUNKS ||
        meta.total_chunks != expected_chunks || meta.width == 0U || meta.height == 0U)
        return -20;
    slot = find_image_rx_slot(link, meta.source_device_id, meta.image_id);
    if (slot) {
        if (slot->event_id != meta.event_id || slot->total_size != meta.total_size ||
            slot->width != meta.width || slot->height != meta.height ||
            slot->total_chunks != meta.total_chunks || slot->quality != meta.quality ||
            slot->purpose != meta.purpose || slot->hop_count != meta.hop_count)
            return -24;
        slot->last_activity_ms = monotonic_ms();
        printf("[LORA-RX-IMAGE] dev=%u image=%u type=IMAGE_META_DUP received=%u/%u\n",
               (unsigned)meta.source_device_id, (unsigned)meta.image_id,
               (unsigned)slot->received_chunks, (unsigned)slot->total_chunks);
        fflush(stdout);
        return 0;
    }
    slot = allocate_image_rx_slot(link, meta.source_device_id, meta.image_id);
    memset(slot, 0, sizeof(*slot));
    slot->active = true;
    slot->source_device_id = meta.source_device_id;
    slot->image_id = meta.image_id;
    slot->event_id = meta.event_id;
    slot->capture_epoch_s = meta.capture_epoch_s;
    slot->total_size = meta.total_size;
    slot->width = meta.width;
    slot->height = meta.height;
    slot->total_chunks = meta.total_chunks;
    slot->quality = meta.quality;
    slot->purpose = meta.purpose;
    slot->flags = meta.flags;
    slot->hop_count = meta.hop_count;
    slot->last_activity_ms = monotonic_ms();
    printf("[LORA-RX-IMAGE] dev=%u image=%u type=IMAGE_META event=%u purpose=%s size=%u "
           "%ux%u q=%u chunks=%u hop=%u clockValid=%u\n",
           (unsigned)meta.source_device_id, (unsigned)meta.image_id,
           (unsigned)meta.event_id, image_purpose_name(meta.purpose),
           (unsigned)meta.total_size, (unsigned)meta.width, (unsigned)meta.height,
           (unsigned)meta.quality, (unsigned)meta.total_chunks, (unsigned)meta.hop_count,
           (unsigned)((meta.flags & DF_WIRE_FLAG_CLOCK_VALID) ? 1U : 0U));
    fflush(stdout);
    return 0;
}

static int handle_image_chunk(LoraUartLink *link, const uint8_t *frame, size_t frame_len) {
    DfImageChunkDecoded chunk;
    LoraImageRxSlot *slot;
    size_t offset;
    int ret = df_image_chunk_decode(frame, frame_len, &chunk);
    if (ret != 0) return ret;
    if (chunk.source_device_id == link->device_id) return 0;
    slot = find_image_rx_slot(link, chunk.source_device_id, chunk.image_id);
    if (!slot) {
        if (link->chain_enabled) {
            ++link->chain_filtered_frames;
            return 0;
        }
        fprintf(stderr, "[LORA-RX-IMAGE] orphan chunk dev=%u image=%u chunk=%u/%u\n",
                (unsigned)chunk.source_device_id, (unsigned)chunk.image_id,
                (unsigned)(chunk.chunk_index + 1U), (unsigned)chunk.total_chunks);
        return 0;
    }
    if (slot->event_id != chunk.event_id || slot->total_chunks != chunk.total_chunks ||
        slot->purpose != chunk.purpose || chunk.chunk_index >= slot->total_chunks)
        return -21;
    offset = (size_t)chunk.chunk_index * DF_IMAGE_CHUNK_DATA_MAX;
    if (offset + chunk.data_len > slot->total_size) return -22;
    {
        size_t expected_len = slot->total_size - offset;
        if (expected_len > DF_IMAGE_CHUNK_DATA_MAX) expected_len = DF_IMAGE_CHUNK_DATA_MAX;
        if ((size_t)chunk.data_len != expected_len) return -23;
    }
    if (!image_bitmap_test(slot, chunk.chunk_index)) {
        memcpy(slot->data + offset, chunk.data, chunk.data_len);
        image_bitmap_set(slot, chunk.chunk_index);
        ++slot->received_chunks;
    }
    slot->last_activity_ms = monotonic_ms();
    if (chunk.chunk_index == 0U ||
        (chunk.chunk_index + 1U) == slot->total_chunks ||
        ((chunk.chunk_index + 1U) % 10U) == 0U) {
        printf("[LORA-RX-IMAGE] dev=%u image=%u chunk=%u/%u received=%u\n",
               (unsigned)chunk.source_device_id, (unsigned)chunk.image_id,
               (unsigned)(chunk.chunk_index + 1U), (unsigned)chunk.total_chunks,
               (unsigned)slot->received_chunks);
        fflush(stdout);
    }
    if (slot->received_chunks == slot->total_chunks && !slot->complete) finish_received_image(link, slot);
    return 0;
}

static void build_missing_bitmap(const LoraImageRxSlot *slot,
                                 uint8_t *missing,
                                 uint8_t *bitmap_len_out) {
    uint16_t i;
    uint8_t bytes;
    if (!slot || !missing || !bitmap_len_out) return;
    bytes = (uint8_t)((slot->total_chunks + 7U) / 8U);
    bitmap_zero(missing, DF_LORA_IMAGE_BITMAP_BYTES);
    for (i = 0U; i < slot->total_chunks; ++i) {
        if (!image_bitmap_test(slot, i)) bitmap_set_raw(missing, i);
    }
    *bitmap_len_out = bytes;
}

static int handle_image_done(LoraUartLink *link, const uint8_t *frame, size_t frame_len) {
    DfImageDoneDecoded done;
    LoraImageRxSlot *slot;
    uint8_t tx_frame[DF_WIRE_MAX_FRAME_SIZE];
    size_t tx_len;
    uint8_t missing[DF_LORA_IMAGE_BITMAP_BYTES];
    uint8_t bitmap_len = 0U;
    uint8_t flags = 0U;
    uint16_t missing_count = 0U;
    int ret = df_image_done_decode(frame, frame_len, &done);
    if (ret != 0) return ret;
    if (done.source_device_id == link->device_id) return 0;
    slot = find_image_rx_slot(link, done.source_device_id, done.image_id);
    if (!slot) {
        if (link->chain_enabled) {
            ++link->chain_filtered_frames;
            return 0;
        }
        fprintf(stderr, "[LORA-RX-IMAGE] DONE without META dev=%u image=%u round=%u\n",
                (unsigned)done.source_device_id, (unsigned)done.image_id, (unsigned)done.round);
        return 0;
    }
    if (slot->event_id != done.event_id || slot->total_chunks != done.total_chunks ||
        slot->purpose != done.purpose) return -25;
    slot->last_done_round = done.round;
    slot->last_activity_ms = monotonic_ms();
    if (slot->received_chunks == slot->total_chunks) {
        if (!slot->complete) finish_received_image(link, slot);
        flags = DF_IMAGE_FEEDBACK_FLAG_COMPLETE;
        tx_len = df_image_feedback_encode(link->device_id, done.source_device_id,
                                          done.image_id, done.event_id, done.total_chunks,
                                          done.round, flags, NULL, 0U,
                                          tx_frame, sizeof(tx_frame));
        printf("[LORA-RX-IMAGE] dev=%u image=%u type=IMAGE_DONE round=%u received=%u/%u feedback=ACK\n",
               (unsigned)done.source_device_id, (unsigned)done.image_id, (unsigned)done.round,
               (unsigned)slot->received_chunks, (unsigned)slot->total_chunks);
    } else {
        build_missing_bitmap(slot, missing, &bitmap_len);
        missing_count = bitmap_count_raw(missing, slot->total_chunks);
        tx_len = df_image_feedback_encode(link->device_id, done.source_device_id,
                                          done.image_id, done.event_id, done.total_chunks,
                                          done.round, 0U, missing, bitmap_len,
                                          tx_frame, sizeof(tx_frame));
        printf("[LORA-RX-IMAGE] dev=%u image=%u type=IMAGE_DONE round=%u received=%u/%u feedback=NACK missing=%u\n",
               (unsigned)done.source_device_id, (unsigned)done.image_id, (unsigned)done.round,
               (unsigned)slot->received_chunks, (unsigned)slot->total_chunks,
               (unsigned)missing_count);
    }
    fflush(stdout);
    if (tx_len == 0U) return -26;
    {
        uint64_t feedback_due = image_feedback_due_ms(link, done.source_device_id, done.image_id, done.round);
        uint8_t copies = link->chain_enabled ? DF_LORA_CHAIN_IMAGE_FEEDBACK_COPIES : 1U;
        uint32_t repeat_ms = link->chain_enabled ? DF_LORA_CHAIN_IMAGE_FEEDBACK_REPEAT_MS : 0U;
        (void)schedule_control_frame(link, LORA_CONTROL_IMAGE_FEEDBACK,
                                     feedback_due, tx_frame, tx_len, done.source_device_id, 0U,
                                     done.image_id, 0U, done.round, copies, repeat_ms);
        if (slot->received_chunks == slot->total_chunks && link->chain_enabled) {
            uint64_t relay_after = feedback_due +
                (uint64_t)(copies > 0U ? copies - 1U : 0U) * (uint64_t)repeat_ms +
                DF_LORA_CHAIN_IMAGE_RELAY_GUARD_MS;
            (void)image_queue_push_relay_from_slot(link, slot, relay_after);
        }
    }
    return 0;
}

static int handle_image_feedback(LoraUartLink *link, const uint8_t *frame, size_t frame_len) {
    DfImageFeedbackDecoded fb;
    int ret = df_image_feedback_decode(frame, frame_len, &fb);
    if (ret != 0) return ret;
    if (!chain_feedback_from_next(link, fb.responder_device_id)) {
        if (link->chain_enabled) ++link->chain_filtered_frames;
        return 0;
    }
    ++link->image_feedback_rx_count_total;
    printf("[LORA-RX-REL] IMAGE_FEEDBACK from=%u forOrigin=%u image=%u round=%u %s",
           (unsigned)fb.responder_device_id, (unsigned)fb.source_device_id,
           (unsigned)fb.image_id, (unsigned)fb.round,
           (fb.flags & DF_IMAGE_FEEDBACK_FLAG_COMPLETE) ? "ACK" : "NACK");
    if (!(fb.flags & DF_IMAGE_FEEDBACK_FLAG_COMPLETE)) {
        printf(" missing=%u", (unsigned)bitmap_count_raw(fb.missing_bitmap, fb.total_chunks));
    }
    printf("\n");
    fflush(stdout);
    feedback_inbox_push(link, &fb);
    return 0;
}

static int handle_event_ack(LoraUartLink *link, const uint8_t *frame, size_t frame_len) {
    DfEventAckDecoded ack;
    int ret = df_event_ack_decode(frame, frame_len, &ack);
    if (ret != 0) return ret;
    critical_event_ack_received(link, &ack);
    return 0;
}

static void schedule_event_ack_for_rx(LoraUartLink *link, const DfTelemetryDecoded *d) {
    uint8_t frame[DF_WIRE_MAX_FRAME_SIZE];
    size_t frame_len;
    if (!link || !d || d->source_device_id == link->device_id) return;
    if (link->chain_enabled && !chain_accept_downstream_origin_hop(link, d->source_device_id, d->hop_count))
        return;
    if (d->type != DF_WIRE_EVENT_START && d->type != DF_WIRE_EVENT_END) return;
    frame_len = df_event_ack_encode(link->device_id, d->source_device_id,
                                    d->sequence, d->event_id, d->type,
                                    frame, sizeof(frame));
    if (frame_len == 0U) return;
    (void)schedule_control_frame(link, LORA_CONTROL_EVENT_ACK,
                                 event_ack_due_ms(link, d->source_device_id, d->sequence, (uint8_t)d->type),
                                 frame, frame_len, d->source_device_id, d->sequence,
                                 0U, (uint8_t)d->type, 0U,
                                 link->chain_enabled ? DF_LORA_CHAIN_EVENT_ACK_COPIES : 1U,
                                 link->chain_enabled ? DF_LORA_CHAIN_EVENT_ACK_REPEAT_MS : 0U);
}

static void parse_rx_frames(LoraUartLink *link) {
    while (link->rx_size >= 2U) {
        size_t i;
        size_t frame_len = 0U;
        int peek_ret;
        int decode_ret = -99;

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
                if (link->chain_enabled && hb.source_device_id != chain_prev_id(link)) {
                    ++link->chain_filtered_frames;
                    remove_rx_prefix(link, frame_len);
                    continue;
                }
                ++link->rx_ok;
                printf("[LORA-RX] dev=%u seq=%u type=HEARTBEAT uptime=%us txOK=%u "
                       "uart=%u auxBusy=%u auxFallback=%u\n",
                       (unsigned)hb.source_device_id, (unsigned)hb.sequence,
                       (unsigned)hb.uptime_s, (unsigned)hb.tx_ok,
                       (unsigned)((hb.flags & DF_HEARTBEAT_FLAG_UART_OPEN) ? 1U : 0U),
                       (unsigned)((hb.flags & DF_HEARTBEAT_FLAG_AUX_HIGH) ? 1U : 0U),
                       (unsigned)((hb.flags & DF_HEARTBEAT_FLAG_AUX_ADVISORY) ? 1U : 0U));
                fflush(stdout);
                remove_rx_prefix(link, frame_len);
                continue;
            }
        } else if (link->rx_buffer[3] == (uint8_t)DF_WIRE_IMAGE_META) {
            decode_ret = handle_image_meta(link, link->rx_buffer, frame_len);
            if (decode_ret == 0) {
                ++link->rx_ok;
                remove_rx_prefix(link, frame_len);
                continue;
            }
        } else if (link->rx_buffer[3] == (uint8_t)DF_WIRE_IMAGE_CHUNK) {
            decode_ret = handle_image_chunk(link, link->rx_buffer, frame_len);
            if (decode_ret == 0) {
                ++link->rx_ok;
                remove_rx_prefix(link, frame_len);
                continue;
            }
        } else if (link->rx_buffer[3] == (uint8_t)DF_WIRE_IMAGE_DONE) {
            decode_ret = handle_image_done(link, link->rx_buffer, frame_len);
            if (decode_ret == 0) {
                ++link->rx_ok;
                remove_rx_prefix(link, frame_len);
                continue;
            }
        } else if (link->rx_buffer[3] == (uint8_t)DF_WIRE_IMAGE_FEEDBACK) {
            decode_ret = handle_image_feedback(link, link->rx_buffer, frame_len);
            if (decode_ret == 0) {
                ++link->rx_ok;
                remove_rx_prefix(link, frame_len);
                continue;
            }
        } else if (link->rx_buffer[3] == (uint8_t)DF_WIRE_EVENT_ACK) {
            decode_ret = handle_event_ack(link, link->rx_buffer, frame_len);
            if (decode_ret == 0) {
                ++link->rx_ok;
                remove_rx_prefix(link, frame_len);
                continue;
            }
        } else if (link->rx_buffer[3] >= (uint8_t)DF_WIRE_EVENT_START &&
                   link->rx_buffer[3] <= (uint8_t)DF_WIRE_EVENT_END) {
            DfTelemetryDecoded d;
            decode_ret = df_telemetry_decode(link->rx_buffer, frame_len, &d);
            if (decode_ret == 0) {
                if (d.source_device_id == link->device_id ||
                    !chain_accept_downstream_origin_hop(link, d.source_device_id, d.hop_count)) {
                    if (link->chain_enabled) ++link->chain_filtered_frames;
                    remove_rx_prefix(link, frame_len);
                    continue;
                }
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
                schedule_event_ack_for_rx(link, &d);
                relay_consider_valid_frame(link, link->rx_buffer, frame_len);
                remove_rx_prefix(link, frame_len);
                continue;
            }
        }
        ++link->rx_bad;
        fprintf(stderr, "[LORA-RX] bad frame type=0x%02x decode=%d buffered=%zu\n",
                (unsigned)link->rx_buffer[3], decode_ret, link->rx_size);
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


static void expire_image_rx_slots(LoraUartLink *link) {
    size_t i;
    const uint64_t now = monotonic_ms();
    for (i = 0U; i < DF_LORA_IMAGE_RX_SLOT_COUNT; ++i) {
        LoraImageRxSlot *slot = &link->image_rx[i];
        if (!slot->active || now <= slot->last_activity_ms) continue;
        if (slot->complete) {
            if (now - slot->last_activity_ms > DF_LORA_IMAGE_COMPLETE_HOLD_MS)
                slot->active = false;
        } else if (now - slot->last_activity_ms > DF_LORA_IMAGE_RX_TIMEOUT_MS) {
            ++link->rx_images_incomplete;
            fprintf(stderr,
                    "[LORA-RX-IMAGE] timeout incomplete dev=%u image=%u received=%u/%u\n",
                    (unsigned)slot->source_device_id, (unsigned)slot->image_id,
                    (unsigned)slot->received_chunks, (unsigned)slot->total_chunks);
            slot->active = false;
        }
    }
}

static bool any_image_rx_active(const LoraUartLink *link) {
    size_t i;
    for (i = 0U; i < DF_LORA_IMAGE_RX_SLOT_COUNT; ++i)
        if (link->image_rx[i].active && !link->image_rx[i].complete) return true;
    return false;
}

/* Bulk-image arbitration for four broadcast peers: if this node is currently
 * receiving an incomplete image from a lower device-id, let that lower-id
 * transfer finish/repair first.  Every node still transmits its own data; this
 * only serializes simultaneous bulk images when their metas overlap. */
static bool defer_local_image_for_rx(const LoraUartLink *link) {
    size_t i;
    for (i = 0U; i < DF_LORA_IMAGE_RX_SLOT_COUNT; ++i) {
        if (link->image_rx[i].active && !link->image_rx[i].complete &&
            link->image_rx[i].source_device_id < link->device_id)
            return true;
    }
    return false;
}

static int send_frame_bytes(LoraUartLink *link,
                            const uint8_t *frame,
                            size_t frame_len,
                            const char *tag) {
    /* Hardware pacing: never feed a new frame while AUX says RF is busy. */
    if (wait_aux_idle_low(link, DF_LORA_AUX_PRE_TX_TIMEOUT_MS, "pre-tx") != 0)
        return 1; /* not written; caller may retry the same frame later */

    if (write_all(link->fd, frame, frame_len) == (ssize_t)frame_len) {
        if (tcdrain(link->fd) != 0)
            fprintf(stderr, "[LORA-TX] tcdrain warning %s: %s\n", tag ? tag : "frame", strerror(errno));
        ++link->tx_ok;

        /* For IMAGE_META/IMAGE_CHUNK this replaces the former fixed usleep():
         * the next chunk is released only after AUX reports LOW/complete. */
        wait_aux_tx_complete(link, tag);
        return 0;
    }
    ++link->tx_fail;
    fprintf(stderr, "[LORA-TX] UART write failed %s: %s\n", tag ? tag : "frame", strerror(errno));
    close_uart(link);
    return -1;
}

static bool send_due_control(LoraUartLink *link) {
    size_t i, chosen = SIZE_MAX;
    uint64_t chosen_due = UINT64_MAX;
    uint64_t now = monotonic_ms();
    int ret;
    for (i = 0U; i < DF_LORA_RELIABILITY_CONTROL_QUEUE_CAPACITY; ++i) {
        LoraScheduledControl *c = &link->control_tx[i];
        if (c->active && c->due_ms <= now && c->due_ms < chosen_due) {
            chosen = i;
            chosen_due = c->due_ms;
        }
    }
    if (chosen == SIZE_MAX) return false;
    ret = send_frame_bytes(link, link->control_tx[chosen].frame,
                           link->control_tx[chosen].frame_len,
                           link->control_tx[chosen].kind == LORA_CONTROL_IMAGE_FEEDBACK ?
                           "IMAGE_FEEDBACK" : "EVENT_ACK");
    if (ret == 0) {
        LoraScheduledControl *c = &link->control_tx[chosen];
        uint8_t copy_no;
        if (c->copies_total == 0U) c->copies_total = 1U;
        ++c->copies_sent;
        copy_no = c->copies_sent;
        if (c->kind == LORA_CONTROL_IMAGE_FEEDBACK) {
            DfImageFeedbackDecoded fb;
            if (df_image_feedback_decode(c->frame, c->frame_len, &fb) == 0) {
                if (fb.flags & DF_IMAGE_FEEDBACK_FLAG_COMPLETE) {
                    ++link->image_feedback_ack_tx;
                    printf("[LORA-TX-REL] IMAGE_ACK dev=%u forDev=%u image=%u round=%u copy=%u/%u status=OK\n",
                           (unsigned)link->device_id, (unsigned)fb.source_device_id,
                           (unsigned)fb.image_id, (unsigned)fb.round,
                           (unsigned)copy_no, (unsigned)c->copies_total);
                } else {
                    ++link->image_feedback_nack_tx;
                    printf("[LORA-TX-REL] IMAGE_NACK dev=%u forDev=%u image=%u round=%u missing=%u copy=%u/%u status=OK\n",
                           (unsigned)link->device_id, (unsigned)fb.source_device_id,
                           (unsigned)fb.image_id, (unsigned)fb.round,
                           (unsigned)bitmap_count_raw(fb.missing_bitmap, fb.total_chunks),
                           (unsigned)copy_no, (unsigned)c->copies_total);
                }
                fflush(stdout);
            }
        } else if (c->kind == LORA_CONTROL_EVENT_ACK) {
            DfEventAckDecoded ack;
            if (df_event_ack_decode(c->frame, c->frame_len, &ack) == 0) {
                ++link->event_ack_tx;
                printf("[LORA-TX-REL] EVENT_ACK dev=%u forDev=%u seq=%u event=%u type=%s copy=%u/%u status=OK\n",
                       (unsigned)link->device_id, (unsigned)ack.source_device_id,
                       (unsigned)ack.sequence, (unsigned)ack.event_id,
                       df_wire_packet_type_name((DfWirePacketType)ack.event_type),
                       (unsigned)copy_no, (unsigned)c->copies_total);
                fflush(stdout);
            }
        }
        if (c->copies_sent < c->copies_total && c->repeat_interval_ms > 0U) {
            c->due_ms = monotonic_ms() + c->repeat_interval_ms;
            if (link->chain_enabled) ++link->chain_feedback_repeat_tx;
        } else {
            c->active = false;
        }
    }
    return true;
}

static bool process_critical_event_retries(LoraUartLink *link) {
    size_t i;
    uint64_t now = monotonic_ms();
    for (i = 0U; i < DF_LORA_CRITICAL_EVENT_PENDING_CAPACITY; ++i) {
        LoraCriticalEventPending *p = &link->critical_event_tx[i];
        int ret;
        if (!p->active || now < p->next_retry_ms) continue;
        if (p->retries >= DF_LORA_EVENT_ACK_MAX_RETRIES) {
            ++link->critical_event_unconfirmed;
            fprintf(stderr,
                    "[LORA-REL] critical event unconfirmed origin=%u seq=%u event=%u type=%s retries=%u\n",
                    (unsigned)p->source_device_id, (unsigned)p->sequence, (unsigned)p->event_id,
                    df_wire_packet_type_name((DfWirePacketType)p->event_type),
                    (unsigned)p->retries);
            p->active = false;
            continue;
        }
        ret = send_frame_bytes(link, p->frame, p->frame_len, "EVENT_RETRY");
        if (ret == 0) {
            uint64_t sent_ms = monotonic_ms();
            ++p->retries;
            ++link->critical_event_retries;
            p->next_retry_ms = sent_ms + DF_LORA_EVENT_ACK_RETRY_MS;
            p->quiet_until_ms = link->chain_enabled
                ? sent_ms + DF_LORA_CHAIN_EVENT_SENDER_QUIET_MS : 0U;
            printf("[LORA-TX-REL] retry origin=%u seq=%u event=%u type=%s attempt=%u/%u status=OK\n",
                   (unsigned)p->source_device_id, (unsigned)p->sequence, (unsigned)p->event_id,
                   df_wire_packet_type_name((DfWirePacketType)p->event_type),
                   (unsigned)p->retries, (unsigned)DF_LORA_EVENT_ACK_MAX_RETRIES);
            fflush(stdout);
        }
        return true;
    }
    return false;
}

static void image_tx_session_start(LoraImageTxSession *tx, const LoraImageJob *job) {
    memset(tx, 0, sizeof(*tx));
    tx->active = true;
    tx->image = *job;
    tx->phase = IMAGE_TX_PHASE_META;
    tx->total_chunks = (uint16_t)((job->jpeg_size + DF_IMAGE_CHUNK_DATA_MAX - 1U) /
                                  DF_IMAGE_CHUNK_DATA_MAX);
}

static void image_tx_session_finish(LoraUartLink *link,
                                    LoraImageTxSession *tx,
                                    bool confirmed,
                                    const char *reason) {
    if (!link || !tx || !tx->active) return;
    bool gateway_no_ack = reason && strcmp(reason, "gateway-facing-no-ack") == 0;
    ++link->tx_images_ok;
    if (gateway_no_ack) ++link->tx_images_gateway_sent;
    else if (confirmed) ++link->tx_images_confirmed;
    else ++link->tx_images_unconfirmed;
    printf("[LORA-TX-IMAGE] txDev=%u origin=%u image=%u event=%llu purpose=%s hop=%u relayed=%u chunks=%u "
           "status=%s reason=%s repairRounds=%u\n",
           (unsigned)link->device_id, (unsigned)tx->image.source_device_id,
           (unsigned)tx->image.image_id, (unsigned long long)tx->image.event_id,
           image_purpose_name(tx->image.purpose), (unsigned)tx->image.hop_count,
           (unsigned)(tx->image.relayed ? 1U : 0U), (unsigned)tx->total_chunks,
           gateway_no_ack ? "SENT_NO_ACK" : (confirmed ? "CONFIRMED" : "UNCONFIRMED"),
           reason ? reason : "unknown", (unsigned)tx->round);
    fflush(stdout);
    memset(tx, 0, sizeof(*tx));
    set_image_in_flight(link, false);
}

static void image_tx_consume_feedback(LoraUartLink *link, LoraImageTxSession *tx) {
    DfImageFeedbackDecoded fb;
    size_t bitmap_bytes;
    while (feedback_inbox_pop(link, &fb)) {
        if (!tx->active || tx->phase != IMAGE_TX_PHASE_WAIT_FEEDBACK ||
            fb.source_device_id != tx->image.source_device_id || fb.image_id != tx->image.image_id ||
            fb.event_id != (uint32_t)(tx->image.event_id > 0xffffffffULL ? 0xffffffffULL : tx->image.event_id) ||
            fb.total_chunks != tx->total_chunks || fb.round != tx->round) {
            continue;
        }
        tx->feedback_seen = true;
        if (fb.flags & DF_IMAGE_FEEDBACK_FLAG_COMPLETE) {
            tx->ack_seen = true;
        } else {
            bitmap_bytes = (size_t)((tx->total_chunks + 7U) / 8U);
            bitmap_or_raw(tx->missing_union, fb.missing_bitmap, bitmap_bytes);
        }
    }
}

static void image_tx_evaluate_feedback(LoraUartLink *link, LoraImageTxSession *tx) {
    size_t bitmap_bytes;
    bool missing;
    uint16_t missing_count;
    uint64_t now;
    if (!tx->active || tx->phase != IMAGE_TX_PHASE_WAIT_FEEDBACK) return;
    now = monotonic_ms();
    /* In an ordered chain there is exactly one legal downstream responder.
     * As soon as one ACK/NACK has been heard and the protected turnaround
     * window has elapsed, act on it instead of waiting the full broadcast
     * aggregation window.  If nothing was heard, keep the normal deadline. */
    if (link->chain_enabled && tx->feedback_seen) {
        if (now < tx->feedback_quiet_until_ms) return;
    } else if (now < tx->feedback_deadline_ms) {
        return;
    }
    bitmap_bytes = (size_t)((tx->total_chunks + 7U) / 8U);
    missing = bitmap_any_raw(tx->missing_union, bitmap_bytes);
    missing_count = bitmap_count_raw(tx->missing_union, tx->total_chunks);
    if (missing) {
        if (tx->round < DF_LORA_IMAGE_MAX_REPAIR_ROUNDS) {
            memcpy(tx->repair_bitmap, tx->missing_union, bitmap_bytes);
            ++tx->round;
            ++link->tx_image_repair_rounds;
            tx->phase = IMAGE_TX_PHASE_REPAIR;
            tx->next_chunk = 0U;
            tx->no_feedback_retries = 0U;
            tx->feedback_seen = false;
            tx->ack_seen = false;
            tx->feedback_quiet_until_ms = 0U;
            bitmap_zero(tx->missing_union, sizeof(tx->missing_union));
            printf("[LORA-TX-REL] image=%u repairRound=%u missingUnion=%u\n",
                   (unsigned)tx->image.image_id, (unsigned)tx->round,
                   (unsigned)missing_count);
            fflush(stdout);
            return;
        }
        image_tx_session_finish(link, tx, tx->ack_seen,
                                tx->ack_seen ? "max-repair-partial-ack" : "max-repair-no-complete-ack");
        return;
    }
    if (tx->ack_seen) {
        image_tx_session_finish(link, tx, true,
                                link->chain_enabled ? "hop-ack" : "broadcast-ack");
        return;
    }
    if (!tx->feedback_seen) {
        if (tx->no_feedback_retries == 0U) {
            ++tx->no_feedback_retries;
            tx->phase = IMAGE_TX_PHASE_DONE;
            tx->feedback_quiet_until_ms = 0U;
            printf("[LORA-TX-REL] image=%u round=%u no feedback; retry DONE only\n",
                   (unsigned)tx->image.image_id, (unsigned)tx->round);
            fflush(stdout);
            return;
        }
        if (tx->no_feedback_retries < DF_LORA_IMAGE_NO_FEEDBACK_RETRIES) {
            ++tx->no_feedback_retries;
            tx->phase = IMAGE_TX_PHASE_META;
            tx->next_chunk = 0U;
            tx->feedback_quiet_until_ms = 0U;
            tx->meta_refresh_needed = false;
            printf("[LORA-TX-REL] image=%u round=%u still no feedback; full META+CHUNK resend %u/%u\n",
                   (unsigned)tx->image.image_id, (unsigned)tx->round,
                   (unsigned)tx->no_feedback_retries,
                   (unsigned)DF_LORA_IMAGE_NO_FEEDBACK_RETRIES);
            fflush(stdout);
            return;
        }
        image_tx_session_finish(link, tx, false, "no-feedback-after-full-resend");
        return;
    }
    image_tx_session_finish(link, tx, false, "feedback-without-ack-or-missing");
}

static int image_tx_send_meta(LoraUartLink *link,
                              LoraImageTxSession *tx,
                              uint8_t *frame,
                              size_t frame_capacity,
                              bool duplicate) {
    size_t frame_len = df_image_meta_encode(tx->image.source_device_id, tx->image.image_id,
                                            (uint32_t)(tx->image.event_id > 0xffffffffULL ? 0xffffffffULL : tx->image.event_id),
                                            tx->image.capture_epoch_ms,
                                            tx->image.width, tx->image.height,
                                            (uint32_t)tx->image.jpeg_size,
                                            tx->total_chunks,
                                            tx->image.quality,
                                            (DfImagePurpose)tx->image.purpose,
                                            frame, frame_capacity);
    if (frame_len == 0U) return -1;
    if (tx->image.hop_count > 0U &&
        df_wire_set_hop_count(frame, frame_len, tx->image.hop_count) != 0) return -1;
    if (send_frame_bytes(link, frame, frame_len, "IMAGE_META") != 0) return 1;
    printf("[LORA-TX-IMAGE] txDev=%u origin=%u image=%u type=%s event=%llu purpose=%s hop=%u size=%zu "
           "%ux%u q=%u chunks=%u status=OK\n",
           (unsigned)link->device_id, (unsigned)tx->image.source_device_id,
           (unsigned)tx->image.image_id,
           duplicate ? "IMAGE_META_REFRESH" : "IMAGE_META",
           (unsigned long long)tx->image.event_id, image_purpose_name(tx->image.purpose),
           (unsigned)tx->image.hop_count, tx->image.jpeg_size,
           (unsigned)tx->image.width, (unsigned)tx->image.height,
           (unsigned)tx->image.quality, (unsigned)tx->total_chunks);
    fflush(stdout);
    return 0;
}

static int image_tx_send_chunk(LoraUartLink *link,
                               LoraImageTxSession *tx,
                               uint16_t index,
                               bool repair,
                               uint8_t *frame,
                               size_t frame_capacity) {
    size_t offset = (size_t)index * DF_IMAGE_CHUNK_DATA_MAX;
    size_t remain = tx->image.jpeg_size - offset;
    uint8_t data_len = (uint8_t)(remain > DF_IMAGE_CHUNK_DATA_MAX ?
                                 DF_IMAGE_CHUNK_DATA_MAX : remain);
    size_t frame_len = df_image_chunk_encode(tx->image.source_device_id, tx->image.image_id,
                                             (uint32_t)(tx->image.event_id > 0xffffffffULL ? 0xffffffffULL : tx->image.event_id),
                                             index, tx->total_chunks,
                                             (DfImagePurpose)tx->image.purpose,
                                             tx->image.jpeg + offset, data_len,
                                             frame, frame_capacity);
    if (frame_len == 0U) return -1;
    if (send_frame_bytes(link, frame, frame_len, repair ? "IMAGE_REPAIR" : "IMAGE_CHUNK") != 0) return 1;
    if (repair) ++link->tx_image_repair_chunks;
    if (repair || index == 0U || index + 1U == tx->total_chunks || ((index + 1U) % 10U) == 0U) {
        printf("[LORA-TX-IMAGE] txDev=%u origin=%u image=%u %schunk=%u/%u bytes=%u round=%u status=OK\n",
               (unsigned)link->device_id, (unsigned)tx->image.source_device_id,
               (unsigned)tx->image.image_id,
               repair ? "repair-" : "",
               (unsigned)(index + 1U), (unsigned)tx->total_chunks,
               (unsigned)data_len, (unsigned)tx->round);
        fflush(stdout);
    }
    return 0;
}

static int image_tx_send_done(LoraUartLink *link,
                              LoraImageTxSession *tx,
                              uint8_t *frame,
                              size_t frame_capacity) {
    size_t frame_len = df_image_done_encode(tx->image.source_device_id, tx->image.image_id,
                                            (uint32_t)(tx->image.event_id > 0xffffffffULL ? 0xffffffffULL : tx->image.event_id),
                                            tx->total_chunks, tx->round,
                                            (DfImagePurpose)tx->image.purpose,
                                            frame, frame_capacity);
    if (frame_len == 0U) return -1;
    if (send_frame_bytes(link, frame, frame_len, "IMAGE_DONE") != 0) return 1;
    if (!tx->image.expect_feedback) {
        printf("[LORA-TX-REL] txDev=%u origin=%u image=%u type=IMAGE_DONE round=%u gatewayFacing=1 status=OK\n",
               (unsigned)link->device_id, (unsigned)tx->image.source_device_id,
               (unsigned)tx->image.image_id, (unsigned)tx->round);
        fflush(stdout);
        image_tx_session_finish(link, tx, false, "gateway-facing-no-ack");
        return 0;
    }
    bitmap_zero(tx->missing_union, sizeof(tx->missing_union));
    tx->feedback_seen = false;
    tx->ack_seen = false;
    {
        uint64_t now = monotonic_ms();
        tx->feedback_deadline_ms = now + DF_LORA_IMAGE_FEEDBACK_WINDOW_MS;
        tx->feedback_quiet_until_ms = link->chain_enabled
            ? now + DF_LORA_CHAIN_IMAGE_SENDER_QUIET_MS : now;
    }
    tx->phase = IMAGE_TX_PHASE_WAIT_FEEDBACK;
    printf("[LORA-TX-REL] txDev=%u origin=%u image=%u type=IMAGE_DONE round=%u waitFeedback=%ums quiet=%ums status=OK\n",
           (unsigned)link->device_id, (unsigned)tx->image.source_device_id,
           (unsigned)tx->image.image_id, (unsigned)tx->round,
           (unsigned)DF_LORA_IMAGE_FEEDBACK_WINDOW_MS,
           (unsigned)(link->chain_enabled ? DF_LORA_CHAIN_IMAGE_SENDER_QUIET_MS : 0U));
    fflush(stdout);
    return 0;
}

static void poll_receive_once(LoraUartLink *link, int timeout_ms) {
    struct pollfd pfd;
    int pr;
    if (!link || link->fd < 0) return;
    pfd.fd = link->fd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    pr = poll(&pfd, 1, timeout_ms);
    if (pr > 0 && (pfd.revents & POLLIN)) receive_uart(link);
    else if (pr < 0 && errno != EINTR) {
        fprintf(stderr, "[LORA] UART poll error: %s\n", strerror(errno));
        close_uart(link);
    }
}

static void *lora_worker(void *arg) {
    LoraUartLink *link = (LoraUartLink *)arg;
    MonitorMessage pending;
    bool pending_valid = false;
    LoraImageTxSession image_tx;
    bool gpio_attempted = false;
    const uint64_t worker_start_ms = monotonic_ms();
    uint64_t next_heartbeat_ms = link->heartbeat_interval_s > 0U
        ? worker_start_ms + (uint64_t)link->heartbeat_interval_s * 1000ULL : 0ULL;
    memset(&image_tx, 0, sizeof(image_tx));

    while (!link_should_stop(link)) {
        uint8_t frame[DF_WIRE_MAX_FRAME_SIZE];
        size_t frame_len;
        bool defer_image;

        expire_image_rx_slots(link);

        if (!gpio_attempted) {
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

        /* Drain received broadcast traffic before deciding whether this node may TX. */
        poll_receive_once(link, 0);
        image_tx_consume_feedback(link, &image_tx);
        image_tx_evaluate_feedback(link, &image_tx);

        {
            const uint64_t now = monotonic_ms();
            const bool rx_bulk = any_image_rx_active(link);
            const bool image_feedback_quiet = link->chain_enabled && image_tx.active &&
                image_tx.phase == IMAGE_TX_PHASE_WAIT_FEEDBACK &&
                now < image_tx.feedback_quiet_until_ms;
            const bool event_ack_quiet = critical_event_quiet_active(link);
            const bool ordinary_tx_quiet = image_feedback_quiet || event_ack_quiet || rx_bulk;

            /* ACK/NACK generated for the upstream hop is the highest MAC-control priority.
             * It is allowed even while this node is otherwise quiet. */
            if (send_due_control(link)) {
                poll_receive_once(link, 5);
                continue;
            }

            /* START/END forwarding is still allowed to preempt a protected window: alarm
             * boundaries outrank JPEG reliability.  UPDATE relays are held below. */
            if (send_due_relay(link, (uint8_t)LORA_RELAY_PRIORITY_EVENT)) {
                poll_receive_once(link, 5);
                continue;
            }

            /* Do not inject an old START/END retry into the first IMAGE feedback-turnaround
             * window.  The original critical frame was already sent and can wait <=1.2 s. */
            if (!image_feedback_quiet && process_critical_event_retries(link)) {
                poll_receive_once(link, 5);
                continue;
            }

            /* RX-BULK and ACK-turnaround guards suppress ordinary UPDATE traffic.  A newly
             * generated START/END is still returned by queue_pop_prioritized(..., false). */
            if (!pending_valid) {
                bool allow_update = !ordinary_tx_quiet &&
                    !(image_tx.active && image_tx.phase == IMAGE_TX_PHASE_WAIT_FEEDBACK);
                pending_valid = queue_pop_prioritized(link, &pending, allow_update);
                if (pending_valid) set_in_flight(link, true);
            }

            /* A previously selected UPDATE may survive an AUX-busy retry; do not let it
             * leak into a newly opened quiet/RX-BULK window. */
            if (pending_valid && ordinary_tx_quiet && pending.type == MONITOR_EVENT_UPDATE) {
                poll_receive_once(link, image_feedback_quiet ? 20 : 10);
                continue;
            }

        if (pending_valid) {
            frame_len = df_telemetry_encode(&pending, link->device_id, frame, sizeof(frame));
            if (frame_len == 0U) {
                fprintf(stderr, "[LORA-TX] unsupported message seq=%llu type=%d\n",
                        (unsigned long long)pending.sequence, (int)pending.type);
                pending_valid = false;
                set_in_flight(link, false);
            } else {
                int send_ret = send_frame_bytes(link, frame, frame_len, "EVENT");
                if (send_ret == 0) {
                    uint32_t seq32 = (uint32_t)(pending.sequence > 0xffffffffULL ? 0xffffffffULL : pending.sequence);
                    uint32_t event32 = (uint32_t)(pending.event.event_id > 0xffffffffULL ? 0xffffffffULL : pending.event.event_id);
                    printf("[LORA-TX] dev=%u seq=%llu type=%s event=%llu bytes=%zu status=OK\n",
                           (unsigned)link->device_id,
                           (unsigned long long)pending.sequence,
                           df_wire_packet_type_name((DfWirePacketType)frame[3]),
                           (unsigned long long)pending.event.event_id,
                           frame_len);
                    fflush(stdout);
                    if ((frame[3] == (uint8_t)DF_WIRE_EVENT_START || frame[3] == (uint8_t)DF_WIRE_EVENT_END) &&
                        (!link->chain_enabled || chain_next_id(link) != 0U))
                        register_critical_event_tx(link, frame, frame_len, link->device_id,
                                                   seq32, event32, frame[3]);
                    pending_valid = false;
                    set_in_flight(link, false);
                    poll_receive_once(link, 5);
                    continue;
                }
                poll_receive_once(link, 5);
                continue;
            }
        }

            /* If no critical frame was selected, preserve the protected half-duplex window. */
            if (!pending_valid && ordinary_tx_quiet) {
                poll_receive_once(link, image_feedback_quiet ? 20 : 10);
                continue;
            }
        }

        /* Low-priority relays (UPDATE/HEARTBEAT/IMAGE_CHUNK) yield to this
         * monitor's START/END/UPDATE but run before local heartbeat/image bulk. */
        if (!pending_valid && send_due_relay(link, (uint8_t)LORA_RELAY_PRIORITY_BULK)) {
            poll_receive_once(link, 5);
            continue;
        }

        /* Heartbeats are best-effort and never interrupt an image/repair session. */
        if (!pending_valid && !image_tx.active && link->fd >= 0 && link->heartbeat_interval_s > 0U &&
            !any_image_rx_active(link) && monotonic_ms() >= next_heartbeat_ms) {
            uint8_t hb_flags = DF_HEARTBEAT_FLAG_UART_OPEN;
            int aux_value = link->aux_ready ? gpio_read_one(link->gpio_aux) : -1;
            if (aux_value == 1) hb_flags |= DF_HEARTBEAT_FLAG_AUX_HIGH;
            if (link->aux_advisory_only) hb_flags |= DF_HEARTBEAT_FLAG_AUX_ADVISORY;
            ++link->heartbeat_sequence;
            frame_len = df_heartbeat_encode(link->device_id, link->heartbeat_sequence,
                                            (uint32_t)((monotonic_ms() - worker_start_ms) / 1000ULL),
                                            (uint32_t)(link->tx_ok > 0xffffffffULL ? 0xffffffffULL : link->tx_ok),
                                            hb_flags, frame, sizeof(frame));
            if (frame_len > 0U && send_frame_bytes(link, frame, frame_len, "HEARTBEAT") == 0) {
                printf("[LORA-TX] dev=%u hbSeq=%u type=HEARTBEAT uptime=%llus bytes=%zu status=OK\n",
                       (unsigned)link->device_id, (unsigned)link->heartbeat_sequence,
                       (unsigned long long)((monotonic_ms() - worker_start_ms) / 1000ULL), frame_len);
                fflush(stdout);
            }
            next_heartbeat_ms = monotonic_ms() + (uint64_t)link->heartbeat_interval_s * 1000ULL;
            poll_receive_once(link, 5);
            continue;
        }

        if (!image_tx.active && !pending_valid) {
            LoraImageJob job;
            if (image_queue_pop(link, &job)) {
                image_tx_session_start(&image_tx, &job);
                set_image_in_flight(link, true);
            }
        }

        if (image_tx.active && image_tx.phase != IMAGE_TX_PHASE_WAIT_FEEDBACK && link->fd >= 0) {
            defer_image = defer_local_image_for_rx(link);
            if (defer_image) {
                if (image_tx.phase != IMAGE_TX_PHASE_META) image_tx.meta_refresh_needed = true;
                image_tx.was_deferred = true;
            } else {
                if (image_tx.was_deferred) image_tx.was_deferred = false;
                if (image_tx.meta_refresh_needed && image_tx.phase != IMAGE_TX_PHASE_META) {
                    int ret = image_tx_send_meta(link, &image_tx, frame, sizeof(frame), true);
                    if (ret == 0) image_tx.meta_refresh_needed = false;
                    poll_receive_once(link, 5);
                    continue;
                }
                if (image_tx.phase == IMAGE_TX_PHASE_META) {
                    int ret = image_tx_send_meta(link, &image_tx, frame, sizeof(frame), false);
                    if (ret == 0) {
                        image_tx.phase = IMAGE_TX_PHASE_INITIAL;
                        image_tx.next_chunk = 0U;
                    }
                    poll_receive_once(link, 5);
                    continue;
                }
                if (image_tx.phase == IMAGE_TX_PHASE_INITIAL) {
                    uint16_t index = image_tx.next_chunk;
                    int ret = image_tx_send_chunk(link, &image_tx, index, false, frame, sizeof(frame));
                    if (ret == 0) {
                        ++image_tx.next_chunk;
                        if (image_tx.next_chunk >= image_tx.total_chunks)
                            image_tx.phase = IMAGE_TX_PHASE_DONE;
                    }
                    poll_receive_once(link, 5);
                    continue;
                }
                if (image_tx.phase == IMAGE_TX_PHASE_REPAIR) {
                    uint16_t index = image_tx.next_chunk;
                    while (index < image_tx.total_chunks && !bitmap_test_raw(image_tx.repair_bitmap, index))
                        ++index;
                    if (index >= image_tx.total_chunks) {
                        bitmap_zero(image_tx.repair_bitmap, sizeof(image_tx.repair_bitmap));
                        image_tx.phase = IMAGE_TX_PHASE_DONE;
                        image_tx.next_chunk = 0U;
                    } else {
                        int ret = image_tx_send_chunk(link, &image_tx, index, true, frame, sizeof(frame));
                        if (ret == 0) image_tx.next_chunk = (uint16_t)(index + 1U);
                        poll_receive_once(link, 5);
                        continue;
                    }
                }
                if (image_tx.phase == IMAGE_TX_PHASE_DONE) {
                    (void)image_tx_send_done(link, &image_tx, frame, sizeof(frame));
                    poll_receive_once(link, 5);
                    continue;
                }
            }
        }

        poll_receive_once(link, image_tx.active ? 20 : 50);
    }

    if (image_tx.active) {
        ++link->tx_images_unconfirmed;
        set_image_in_flight(link, false);
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
                   unsigned heartbeat_interval_s,
                   uint16_t chain_first_id,
                   uint16_t chain_last_id) {
    if (!link || !device_path || !device_path[0] || baud_to_termios(baud_rate) == (speed_t)0)
        return -1;
    memset(link, 0, sizeof(*link));
    snprintf(link->device_path, sizeof(link->device_path), "%s", device_path);
    link->baud_rate = baud_rate;
    link->device_id = device_id;
    link->chain_enabled = chain_first_id != 0U && chain_last_id != 0U;
    link->chain_first_id = chain_first_id;
    link->chain_last_id = chain_last_id;
    link->gpio_m0 = gpio_m0;
    link->gpio_m1 = gpio_m1;
    link->gpio_aux = gpio_aux;
    link->heartbeat_interval_s = heartbeat_interval_s;
    link->fd = -1;
    link->next_image_id = 1U;
    if (pthread_mutex_init(&link->queue_mutex, NULL) != 0) return -1;
    link->mutex_ready = true;
    return 0;
}

int lora_uart_start(LoraUartLink *link) {
    if (!link || !link->mutex_ready || link->worker_started) return -1;
    link->stop_requested = false;
    if (pthread_create(&link->worker_thread, NULL, lora_worker, link) != 0) return -1;
    link->worker_started = true;
    printf("[LORA] worker started device=%s baud=%d deviceId=%u txQueue=%u imageQueue=%u "
           "imageChunkData=%u M0=%d M1=%d AUX=%d auxFlow=LOW_IDLE/HIGH_BUSY "
           "reliability=HOP_NACK+EVENT_ACK mac=%s relay=%s chain=%u..%u prev=%u next=%u heartbeat=%us\n",
           link->device_path, link->baud_rate, (unsigned)link->device_id,
           (unsigned)DF_LORA_TX_QUEUE_CAPACITY, (unsigned)DF_LORA_IMAGE_QUEUE_CAPACITY,
           (unsigned)DF_IMAGE_CHUNK_DATA_MAX, link->gpio_m0, link->gpio_m1, link->gpio_aux,
           link->chain_enabled ? "QUIET+DUAL_FEEDBACK" : "BCAST",
           link->chain_enabled ? "ORDERED_STORE_FORWARD" : "OFF",
           (unsigned)link->chain_first_id, (unsigned)link->chain_last_id,
           (unsigned)chain_prev_id(link), (unsigned)chain_next_id(link),
           link->heartbeat_interval_s);
    fflush(stdout);
    if (link->chain_enabled) {
        printf("[LORA-MAC] chainFix=3 imageTurn=%ums imageCopies=%u imageRepeat=%ums imageQuiet=%ums "
               "eventTurn=%ums eventCopies=%u eventRepeat=%ums eventQuiet=%ums rxBulkGuard=1\n",
               (unsigned)DF_LORA_CHAIN_IMAGE_FEEDBACK_TURNAROUND_MS,
               (unsigned)DF_LORA_CHAIN_IMAGE_FEEDBACK_COPIES,
               (unsigned)DF_LORA_CHAIN_IMAGE_FEEDBACK_REPEAT_MS,
               (unsigned)DF_LORA_CHAIN_IMAGE_SENDER_QUIET_MS,
               (unsigned)DF_LORA_CHAIN_EVENT_ACK_TURNAROUND_MS,
               (unsigned)DF_LORA_CHAIN_EVENT_ACK_COPIES,
               (unsigned)DF_LORA_CHAIN_EVENT_ACK_REPEAT_MS,
               (unsigned)DF_LORA_CHAIN_EVENT_SENDER_QUIET_MS);
        fflush(stdout);
    }
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
        fprintf(stderr, "[LORA-QUEUE] telemetry overflow drop=%llu\n",
                (unsigned long long)link->dropped_messages);
    }

    index = link->tx_tail;
    link->tx_queue[index] = *message;
    link->tx_tail = (link->tx_tail + 1U) % DF_LORA_TX_QUEUE_CAPACITY;
    ++link->tx_count;
    pthread_mutex_unlock(&link->queue_mutex);
    return true;
}

bool lora_uart_enqueue_image(LoraUartLink *link,
                             uint64_t event_id,
                             uint64_t capture_epoch_ms,
                             DfImagePurpose purpose,
                             uint16_t width,
                             uint16_t height,
                             uint8_t quality,
                             const uint8_t *jpeg,
                             size_t jpeg_size,
                             uint32_t *image_id_out) {
    LoraImageJob job;
    size_t i;
    uint32_t image_id;
    if (image_id_out) *image_id_out = 0U;
    if (!link || !jpeg || jpeg_size == 0U || jpeg_size > DF_IMAGE_MAX_JPEG_BYTES ||
        width == 0U || height == 0U || !link->mutex_ready ||
        (purpose != DF_IMAGE_PURPOSE_EVENT && purpose != DF_IMAGE_PURPOSE_DEPLOY_PREVIEW))
        return false;

    memset(&job, 0, sizeof(job));
    pthread_mutex_lock(&link->queue_mutex);
    image_id = link->next_image_id++;
    if (link->next_image_id == 0U) link->next_image_id = 1U;
    job.source_device_id = link->device_id;
    job.hop_count = 0U;
    job.relayed = false;
    job.expect_feedback = !link->chain_enabled || chain_next_id(link) != 0U;
    job.not_before_ms = 0U;
    job.image_id = image_id;
    job.event_id = event_id;
    job.capture_epoch_ms = capture_epoch_ms;
    job.width = width;
    job.height = height;
    job.quality = quality;
    job.purpose = (uint8_t)purpose;
    job.jpeg_size = jpeg_size;
    memcpy(job.jpeg, jpeg, jpeg_size);

    if (purpose == DF_IMAGE_PURPOSE_DEPLOY_PREVIEW) {
        /* latest-wins for pending deployment previews */
        for (i = 0U; i < link->image_count; ++i) {
            if (link->image_queue[i].purpose == (uint8_t)DF_IMAGE_PURPOSE_DEPLOY_PREVIEW) {
                link->image_queue[i] = job;
                ++link->dropped_images;
                pthread_mutex_unlock(&link->queue_mutex);
                if (image_id_out) *image_id_out = image_id;
                printf("[LORA-IMG-QUEUE] replace preview image=%u size=%zu dropped=%llu\n",
                       (unsigned)image_id, jpeg_size, (unsigned long long)link->dropped_images);
                fflush(stdout);
                return true;
            }
        }
        if (link->image_count >= DF_LORA_IMAGE_QUEUE_CAPACITY) {
            ++link->dropped_images;
            pthread_mutex_unlock(&link->queue_mutex);
            fprintf(stderr, "[LORA-IMG-QUEUE] drop preview image=%u queue full dropped=%llu\n",
                    (unsigned)image_id, (unsigned long long)link->dropped_images);
            return false;
        }
    } else if (link->image_count >= DF_LORA_IMAGE_QUEUE_CAPACITY) {
        bool removed_preview = false;
        for (i = 0U; i < link->image_count; ++i) {
            if (link->image_queue[i].purpose == (uint8_t)DF_IMAGE_PURPOSE_DEPLOY_PREVIEW) {
                image_queue_remove_at_locked(link, i);
                removed_preview = true;
                break;
            }
        }
        if (!removed_preview) image_queue_remove_at_locked(link, 0U);
        ++link->dropped_images;
        fprintf(stderr, "[LORA-IMG-QUEUE] make room for event image=%u dropped=%llu\n",
                (unsigned)image_id, (unsigned long long)link->dropped_images);
    }

    link->image_queue[link->image_count++] = job;
    i = link->image_count;
    pthread_mutex_unlock(&link->queue_mutex);
    if (image_id_out) *image_id_out = image_id;
    printf("[LORA-IMG-QUEUE] image=%u event=%llu purpose=%s size=%zu q=%u pending=%zu\n",
           (unsigned)image_id, (unsigned long long)event_id,
           image_purpose_name((uint8_t)purpose), jpeg_size, (unsigned)quality,
           i);
    fflush(stdout);
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
    printf("[LORA] worker stopped txOK=%llu txFail=%llu rxOK=%llu rxBad=%llu telemetryDrop=%llu "
           "imageDrop=%llu txImages=%llu txImageConfirmed=%llu txImageUnconfirmed=%llu txImageGatewaySent=%llu "
           "rxImages=%llu rxImageIncomplete=%llu repairRounds=%llu repairChunks=%llu "
           "imgAckTx=%llu imgNackTx=%llu imgFeedbackRx=%llu eventAckTx=%llu eventAckRx=%llu "
           "eventRetries=%llu eventUnconfirmed=%llu relayQueued=%llu relayTx=%llu "
           "relaySuppressed=%llu relayDrop=%llu relayOwnIgnored=%llu chainFiltered=%llu chainImages=%llu "
           "feedbackRepeatTx=%llu auxTimeout=%llu auxBusyCycles=%llu auxNoBusyObserved=%llu\n",
           (unsigned long long)link->tx_ok,
           (unsigned long long)link->tx_fail,
           (unsigned long long)link->rx_ok,
           (unsigned long long)link->rx_bad,
           (unsigned long long)link->dropped_messages,
           (unsigned long long)link->dropped_images,
           (unsigned long long)link->tx_images_ok,
           (unsigned long long)link->tx_images_confirmed,
           (unsigned long long)link->tx_images_unconfirmed,
           (unsigned long long)link->tx_images_gateway_sent,
           (unsigned long long)link->rx_images_ok,
           (unsigned long long)link->rx_images_incomplete,
           (unsigned long long)link->tx_image_repair_rounds,
           (unsigned long long)link->tx_image_repair_chunks,
           (unsigned long long)link->image_feedback_ack_tx,
           (unsigned long long)link->image_feedback_nack_tx,
           (unsigned long long)link->image_feedback_rx_count_total,
           (unsigned long long)link->event_ack_tx,
           (unsigned long long)link->event_ack_rx,
           (unsigned long long)link->critical_event_retries,
           (unsigned long long)link->critical_event_unconfirmed,
           (unsigned long long)link->relay_queued,
           (unsigned long long)link->relay_tx_ok,
           (unsigned long long)link->relay_suppressed,
           (unsigned long long)link->relay_dropped,
           (unsigned long long)link->relay_own_ignored,
           (unsigned long long)link->chain_filtered_frames,
           (unsigned long long)link->chain_images_queued,
           (unsigned long long)link->chain_feedback_repeat_tx,
           (unsigned long long)link->aux_wait_timeouts,
           (unsigned long long)link->aux_busy_cycles,
           (unsigned long long)link->aux_no_busy_observed);
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
