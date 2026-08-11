/*
 * dtu_gps.c - Luckfox Pico Zero 通过 UART3 读取银尔达 M100MG-B1 DTU 定位数据
 *
 * 编译（在 PC 端 SDK 交叉编译环境中）:
 *   export PATH=<SDK>/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin:$PATH
 *   arm-rockchip830-linux-uclibcgnueabihf-gcc -O2 -o dtu_gps dtu_gps.c
 *
 * 使用:
 *   ./dtu_gps [--port /dev/ttyS3] [--baud 115200] [--interval 5]
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_PORT "/dev/ttyS3"
#define DEFAULT_BAUD 115200
#define DEFAULT_INTERVAL 5

static int g_verbose = 0;   /* --raw: 打印 DTU 原始返回 */

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 1200:    return B1200;
    case 2400:    return B2400;
    case 4800:    return B4800;
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    default:      return B115200;
    }
}

static int open_serial(const char *port, int baud)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "打开串口 %s 失败: %s\n", port, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    speed_t spd = baud_to_speed(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    tty.c_cflag &= ~PARENB;          /* 无校验 */
    tty.c_cflag &= ~CSTOPB;          /* 1 停止位 */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;              /* 8 数据位 */
    tty.c_cflag &= ~CRTSCTS;         /* 无硬件流控 */
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

/* 非阻塞读取，等待最多 timeout_ms 毫秒，返回读取字节数 */
static ssize_t read_serial(int fd, char *buf, size_t len, int timeout_ms)
{
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;

    size_t total = 0;
    time_t deadline = time(NULL) + timeout_ms / 1000 + 1;

    while (total < len - 1) {
        int rc = poll(&pfd, 1, timeout_ms);
        if (rc <= 0)
            break;
        ssize_t n = read(fd, buf + total, len - 1 - total);
        if (n > 0) {
            total += (size_t)n;
            if (time(NULL) > deadline)
                break;
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            break;
        }
    }
    buf[total] = '\0';
    return (ssize_t)total;
}

static void handle_line(const char *line)
{
    char tbuf[32];
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    strftime(tbuf, sizeof(tbuf), "%H:%M:%S", tm ? tm : gmtime(&now));

    if (strncmp(line, "config,gps,ok,", 14) == 0) {
        if (g_verbose)
            printf("%s | RAW: %s\n", tbuf, line);
        const char *p = line + 14;
        int fix = -1;
        char lon_type = 'E', lat_type = 'N';
        double lon = 0, lat = 0, speed_kmh = 0, speed_kn = 0;
        double alt = 0, ell = 0, course = 0;

        /* 扩展格式: 1,E,113.9739056,N,22.6927826,12.5,6.7,184,-3.6,90,time */
        int n = sscanf(p, "%d,%c,%lf,%c,%lf,%lf,%lf,%lf,%lf,%lf",
                       &fix, &lon_type, &lon, &lat_type, &lat,
                       &speed_kmh, &speed_kn, &alt, &ell, &course);
        (void)speed_kn;
        (void)ell;
        if (n < 5) {
            /* 简单格式: E,113.9739056,N,22.6927826 */
            n = sscanf(p, "%c,%lf,%c,%lf",
                       &lon_type, &lon, &lat_type, &lat);
            if (n == 4)
                fix = (lon != 0 || lat != 0) ? 1 : 0;
        }
        if (n >= 4)
            printf("%s | %s | 经度 %.6f | 纬度 %.6f | 速度 %.1f km/h | 海拔 %.1f m\n",
                   tbuf, fix ? "已定位" : "未定位",
                   lon_type == 'W' ? -lon : lon,
                   lat_type == 'S' ? -lat : lat,
                   speed_kmh, alt);
        return;
    }

    /* NMEA 语句直通 */
    if (line[0] == '$') {
        if (g_verbose)
            printf("%s | RAW: %s\n", tbuf, line);
        else
            printf("%s | %s\n", tbuf, line);
    }
}

static void parse_reply(char *reply)
{
    char *save = NULL;
    for (char *line = strtok_r(reply, "\r\n", &save); line;
         line = strtok_r(NULL, "\r\n", &save)) {
        while (*line == ' ')
            line++;
        if (*line)
            handle_line(line);
    }
}

int main(int argc, char **argv)
{
    const char *port = DEFAULT_PORT;
    int baud = DEFAULT_BAUD;
    int interval = DEFAULT_INTERVAL;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc)
            port = argv[++i];
        else if (strcmp(argv[i], "--baud") == 0 && i + 1 < argc)
            baud = atoi(argv[++i]);
        else if (strcmp(argv[i], "--interval") == 0 && i + 1 < argc)
            interval = atoi(argv[++i]);
        else if (strcmp(argv[i], "--raw") == 0)
            g_verbose = 1;
        else {
            fprintf(stderr, "用法: %s [--port %s] [--baud 115200] [--interval 5] [--raw]\n",
                    argv[0], DEFAULT_PORT);
            return 1;
        }
    }

    printf("打开串口 %s @ %d bps ...\n", port, baud);
    int fd = open_serial(port, baud);
    if (fd < 0)
        return 1;

    /* 开启 GPS 定位（上报类型 0 表示仅串口查询，不上报服务器） */
    const char enable[] = "config,set,location,2,1,5,0,0,1\r\n";
    write(fd, enable, strlen(enable));
    usleep(500 * 1000);
    tcflush(fd, TCIOFLUSH);
    printf("GPS 定位已开启（config,set,location,2,1,5,0,0,1）\n");

    char buf[1024];
    for (;;) {
        const char q[] = "config,get,gpsext\r\n";
        write(fd, q, strlen(q));

        ssize_t n = read_serial(fd, buf, sizeof(buf), 2000);
        if (n <= 0) {
            char tbuf[32];
            time_t now = time(NULL);
            strftime(tbuf, sizeof(tbuf), "%H:%M:%S", localtime(&now));
            printf("%s | 无定位数据（检查接线/波特率/GPS是否室外）\n", tbuf);
        } else {
            parse_reply(buf);
        }

        sleep((unsigned)interval);
    }

    close(fd);
    return 0;
}
