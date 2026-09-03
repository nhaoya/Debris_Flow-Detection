#include "fourg_forward.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
// ==================== 硬件配置，和你接线一致 ====================
#define FOURG_UART_DEV  "/dev/ttyS4"   // 4G模块接的串口
#define FOURG_BAUD      115200         // 4G模块串口波特率
// =============================================================
static int g_4g_fd = -1;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
// 串口配置：8N1 原始二进制透传模式，不篡改任何数据
static int uart_config(int fd, int baud)
{
    struct termios opt;
    speed_t speed_val;
    switch (baud) {
    case 9600:   speed_val = B9600;   break;
    case 19200:  speed_val = B19200;  break;
    case 38400:  speed_val = B38400;  break;
    case 57600:  speed_val = B57600;  break;
    case 115200: speed_val = B115200; break;
    default:
        fprintf(stderr, "[4G] 不支持的波特率: %d\n", baud);
        return -1;
    }
    if (tcgetattr(fd, &opt) != 0) {
        perror("[4G] 读取串口属性失败");
        return -1;
    }
    cfsetispeed(&opt, speed_val);
    cfsetospeed(&opt, speed_val);
    // 8位数据、无校验、1位停止位、关闭硬件流控
    opt.c_cflag &= ~PARENB;
    opt.c_cflag &= ~CSTOPB;
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    opt.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
    opt.c_cflag &= ~CRTSCTS;
#endif
    // 原始二进制模式：禁止换行转换、禁止回显、禁止流控，保证透传
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
    opt.c_oflag &= ~OPOST;
    opt.c_cc[VMIN]  = 0;
    opt.c_cc[VTIME] = 10;
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &opt) != 0) {
        perror("[4G] 设置串口属性失败");
        return -1;
    }
    // 关闭非阻塞，保证写入稳定，和你手动echo效果一致
    int flag = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flag & ~O_NDELAY);
    return 0;
}
// 可靠写入：保证所有字节真正从硬件发出
static ssize_t write_all(int fd, const uint8_t *data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = write(fd, data + sent, len - sent);
        if (n > 0) {
            sent += (size_t)n;
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        return -1;
    }
    tcdrain(fd); // 等待硬件发送完成，确保数据给到4G模块
    return (ssize_t)sent;
}
int fourg_forward_init(void)
{
    if (g_4g_fd >= 0) {
        printf("[4G] 串口已初始化，跳过重复操作\n");
        return 0;
    }
    int fd = open(FOURG_UART_DEV, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        fprintf(stderr, "[4G] 打开串口 %s 失败: %s\n", FOURG_UART_DEV, strerror(errno));
        return -1;
    }
    if (uart_config(fd, FOURG_BAUD) != 0) {
        close(fd);
        return -2;
    }
    g_4g_fd = fd;
    printf("[4G] 初始化完成: %s @ %d\n", FOURG_UART_DEV, FOURG_BAUD);
    // 初始化后自动发一条测试消息，验证链路
    const char test[] = "4g_module_init_ok";
    write_all(g_4g_fd, (const uint8_t *)test, strlen(test));
    return 0;
}
int fourg_forward_send(const uint8_t *data, size_t len)
{
    if (g_4g_fd < 0) return -1;
    if (data == NULL || len == 0) return -2;
    pthread_mutex_lock(&g_mutex);
    ssize_t ret = write_all(g_4g_fd, data, len);
    pthread_mutex_unlock(&g_mutex);
    return (ret == (ssize_t)len) ? 0 : -3;
}
void fourg_forward_deinit(void)
{
    pthread_mutex_lock(&g_mutex);
    if (g_4g_fd >= 0) {
        close(g_4g_fd);
        g_4g_fd = -1;
    }
    pthread_mutex_unlock(&g_mutex);
    pthread_mutex_destroy(&g_mutex);
}
