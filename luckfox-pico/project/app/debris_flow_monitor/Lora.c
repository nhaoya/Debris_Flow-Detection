/*
 * lora_heartbeat_node.c - Luckfox Pico Zero Lora 心跳节点（透传广播）
 * 每个节点启动时指定自己的编号（1~255），10秒发送一次心跳
 * 内容：Node X: 2026-08-19 10:00:00
 * 
 * 编译：arm-rockchip830-linux-uclibcgnueabihf-gcc -o lora_node lora_heartbeat_node.c -static
 * 运行：./lora_node /dev/ttyS3 1   （1为节点编号）
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

// ========== 用户配置 ==========
#define BAUDRATE B9600
#define ARRAY_SIZE 64
#define UART_DEVICE "/dev/ttyS3"
#define HEARTBEAT_INTERVAL 10   // 心跳间隔（秒）

// GPIO 系统编号（根据引脚图）
#define GPIO_M0   70
#define GPIO_M1   71
#define GPIO_AUX  54
// ================================

static int uart_fd = -1;
static unsigned char rxBuffer[ARRAY_SIZE];
static unsigned int rxIndex = 0;
static int node_id = 2;  // 默认节点编号

// ---------- GPIO 操作 ----------
int gpio_export(int pin) {
    char buf[64];
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return -1;
    snprintf(buf, sizeof(buf), "%d", pin);
    write(fd, buf, strlen(buf));
    close(fd);
    return 0;
}

int gpio_set_direction(int pin, const char* dir) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    write(fd, dir, strlen(dir));
    close(fd);
    return 0;
}

int gpio_write(int pin, int value) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    char buf[2] = { value ? '1' : '0', 0 };
    write(fd, buf, 1);
    close(fd);
    return 0;
}

int gpio_read(int pin) {
    char path[64], value;
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;
    if (read(fd, &value, 1) < 0) { close(fd); return -1; }
    close(fd);
    return (value == '1') ? 1 : 0;
}

// ---------- 串口初始化 ----------
int serial_init(const char* device) {
    struct termios options;
    uart_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd < 0) {
        perror("open serial");
        return -1;
    }
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;  // 1秒超时
    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &options);
    return uart_fd;
}

// ---------- 串口发送 ----------
int uart_send(const unsigned char* buf, int len) {
    if (uart_fd < 0) return -1;
    int written = write(uart_fd, buf, len);
    tcdrain(uart_fd);
    return written;
}

int uart_send_str(const char* str) {
    return uart_send((const unsigned char*)str, strlen(str));
}

// ---------- Lora 模式设置 ----------
void lora_set_mode(int mode) {
    int m0 = (mode >> 1) & 1;
    int m1 = mode & 1;
    gpio_write(GPIO_M0, m0);
    gpio_write(GPIO_M1, m1);
    usleep(100000);  // 等待模块稳定
}

// ---------- 获取当前时间字符串 ----------
void get_time_str(char* buf, int len) {
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", tm_info);
}

// ---------- 主函数 ----------
int main(int argc, char* argv[]) {
    const char* uart_dev = (argc > 1) ? argv[1] : UART_DEVICE;
    if (argc > 2) {
        node_id = atoi(argv[2]);
        if (node_id < 1) node_id = 1;
    }

    printf("=== Luckfox Lora 心跳节点 (透传广播) ===\n");
    printf("节点编号: %d\n", node_id);
    printf("GPIO: M0=70(18pin), M1=71(16pin), AUX=54(12pin)\n");
    printf("波特率: 9600, 心跳间隔: %d 秒\n", HEARTBEAT_INTERVAL);

    // 1. 初始化串口
    if (serial_init(uart_dev) < 0) {
        fprintf(stderr, "Failed to open %s\n", uart_dev);
        return EXIT_FAILURE;
    }
    printf("UART opened: %s\n", uart_dev);

    // 2. 初始化 GPIO
    gpio_export(GPIO_M0);
    gpio_export(GPIO_M1);
    gpio_export(GPIO_AUX);
    gpio_set_direction(GPIO_M0, "out");
    gpio_set_direction(GPIO_M1, "out");
    gpio_set_direction(GPIO_AUX, "in");

    // 3. 设置为普通透传模式
    lora_set_mode(0);
    printf("Lora mode: Normal (M0=0, M1=0) - 透传广播模式\n");

    // 4. 读取 AUX 状态
    int aux = gpio_read(GPIO_AUX);
    printf("AUX initial state: %d (1=idle, 0=busy)\n", aux);

    // 5. 准备心跳发送
    time_t last_heartbeat = 0;
    char time_str[64];
    char heartbeat_msg[128];

    // 6. 主循环：接收回显 + 定时心跳
    fd_set readfds;
    struct timeval tv;
    unsigned char tmp[ARRAY_SIZE];

    printf("Listening on %s... (Ctrl+C to exit)\n", uart_dev);

    while (1) {
        // ---------- 检查心跳 ----------
        time_t now = time(NULL);
        if (now - last_heartbeat >= HEARTBEAT_INTERVAL) {
            get_time_str(time_str, sizeof(time_str));
            snprintf(heartbeat_msg, sizeof(heartbeat_msg), "Node %d: %s\n", node_id, time_str);
            // 发送心跳
            uart_send_str(heartbeat_msg);
            printf("Sent: %s", heartbeat_msg);
            last_heartbeat = now;
        }

        // ---------- 检查串口是否有数据 ----------
        FD_ZERO(&readfds);
        FD_SET(uart_fd, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 100000;  // 100ms 轮询

        int ret = select(uart_fd + 1, &readfds, NULL, NULL, &tv);
        if (ret < 0) {
            perror("select");
            break;
        } else if (ret == 0) {
            continue;
        }

        if (FD_ISSET(uart_fd, &readfds)) {
            int n = read(uart_fd, tmp, ARRAY_SIZE - 1);
            if (n > 0) {
                // 存入缓冲区
                for (int i = 0; i < n; i++) {
                    if (rxIndex < ARRAY_SIZE) {
                        rxBuffer[rxIndex++] = tmp[i];
                    }
                }
                // 回显（可选，如果你想在本地打印接收到的数据）
                if (rxIndex > 0) {
                    // 不自动回显，只是打印出来
                    printf("RX %d bytes: ", rxIndex);
                    fwrite(rxBuffer, 1, rxIndex, stdout);
                    printf("\n");
                    // 如果想让模块把收到的数据再发出去（回显），取消下一行注释
                    // uart_send(rxBuffer, rxIndex);
                    memset(rxBuffer, 0, ARRAY_SIZE);
                    rxIndex = 0;
                }
            }
        }
    }

    close(uart_fd);
    return 0;
}