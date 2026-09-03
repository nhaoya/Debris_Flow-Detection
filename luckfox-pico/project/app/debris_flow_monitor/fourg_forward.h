#ifndef FOURG_FORWARD_H
#define FOURG_FORWARD_H
#include <stdint.h>
#include <stddef.h>
/**
 * @brief  初始化4G模块串口（UART4）
 * @return 0成功，负数失败
 */
int fourg_forward_init(void);
/**
 * @brief  发送数据到4G模块，自动透传到MQTT服务器
 * @param  data  待发送数据首地址
 * @param  len   数据字节长度
 * @return 0成功，负数失败
 */
int fourg_forward_send(const uint8_t *data, size_t len);
/**
 * @brief  反初始化，关闭4G串口
 */
void fourg_forward_deinit(void);
#endif
