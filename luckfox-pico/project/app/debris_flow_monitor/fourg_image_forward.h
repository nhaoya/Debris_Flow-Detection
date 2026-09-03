#ifndef FOURG_IMAGE_FORWARD_H
#define FOURG_IMAGE_FORWARD_H
#include <stdint.h>

/**
 * @brief  读取本地JPEG文件，Base64编码后按指定JSON格式经4G模块发送到服务器
 * @param  source_dev_id  图片来源节点ID（用于生成device_id）
 * @param  image_id       图片ID
 * @param  event_id       关联事件ID
 * @param  jpeg_path      本地JPEG文件完整路径
 * @return 0成功，负数失败
 */
int fourg_image_forward_send(uint16_t source_dev_id,
                             uint32_t image_id,
                             uint32_t event_id,
                             const char *jpeg_path);

#endif
