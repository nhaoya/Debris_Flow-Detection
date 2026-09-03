#include "fourg_image_forward.h"
#include "fourg_forward.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

// ========== 可根据部署修改 ==========
#define GATEWAY_ID      "16"
#define DEVICE_ID_FMT  "dev_%03d"
#define MSG_ID_FMT    "%013lld"
// ========================================

static const char base64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int base64_encode(const uint8_t *in, size_t in_len,
                         char *out, size_t out_size, size_t *out_len)
{
    size_t i = 0, j = 0;
    while (i < in_len && j + 4 <= out_size) {
        uint32_t val = in[i++] << 16;
        if (i < in_len) val |= in[i++] << 8;
        if (i < in_len) val |= in[i++];
        out[j++] = base64_table[(val >> 18) & 0x3F];
        out[j++] = base64_table[(val >> 12) & 0x3F];
        out[j++] = (i > in_len - 1) ? '=' : base64_table[(val >> 6) & 0x3F];
        out[j++] = (i > in_len - 2) ? '=' : base64_table[val & 0x3F];
    }
    if (j >= out_size) return -1;
    out[j] = '\0';
    *out_len = j;
    return 0;
}

int fourg_image_forward_send(uint16_t source_dev_id,
                             uint32_t image_id,
                             uint32_t event_id,
                             const char *jpeg_path)
{
    (void)image_id;
    (void)event_id;

    // 读取JPEG文件
    FILE *fp = fopen(jpeg_path, "rb");
    if (!fp) return -1;

    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (fsize <= 0 || fsize > 32768) {
        fclose(fp);
        return -2;
    }

    uint8_t *buf = malloc((size_t)fsize);
    if (!buf) {
        fclose(fp);
        return -3;
    }
    size_t rd = fread(buf, 1, (size_t)fsize, fp);
    fclose(fp);
    if (rd != (size_t)fsize) {
        free(buf);
        return -4;
    }

    // Base64编码
    size_t b64_len = 0;
    size_t b64_max = ((rd + 2) / 3) * 4 + 1;
    char *b64 = malloc(b64_max);
    if (!b64) {
        free(buf);
        return -5;
    }
    if (base64_encode(buf, rd, b64, b64_max, &b64_len) != 0) {
        free(b64);
        free(buf);
        return -6;
    }
    free(buf);

    // 组装字段
    char device_id[32];
    char msg_id[32];
    struct timeval tv;
    gettimeofday(&tv, NULL);

    snprintf(device_id, sizeof(device_id), DEVICE_ID_FMT, source_dev_id);
    snprintf(msg_id, sizeof(msg_id), MSG_ID_FMT,
             (long long)(tv.tv_sec * 1000ULL + tv.tv_usec / 1000ULL));

    // JSON严格按指定字段顺序：device_type → device_id → msg_id → gateway_id → timestamp → data
    size_t json_size = b64_len + 256;
    char *json = malloc(json_size);
    if (!json) {
        free(b64);
        return -7;
    }

    int json_len = snprintf(json, json_size,
        "{\"device_type\":\"image\",\"device_id\":\"%s\",\"msg_id\":\"%s\","
        "\"gateway_id\":\"%s\",\"timestamp\":%lld,\"data\":\"%s\"}",
        device_id, msg_id, GATEWAY_ID,
        (long long)(tv.tv_sec * 1000ULL + tv.tv_usec / 1000ULL),
        b64);

    free(b64);

    if (json_len <= 0 || (size_t)json_len >= json_size) {
        free(json);
        return -8;
    }

    // 通过4G模块发送
    int ret = fourg_forward_send((const uint8_t *)json, (size_t)json_len);
    free(json);
    return ret;
}
