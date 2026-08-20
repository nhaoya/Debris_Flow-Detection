#include "isp_min.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <rk_aiq_user_api2_sysctl.h>

static rk_aiq_sys_ctx_t *g_aiq_ctx = NULL;
static int g_started = 0;

int isp_min_start(const char *iq_dir) {
    const int cam_id = 0;
    rk_aiq_static_info_t info;
    int ret;

    if (!iq_dir) return -1;
    printf("[ISP] iq dir: %s\n", iq_dir);
    memset(&info, 0, sizeof(info));

    ret = rk_aiq_uapi2_sysctl_enumStaticMetasByPhyId(cam_id, &info);
    if (ret < 0) {
        fprintf(stderr, "[ISP] enumStaticMetasByPhyId failed, ret=%d\n", ret);
        return -1;
    }

    printf("[ISP] cam=%d phyId=%d sensor=%s\n",
           cam_id, info.sensor_info.phyId, info.sensor_info.sensor_name);

    /* 保持当前 normal/day + 2 个 rkraw_rx buffer 的启动方式。 */
    setenv("HDR_MODE", "0", 1);

    printf("[ISP] preInit_devBufCnt begin\n");
    ret = rk_aiq_uapi2_sysctl_preInit_devBufCnt(
        info.sensor_info.sensor_name, "rkraw_rx", 2);
    printf("[ISP] preInit_devBufCnt end, ret=%d\n", ret);
    if (ret < 0) return -1;

    printf("[ISP] preInit_scene begin, sensor='%s', main='normal', sub='day'\n",
           info.sensor_info.sensor_name);
    ret = rk_aiq_uapi2_sysctl_preInit_scene(
        info.sensor_info.sensor_name, "normal", "day");
    printf("[ISP] preInit_scene end, ret=%d\n", ret);
    if (ret < 0) return -1;

    printf("[ISP] sysctl_init begin, sensor='%s', iq='%s'\n",
           info.sensor_info.sensor_name, iq_dir);
    g_aiq_ctx = rk_aiq_uapi2_sysctl_init(
        info.sensor_info.sensor_name, iq_dir, NULL, NULL);
    printf("[ISP] sysctl_init end, ctx=%p\n", (void *)g_aiq_ctx);
    if (!g_aiq_ctx) return -1;

    printf("[ISP] prepare begin\n");
    ret = rk_aiq_uapi2_sysctl_prepare(
        g_aiq_ctx, 0, 0, RK_AIQ_WORKING_MODE_NORMAL);
    printf("[ISP] prepare end, ret=%d\n", ret);
    if (ret < 0) {
        rk_aiq_uapi2_sysctl_deinit(g_aiq_ctx);
        g_aiq_ctx = NULL;
        return -1;
    }

    printf("[ISP] start begin\n");
    ret = rk_aiq_uapi2_sysctl_start(g_aiq_ctx);
    printf("[ISP] start end, ret=%d\n", ret);
    if (ret < 0) {
        rk_aiq_uapi2_sysctl_deinit(g_aiq_ctx);
        g_aiq_ctx = NULL;
        return -1;
    }

    g_started = 1;
    printf("[ISP] RKAIQ started successfully\n");
    fflush(stdout);
    return 0;
}

void isp_min_stop(void) {
    if (!g_aiq_ctx) return;
    printf("[ISP] stopping RKAIQ\n");
    if (g_started) rk_aiq_uapi2_sysctl_stop(g_aiq_ctx, false);
    rk_aiq_uapi2_sysctl_deinit(g_aiq_ctx);
    g_aiq_ctx = NULL;
    g_started = 0;
    printf("[ISP] RKAIQ stopped\n");
    fflush(stdout);
}
