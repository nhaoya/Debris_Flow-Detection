#include "camera_pipeline.h"
#include "debris_config.h"
#include "monitor_dispatcher.h"

#include <stdio.h>
#include <string.h>

#include <rk_mpi_sys.h>
#include <rk_mpi_vi.h>
#include <rk_mpi_mb.h>

void camera_pipeline_reset(CameraPipeline *pipeline) {
    if (!pipeline) return;
    memset(pipeline, 0, sizeof(*pipeline));
    motion_engine_init(&pipeline->motion);
}

static int init_vi_device(CameraPipeline *pipeline) {
    VI_DEV_ATTR_S dev_attr;
    VI_DEV_BIND_PIPE_S bind_pipe;
    int ret;
    memset(&dev_attr, 0, sizeof(dev_attr));
    memset(&bind_pipe, 0, sizeof(bind_pipe));

    ret = RK_MPI_VI_GetDevAttr(DF_DEV_ID, &dev_attr);
    if (ret == RK_ERR_VI_NOT_CONFIG) {
        ret = RK_MPI_VI_SetDevAttr(DF_DEV_ID, &dev_attr);
        if (ret != RK_SUCCESS) {
            fprintf(stderr, "[VI] SetDevAttr failed: %#x\n", ret);
            return ret;
        }
        printf("[VI] device configured\n");
    } else if (ret != RK_SUCCESS) {
        fprintf(stderr, "[VI] GetDevAttr failed: %#x\n", ret);
        return ret;
    }

    ret = RK_MPI_VI_GetDevIsEnable(DF_DEV_ID);
    if (ret != RK_SUCCESS) {
        ret = RK_MPI_VI_EnableDev(DF_DEV_ID);
        if (ret != RK_SUCCESS) {
            fprintf(stderr, "[VI] EnableDev failed: %#x\n", ret);
            return ret;
        }
        pipeline->dev_enabled = true;

        /* 保持 V1.4.2 / RV1106 IPC SDK 当前行为：u32Num = pipe_id(0). */
        bind_pipe.u32Num = DF_PIPE_ID;
        bind_pipe.PipeId[0] = DF_PIPE_ID;
        ret = RK_MPI_VI_SetDevBindPipe(DF_DEV_ID, &bind_pipe);
        if (ret != RK_SUCCESS) {
            fprintf(stderr, "[VI] SetDevBindPipe failed: %#x\n", ret);
            return ret;
        }
        printf("[VI] device enabled\n");
        printf("[VI] dev=%d bind pipe=%d\n", DF_DEV_ID, DF_PIPE_ID);
    } else {
        printf("[VI] device already enabled\n");
    }
    return RK_SUCCESS;
}

static int init_detect_channel(CameraPipeline *pipeline) {
    VI_CHN_ATTR_S attr;
    int ret;
    memset(&attr, 0, sizeof(attr));
    attr.stIspOpt.u32BufCount = 2;
    attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
    attr.stIspOpt.stMaxSize.u32Width = DF_MAX_WIDTH;
    attr.stIspOpt.stMaxSize.u32Height = DF_MAX_HEIGHT;
    attr.stSize.u32Width = DF_WIDTH;
    attr.stSize.u32Height = DF_HEIGHT;
    attr.enPixelFormat = RK_FMT_YUV420SP;
    attr.enCompressMode = COMPRESS_MODE_NONE;
    attr.u32Depth = 1;

    ret = RK_MPI_VI_SetChnAttr(DF_PIPE_ID, DF_DETECT_CHN, &attr);
    if (ret != RK_SUCCESS) {
        fprintf(stderr, "[VI] SetChnAttr ch2 failed: %#x\n", ret);
        return ret;
    }
    ret = RK_MPI_VI_EnableChn(DF_PIPE_ID, DF_DETECT_CHN);
    if (ret != RK_SUCCESS) {
        fprintf(stderr, "[VI] EnableChn ch2 failed: %#x\n", ret);
        return ret;
    }
    pipeline->chn_enabled = true;
    printf("[VI] detect channel ready: %dx%d NV12, max=%dx%d, depth=1\n",
           DF_WIDTH, DF_HEIGHT, DF_MAX_WIDTH, DF_MAX_HEIGHT);
    return RK_SUCCESS;
}

int camera_pipeline_init(CameraPipeline *pipeline) {
    int ret;
    if (!pipeline) return -1;
    ret = init_vi_device(pipeline);
    if (ret != RK_SUCCESS) return ret;
    ret = init_detect_channel(pipeline);
    if (ret != RK_SUCCESS) return ret;
    return RK_SUCCESS;
}

int camera_pipeline_capture_loop(CameraPipeline *pipeline,
                                 volatile sig_atomic_t *running,
                                 LoraUartLink *lora) {
    uint64_t fps_frames = 0;
    double fps_begin;
    if (!pipeline || !running) {
        fprintf(stderr, "[CAM] running is null\n");
        return -1;
    }

    printf("[CAM] start capture\n");
    motion_engine_print_config();
    fps_begin = monotonic_seconds();

    while (*running) {
        VIDEO_FRAME_INFO_S frame;
        RK_S32 ret;
        void *ptr;
        GrayImage gray;
        double now, seconds;
        memset(&frame, 0, sizeof(frame));
        ret = RK_MPI_VI_GetChnFrame(DF_PIPE_ID, DF_DETECT_CHN, &frame, 1000);
        if (ret != RK_SUCCESS) {
            fprintf(stderr, "[CAM] GetChnFrame timeout/error: %d\n", ret);
            continue;
        }

        ++fps_frames;
        ptr = RK_MPI_MB_Handle2VirAddr(frame.stVFrame.pMbBlk);
        if (!ptr) {
            fprintf(stderr, "[CAM] Handle2VirAddr NULL\n");
            RK_MPI_VI_ReleaseChnFrame(DF_PIPE_ID, DF_DETECT_CHN, &frame);
            continue;
        }

        gray.data = (uint8_t *)ptr;
        gray.width = (int)frame.stVFrame.u32Width;
        gray.height = (int)frame.stVFrame.u32Height;
        gray.stride = (int)frame.stVFrame.u32VirWidth;
        motion_engine_process_frame(&pipeline->motion, &gray);

        ret = RK_MPI_VI_ReleaseChnFrame(DF_PIPE_ID, DF_DETECT_CHN, &frame);
        if (ret != RK_SUCCESS)
            fprintf(stderr, "[CAM] ReleaseChnFrame failed: %d\n", ret);

        /*
         * Keep the V1.4.2 ordering: only consume event queues after the VI frame
         * is released. UART I/O itself is performed by an independent worker.
         */
        monitor_dispatch_event_manager(&pipeline->motion.event_manager, lora);

        now = monotonic_seconds();
        seconds = now - fps_begin;
        if (seconds >= 5.0) {
            const double fps = (double)fps_frames / seconds;
            printf("[CAM] average fps=%.2f frames=%llu time=%.2fs\n",
                   fps, (unsigned long long)fps_frames, seconds);
            fflush(stdout);
            fps_frames = 0;
            fps_begin = now;
        }
    }

    motion_engine_finish_process(&pipeline->motion);
    monitor_dispatch_event_manager(&pipeline->motion.event_manager, lora);
    printf("[CAM] capture loop exit\n");
    fflush(stdout);
    return 0;
}

void camera_pipeline_deinit(CameraPipeline *pipeline) {
    if (!pipeline) return;
    if (pipeline->chn_enabled) {
        RK_MPI_VI_DisableChn(DF_PIPE_ID, DF_DETECT_CHN);
        pipeline->chn_enabled = false;
        printf("[VI] detect channel disabled\n");
    }
    if (pipeline->dev_enabled) {
        RK_MPI_VI_DisableDev(DF_DEV_ID);
        pipeline->dev_enabled = false;
        printf("[VI] device disabled\n");
    }
    motion_engine_deinit(&pipeline->motion);
}
