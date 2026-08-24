#ifndef CAMERA_PIPELINE_H
#define CAMERA_PIPELINE_H

#include "motion_engine.h"
#include "lora_uart.h"

#include <signal.h>
#include <stdbool.h>

typedef struct {
    bool dev_enabled;
    bool chn_enabled;
    MotionEngine motion;
} CameraPipeline;

void camera_pipeline_reset(CameraPipeline *pipeline);
int camera_pipeline_init(CameraPipeline *pipeline);
int camera_pipeline_capture_loop(CameraPipeline *pipeline,
                                 volatile sig_atomic_t *running,
                                 LoraUartLink *lora);
void camera_pipeline_deinit(CameraPipeline *pipeline);

#endif
