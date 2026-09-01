# Porting notes — V1.4.4

Install this directory as:

```text
luckfox-pico/project/app/debris_flow_monitor/
```

New V1.4.4 source files that must be copied with the rest of the target:

```text
jpeg_gray.c/.h
image_transport.c/.h
```

The Makefile already includes them.

UART3 still has to be enabled by the running board DTS before the application starts. Application code cannot create `/dev/ttyS3` when the kernel/device tree did not instantiate it.

Camera startup is now intentionally before LoRa startup:

```text
RKAIQ -> RK_MPI_SYS -> VI ch2 -> LoRa worker -> capture loop
```

The optional `S99debris_flow_monitor.example` shows unattended Buildroot startup. Edit the node `DEVICE_ID` before deployment.

No new external JPEG/OpenCV library is required. JPEG is pure C and `-lm` was already linked by the target.
