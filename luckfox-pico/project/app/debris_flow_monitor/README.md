# UART-RAW hotfix note

This package is Chain Relay fix3 plus a binary-transparent termios fix for `/dev/ttyS3`. Confirm startup prints `UART raw-binary iflag=0x0 oflag=0x0 lflag=0x0`. See `CHANGELOG_V1_4_4_CHAIN_FIX3_UART_RAW.md`.

# V1.4.4 Communication/Image Reliability Chain Relay fix2

**Recommended deployment topology:** contiguous monitor IDs in high-to-low order, e.g. `10 -> 11 -> 12 -> 13 -> terminal`.

This supersedes the experimental controlled-flood relay for bulk images. EVENT is forwarded one hop at a time; JPEG uses hop-by-hop NACK/ACK and store-and-forward only after the current receiver has a complete image. See `CHAIN_RELAY_PROTOCOL_V1_4_4.md` and `CHANGELOG_V1_4_4_CHAIN_RELAY_FIX2.md`.

Example common chain arguments for all four monitors:

```bash
--chain-first-id 10 --chain-last-id 13
```

# debris_flow_monitor Pure-C V1.4.4 Communication/Image Reliability Relay fix

Base: V1.4.4 Communication/Image AUX-fix1.

The visual detector is unchanged from V1.4.3-fix1: 4-zone camera-shake consensus, event thresholds/FSM, blob/tracker logic and the local >=30 s event-snapshot throttle are not modified here. This patch changes only LoRa communication reliability.

## Hardware defaults

- RV1106G3 / Luckfox Pico Zero style target
- IMX415 -> RKAIQ -> VI dev0/ch2 -> 640x360 NV12
- detector uses Y plane only
- LoRa: DX-LR32-433T22D transparent UART
- `/dev/ttyS3`, 9600 baud, 8N1
- M0 GPIO70, M1 GPIO71, AUX GPIO54
- M0=0/M1=0 high-efficiency mode
- default heartbeat 600 s

## AUX flow control

DX-LR32 AUX polarity is:

```text
AUX=1 -> TX/RX/mode switching busy
AUX=0 -> idle / operation complete
```

The worker waits for stable LOW before a frame, writes UART, calls `tcdrain()`, observes the BUSY cycle, and waits for LOW again before allowing the next frame. Do not add a fixed per-chunk `usleep()` on top of this version.

Startup should show:

```text
imageChunkData=40
auxFlow=LOW_IDLE/HIGH_BUSY
reliability=BCAST_NACK+EVENT_ACK
```

## Four monitoring nodes: all TX and RX

This Reliability fix is explicitly designed for a small broadcast group where every monitoring node can originate data and can receive peers. There is no RX-only mode.

When two bulk images overlap, a node receiving an incomplete image from a lower device ID temporarily defers its own bulk image. After the lower-ID transfer/repair leaves the receiving state, the deferred sender refreshes its META and resumes. Short reliability control frames and critical EVENT traffic remain possible.

This is application-layer arbitration, not full TDMA.

## Image format

```text
source:      640x360 NV12/Y
LoRa JPEG:   340x180 grayscale
chunk data:  40 bytes
full normal chunk wire frame: 64 bytes
```

Deployment preview remains:

```text
first valid frame -> immediately queue JPEG
then every 5 s
for first 600 s by default
```

For bench testing it can still be disabled with:

```text
--deploy-preview-seconds 0
```

This disables only deployment previews, not EVENT images or LoRa receive/transmit capability.

## Image reliability: broadcast NACK + selective repair

The initial image transfer is:

```text
IMAGE_META
IMAGE_CHUNK 1..N
IMAGE_DONE round=0
```

Each receiver answers in a deterministic response slot:

```text
complete -> IMAGE_FEEDBACK ACK
incomplete -> IMAGE_FEEDBACK NACK + missing bitmap
```

The sender waits for the feedback window and merges all NACK bitmaps. Only the union of missing chunks is retransmitted, followed by another IMAGE_DONE. Up to three repair rounds are allowed.

There is no per-chunk ACK.

If no feedback arrives, the sender first repeats IMAGE_DONE. If feedback is still absent, it performs one full META+CHUNK resend so a receiver that missed META can rejoin. The image is eventually logged as either `CONFIRMED` or `UNCONFIRMED`; one bad receiver cannot block the LoRa worker forever.

A complete received image is written to:

```text
/tmp/debris_rx_dev<SOURCE>_latest.jpg
/tmp/debris_rx_dev<SOURCE>_event<EVENT>_img<IMAGE>.jpg
```

## EVENT reliability

```text
EVENT_START  -> ACK + max 2 retransmissions
EVENT_END    -> ACK + max 2 retransmissions
EVENT_UPDATE -> no ACK
HEARTBEAT    -> no ACK
```

All peers may ACK START/END, but ACKs are scheduled in deterministic slots. If a node hears another valid ACK for the same source/sequence before its own ACK slot, it suppresses its pending ACK. Any peer ACK confirms the sender's critical event packet.

During an image feedback window, ordinary EVENT_UPDATE transmission is briefly held to leave room for ACK/NACK traffic. START/END remain eligible.

## Packet types

```text
0x01 EVENT_START
0x02 EVENT_UPDATE
0x03 EVENT_END
0x04 HEARTBEAT
0x10 IMAGE_META
0x11 IMAGE_CHUNK
0x12 IMAGE_DONE
0x13 IMAGE_FEEDBACK
0x14 EVENT_ACK
```

See:

```text
IMAGE_PROTOCOL_V1_4_4.md
RELIABILITY_PROTOCOL_V1_4_4.md
LORA_UART3_PROTOCOL.md
```

## Build

Copy the directory to:

```text
/home/nine_sing/projeck/debris_flow/luckfox-pico/project/app/debris_flow_monitor/
```

Do one clean build because protocol headers and `LoraUartLink` changed:

```sh
cd /home/nine_sing/projeck/debris_flow/luckfox-pico/project/app/debris_flow_monitor
rm -rf build out

cd /home/nine_sing/projeck/debris_flow/luckfox-pico
./build.sh app
```

All monitoring boards must run this same Reliability-Relay-fix binary. Device IDs must be unique.

## Run examples

Normal monitor node 10:

```sh
export LD_LIBRARY_PATH=/oem/usr/lib
/oem/usr/bin/debris_flow_monitor \
  --device-id 10 \
  --heartbeat-seconds 600
```

Monitor node 11, deployment preview disabled only for a bench run:

```sh
/oem/usr/bin/debris_flow_monitor \
  --device-id 11 \
  --heartbeat-seconds 600 \
  --deploy-preview-seconds 0
```

Node 11 still detects local events, transmits local EVENT/event-images and receives node 10. It is not RX-only.

## Expected reliability logs

Sender initial transfer:

```text
[LORA-TX-IMAGE] ... IMAGE_META ...
[LORA-TX-IMAGE] ... chunk=...
[LORA-TX-REL] ... IMAGE_DONE round=0 ...
```

Receiver incomplete:

```text
[LORA-RX-IMAGE] ... IMAGE_DONE ... feedback=NACK missing=5
[LORA-TX-REL] IMAGE_NACK ... missing=5
```

Sender repair:

```text
[LORA-RX-REL] IMAGE_FEEDBACK ... NACK missing=5
[LORA-TX-REL] image=... repairRound=1 missingUnion=5
[LORA-TX-IMAGE] ... repair-chunk=...
```

Successful receiver/sender:

```text
[LORA-RX-IMAGE] ... status=COMPLETE path=/tmp/...
[LORA-TX-REL] IMAGE_ACK ...
[LORA-TX-IMAGE] ... status=CONFIRMED reason=broadcast-ack
```

Critical event:

```text
[LORA-TX-REL] EVENT_ACK ...
[LORA-REL] EVENT_ACK from=... status=CONFIRMED
```

## Shutdown counters

Useful counters now include:

```text
txImageConfirmed
txImageUnconfirmed
repairRounds
repairChunks
imgAckTx
imgNackTx
imgFeedbackRx
eventAckTx
eventAckRx
eventRetries
eventUnconfirmed
relayQueued
relayTx
relaySuppressed
relayDrop
relayOwnIgnored
auxTimeout
auxBusyCycles
auxNoBusyObserved
```

## Relay to the single 4G terminal

This build adds controlled broadcast forwarding. Every monitor still originates its own packets, but every valid foreign LoRa frame is also eligible for forwarding. The wire bytes are preserved, so a forwarded Node 10 EVENT or IMAGE still carries Node 10 as the source when it reaches the terminal.

Fast loops and duplicate broadcast storms are limited by a deterministic relay slot plus a short frame-hash suppression cache. If another relay forwards the same frame before this node's slot, this node cancels its pending copy. Later EVENT retries and IMAGE repair chunks are allowed through after the suppression window.

The terminal should de-duplicate repeated copies by original source + sequence/image/chunk identity before or after 4G forwarding. See `RELAY_PROTOCOL_V1_4_4.md`.

## UART3 prerequisite

```sh
tr '\0' '\n' < /proc/device-tree/serial@ff4d0000/status
ls -l /dev/ttyS3
```

Expected:

```text
okay
/dev/ttyS3
```
