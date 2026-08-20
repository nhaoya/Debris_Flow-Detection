CROSS_COMPILE ?= arm-rockchip830-linux-uclibcgnueabihf-
CC := $(CROSS_COMPILE)gcc

MEDIA_OUT ?= ../../../output/out/media_out
RK_APP_OUTPUT ?= ../out

TARGET := debris_flow_monitor
BUILD_DIR := build
LOCAL_OUT_DIR := out/bin
LOCAL_TARGET := $(LOCAL_OUT_DIR)/$(TARGET)
APP_TARGET := $(RK_APP_OUTPUT)/bin/$(TARGET)

SRCS := \
	main.c \
	isp_min.c \
	camera_pipeline.c \
	image_utils.c \
	motion_blob.c \
	monitor_queue.c \
	motion_engine.c \
	telemetry_protocol.c \
	uart2_link.c \
	monitor_dispatcher.c

OBJS := $(SRCS:%.c=$(BUILD_DIR)/%.o)

CPPFLAGS += \
	-I. \
	-I$(MEDIA_OUT)/include \
	-I$(MEDIA_OUT)/include/rkaiq \
	-I$(MEDIA_OUT)/include/rkaiq/uAPI \
	-I$(MEDIA_OUT)/include/rkaiq/uAPI2 \
	-I$(MEDIA_OUT)/include/rkaiq/common \
	-I$(MEDIA_OUT)/include/rkaiq/xcore \
	-I$(MEDIA_OUT)/include/rkaiq/algos \
	-I$(MEDIA_OUT)/include/rkaiq/iq_parser \
	-I$(MEDIA_OUT)/include/rkaiq/iq_parser_v2 \
	-I$(RK_APP_OUTPUT)/include

CFLAGS += -O2 -Wall -Wextra -std=gnu11 -fno-strict-aliasing

LDFLAGS += \
	-L$(MEDIA_OUT)/lib \
	-L$(RK_APP_OUTPUT)/lib \
	-Wl,-rpath-link,$(MEDIA_OUT)/lib:$(MEDIA_OUT)/root/usr/lib:$(RK_APP_OUTPUT)/lib

LDLIBS += -lrockit -lrkaiq -lpthread -ldl -lm

.PHONY: all clean install check-pure-c

# make 生成 child/out/bin，复制到 project/app/out/bin。
all: $(APP_TARGET)

$(APP_TARGET): $(LOCAL_TARGET)
	@mkdir -p $(RK_APP_OUTPUT)/bin
	@echo "install $(TARGET) -> out/bin"
	cp -f $(LOCAL_TARGET) $(APP_TARGET)

$(LOCAL_TARGET): $(OBJS)
	@mkdir -p $(LOCAL_OUT_DIR)
	$(CC) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

install: $(APP_TARGET)

check-pure-c:
	@! grep -R -nE 'opencv|cv::|std::|<vector>|<chrono>|\.cpp' -- *.c *.h || \
		( echo "ERROR: C++/OpenCV token remains in C sources"; exit 1 )
	@echo "pure-C dependency check: OK"

clean:
	rm -rf $(BUILD_DIR) out
