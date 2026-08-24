#ifndef MONITOR_DISPATCHER_H
#define MONITOR_DISPATCHER_H

#include "monitor_queue.h"
#include "lora_uart.h"

void monitor_dispatch_event_manager(EventManager *manager, LoraUartLink *lora);

#endif
