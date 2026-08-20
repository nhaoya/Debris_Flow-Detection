#ifndef MONITOR_DISPATCHER_H
#define MONITOR_DISPATCHER_H

#include "monitor_queue.h"
#include "uart2_link.h"

void monitor_dispatch_event_manager(EventManager *manager, Uart2Link *uart2);

#endif
