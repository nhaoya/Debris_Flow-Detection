#include "monitor_dispatcher.h"

void monitor_dispatch_event_manager(EventManager *manager, LoraUartLink *lora) {
    MonitorMessage message;
    SnapshotPackage snapshot;
    if (!manager) return;

    while (event_manager_pop_telemetry(manager, &message)) {
        /* Keep the original V1.4.2 log line unchanged. */
        monitor_message_log(&message, "DATA");
        if (lora) lora_uart_enqueue(lora, &message);
    }

    while (event_manager_pop_snapshot(manager, &snapshot)) {
        /* Image payload is intentionally NOT sent over LoRa in this revision. */
        snapshot_package_log(&snapshot);
    }
}
