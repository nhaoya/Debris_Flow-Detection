#include "monitor_dispatcher.h"

void monitor_dispatch_event_manager(EventManager *manager, Uart2Link *uart2) {
    MonitorMessage message;
    SnapshotPackage snapshot;
    if (!manager) return;

    while (event_manager_pop_telemetry(manager, &message)) {
        monitor_message_log(&message, "DATA");
        if (uart2) uart2_link_enqueue(uart2, &message);
    }

    while (event_manager_pop_snapshot(manager, &snapshot)) {
        /* Image payload is intentionally NOT sent over UART2 in this revision. */
        snapshot_package_log(&snapshot);
    }
}
