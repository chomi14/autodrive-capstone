# LEGACY - DO NOT USE FOR VEHICLE RUN

serial_sender_node와 sw_verification_node는 vehicle_io READY/ARM gate를 우회합니다. serial_sender_node는 import 시에도 포트를 엽니다.

CANONICAL - USE THIS: `vehicle_bringup_pkg` + `vehicle_io_pkg`.

세부 분류/launch 표: [WORKSPACE_AUDIT](../../docs/WORKSPACE_AUDIT.md).
