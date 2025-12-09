# MAVLink helper for JmavSim/PX4

Python module and CLI wrapper built with `pymavlink` that:
- keeps the latest telemetry from JmavSim/PX4 in memory without blocking control commands;
- prints telemetry snapshots to the console on demand;
- exposes simple methods/CLI commands for `TAKEOFF`, `LAND`, `RETURN_TO_LAUNCH`, and `DO_REPOSITION`.

## Prerequisites
- Python 3.9+.
- `pymavlink` installed: `pip install -r requirements.txt`.
- Running PX4 + jMAVSim (or any MAVLink autopilot) that listens on UDP `127.0.0.1:14540` by default.

## Running JmavSim/PX4 (typical)
1) Start PX4 SITL with jMAVSim (in another shell): `make px4_sitl jmavsim`.
2) Leave it running; PX4 will broadcast MAVLink on UDP port `14540` (companion) and `14550` (GCS).

If your setup uses a different URL, pass it via `--connect` (any `mavutil` connection string is accepted).

## Usage

### Interactive console
```bash
python mavlink_module.py --connect udp:127.0.0.1:14540 interactive
```
Available commands inside the prompt:
- `status` — print the latest telemetry snapshot;
- `arm` / `disarm`;
- `takeoff <alt_m>`;
- `land`;
- `rtl` — return to launch;
- `goto <lat> <lon> [alt] [yaw_deg] [groundspeed]` — MAV_CMD_DO_REPOSITION;
- `quit` — exit.

Telemetry is continuously updated in the background and can be printed periodically with `monitor` mode instead:
```bash
python mavlink_module.py --connect udp:127.0.0.1:14540 monitor --interval 2
```

### One-shot commands
Subcommands run once after connecting and starting the telemetry listener:
```bash
python mavlink_module.py --connect udp:127.0.0.1:14540 takeoff 10
python mavlink_module.py --connect udp:127.0.0.1:14540 land
python mavlink_module.py --connect udp:127.0.0.1:14540 rtl
python mavlink_module.py --connect udp:127.0.0.1:14540 goto 55.753 -37.62 --alt 20 --yaw 90
```

## Telemetry coverage
The background listener records the latest packet for:
`ALTITUDE`, `ATTITUDE`, `BATTERY_STATUS`, `GPS_RAW_INT`, `GLOBAL_POSITION_INT`, `LOCAL_POSITION_NED`, `VFR_HUD`, `SERVO_OUTPUT_RAW`, `EXTENDED_SYS_STATE`, `HOME_POSITION`, `HEARTBEAT`, `STATUS_TEXT`, `SYS_STATUS`, `ODOMETRY`, `COMMAND_ACK`.
Lat/Lon values are also exposed in degrees for `GLOBAL_POSITION_INT` and `GPS_RAW_INT`.

## How it works
- `MavlinkModule.start()` opens a `mavutil` connection, waits for a heartbeat, and spawns a background thread.
- The listener thread consumes MAVLink messages and updates a thread-safe `TelemetryStore`; `COMMAND_ACK` is also queued so control methods can wait for acknowledgements without blocking telemetry updates.
- Control methods (`takeoff`, `land`, `return_to_launch`, `reposition`, `arm`, `disarm`) send `COMMAND_LONG` frames.

Stop the module with `Ctrl+C` or let CLI commands exit; resources are closed via `MavlinkModule.close()`.
