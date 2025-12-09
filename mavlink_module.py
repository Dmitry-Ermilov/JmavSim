#!/usr/bin/env python3
"""Minimal OOP wrapper around pymavlink for JmavSim/PX4 control and telemetry."""

from __future__ import annotations

import argparse
import queue
import threading
import time
from typing import Any, Dict, Iterable, Optional, Tuple, TYPE_CHECKING, TypeAlias, cast

if TYPE_CHECKING:
    from pymavlink.dialects.v20.common import MAVLink_command_ack_message as _AckMessage
    AckMessage: TypeAlias = _AckMessage
else:
    AckMessage: TypeAlias = Any

from pymavlink import mavutil

# Messages we care about; others are ignored by the telemetry loop.
MONITORED_MESSAGES = {
    "ALTITUDE",
    "ATTITUDE",
    "BATTERY_STATUS",
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "LOCAL_POSITION_NED",
    "VFR_HUD",
    "SERVO_OUTPUT_RAW",
    "EXTENDED_SYS_STATE",
    "HOME_POSITION",
    "HEARTBEAT",
    "STATUS_TEXT",
    "SYS_STATUS",
    "ODOMETRY",
    "COMMAND_ACK",
}


def _now() -> float:
    return time.time()


class TelemetryStore:
    """Thread-safe in-memory store for the latest values of selected MAVLink topics."""

    def __init__(self) -> None:
        self._data: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()

    def update(self, name: str, payload: Dict[str, Any]) -> None:
        cleaned = {k: v for k, v in payload.items() if not k.startswith("_") and k != "mavpackettype"}
        cleaned["timestamp"] = _now()

        if name == "GLOBAL_POSITION_INT":
            lat = cleaned.get("lat")
            lon = cleaned.get("lon")
            rel_alt = cleaned.get("relative_alt")
            cleaned["lat_deg"] = lat / 1e7 if isinstance(lat, (int, float)) else None
            cleaned["lon_deg"] = lon / 1e7 if isinstance(lon, (int, float)) else None
            if isinstance(rel_alt, (int, float)):
                cleaned["relative_alt_m"] = rel_alt / 1000.0

        if name == "GPS_RAW_INT":
            lat = cleaned.get("lat")
            lon = cleaned.get("lon")
            alt = cleaned.get("alt")
            cleaned["lat_deg"] = lat / 1e7 if isinstance(lat, (int, float)) else None
            cleaned["lon_deg"] = lon / 1e7 if isinstance(lon, (int, float)) else None
            if isinstance(alt, (int, float)):
                cleaned["alt_m"] = alt / 1000.0

        with self._lock:
            self._data[name] = cleaned

    def snapshot(self) -> Dict[str, Dict[str, Any]]:
        with self._lock:
            return {k: dict(v) for k, v in self._data.items()}

    def latest(self, key: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            value = self._data.get(key)
            return dict(value) if value else None

    def get_global_position(self) -> Optional[Tuple[float, float, Optional[float]]]:
        data = self.latest("GLOBAL_POSITION_INT")
        if not data:
            return None
        lat = data.get("lat_deg")
        lon = data.get("lon_deg")
        alt = data.get("relative_alt_m")
        if lat is None or lon is None:
            return None
        return float(lat), float(lon), float(alt) if alt is not None else None


class MavlinkModule:
    """High-level controller that keeps telemetry updated in a background thread."""

    def __init__(
        self,
        connection: str,
        *,
        baud: Optional[int] = None,
        heartbeat_timeout: float = 10.0,
    ) -> None:
        self.connection_string = connection
        self.baud = baud
        self.heartbeat_timeout = heartbeat_timeout

        self.master: Any = None
        self.telemetry = TelemetryStore()
        self._listener_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._ack_queue: queue.Queue[AckMessage] = queue.Queue(maxsize=50)

    def start(self) -> None:
        if self.master:
            return
        kwargs: Dict[str, Any] = {"autoreconnect": True}
        if self.baud is not None:
            kwargs["baud"] = self.baud
        master: Any = mavutil.mavlink_connection(self.connection_string, **kwargs)
        master.wait_heartbeat(timeout=self.heartbeat_timeout)
        self.master = master
        self._stop_event.clear()
        self._listener_thread = threading.Thread(target=self._listener_loop, name="mavlink-listener", daemon=True)
        self._listener_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._listener_thread:
            self._listener_thread.join(timeout=2.0)
        if self.master:
            self.master.close()
            self.master = None

    def _listener_loop(self) -> None:
        assert self.master is not None
        while not self._stop_event.is_set():
            msg = self.master.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            msg_type = msg.get_type()
            if msg_type not in MONITORED_MESSAGES:
                continue
            payload = msg.to_dict()
            if msg_type == "COMMAND_ACK":
                try:
                    self._ack_queue.put_nowait(msg)
                except queue.Full:
                    pass
                self.telemetry.update(msg_type, payload)
                continue
            self.telemetry.update(msg_type, payload)

    def _send_command_long(self, command: int, params: Iterable[float], wait_ack: bool = True, timeout: float = 5.0) -> Optional[AckMessage]:
        if not self.master:
            raise RuntimeError("Connection is not started")

        target_system = self.master.target_system or 1
        target_component = self.master.target_component or 1
        p = list(params)
        while len(p) < 7:
            p.append(0)

        self.master.mav.command_long_send(
            target_system,
            target_component,
            command,
            0,
            p[0],
            p[1],
            p[2],
            p[3],
            p[4],
            p[5],
            p[6],
        )

        if not wait_ack:
            return None

        deadline = _now() + timeout
        while _now() < deadline:
            try:
                ack = self._ack_queue.get(timeout=deadline - _now())
            except queue.Empty:
                break
            if ack.command == command:
                return ack
        return None

    def arm(self, force: bool = False, timeout: float = 5.0) -> Optional[AckMessage]:
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1, 21196 if force else 0, 0, 0, 0, 0, 0],
            timeout=timeout,
        )

    def disarm(self, timeout: float = 5.0) -> Optional[AckMessage]:
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [0, 0, 0, 0, 0, 0, 0],
            timeout=timeout,
        )

    def takeoff(self, altitude_m: float, yaw_deg: float = float("nan"), timeout: float = 5.0) -> Optional[AckMessage]:
        lat, lon, _ = self._position_or_raise()
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            [0, 0, 0, yaw_deg, lat, lon, altitude_m],
            timeout=timeout,
        )

    def land(self, yaw_deg: float = float("nan"), timeout: float = 5.0) -> Optional[AckMessage]:
        lat, lon, alt = self._position_or_raise()
        final_alt = alt if alt is not None else 0.0
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            [0, 0, 0, yaw_deg, lat, lon, final_alt],
            timeout=timeout,
        )

    def return_to_launch(self, timeout: float = 5.0) -> Optional[AckMessage]:
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            [0, 0, 0, 0, 0, 0, 0],
            timeout=timeout,
        )

    def reposition(
        self,
        lat_deg: float,
        lon_deg: float,
        alt_m: Optional[float] = None,
        yaw_deg: float = float("nan"),
        groundspeed: float = float("nan"),
        timeout: float = 5.0,
    ) -> Optional[AckMessage]:
        params = [
            groundspeed,
            0,  # MAV_DO_REPOSITION_FLAGS
            yaw_deg,
            0,  # yaw is absolute
            lat_deg,
            lon_deg,
            alt_m if alt_m is not None else 0.0,
        ]
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            params,
            timeout=timeout,
        )

    def _position_or_raise(self) -> Tuple[float, float, Optional[float]]:
        pos = self.telemetry.get_global_position()
        if not pos:
            raise RuntimeError("GLOBAL_POSITION_INT has not been received yet")
        return pos

    def print_snapshot(self) -> None:
        snap = self.telemetry.snapshot()
        for key in sorted(snap.keys()):
            print(f"{key}: {snap[key]}")


def _interactive_loop(module: MavlinkModule, print_interval: float) -> None:
    module.start()
    last_print = 0.0
    print("Connected. Type 'help' for available commands. Ctrl+C to exit.")
    try:
        while True:
            if print_interval and _now() - last_print > print_interval:
                module.print_snapshot()
                last_print = _now()
            try:
                command = input("cmd> ").strip()
            except EOFError:
                break
            if not command:
                continue
            parts = command.split()
            op = parts[0].lower()
            args = parts[1:]
            if op in {"quit", "exit"}:
                break
            if op == "help":
                print("Commands: status, arm, disarm, takeoff <alt>, land, rtl, goto <lat> <lon> [alt] [yaw] [gs]")
                continue
            if op == "status":
                module.print_snapshot()
                continue
            if op == "arm":
                print(module.arm())
                continue
            if op == "disarm":
                print(module.disarm())
                continue
            if op == "takeoff" and args:
                altitude = float(args[0])
                print(module.takeoff(altitude))
                continue
            if op == "land":
                print(module.land())
                continue
            if op == "rtl":
                print(module.return_to_launch())
                continue
            if op == "goto" and len(args) >= 2:
                lat = float(args[0])
                lon = float(args[1])
                alt = float(args[2]) if len(args) >= 3 else None
                yaw = float(args[3]) if len(args) >= 4 else float("nan")
                gs = float(args[4]) if len(args) >= 5 else float("nan")
                print(module.reposition(lat, lon, alt, yaw, gs))
                continue
            print("Unknown command, type 'help'")
    except KeyboardInterrupt:
        pass
    finally:
        module.close()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MAVLink helper for JmavSim/PX4")
    parser.add_argument("--connect", default="udp:127.0.0.1:14540", help="Connection string for mavutil (e.g. udp:127.0.0.1:14540)")
    parser.add_argument("--baud", type=int, default=None, help="Serial baudrate if using a UART connection")
    sub = parser.add_subparsers(dest="command", required=False)

    monitor = sub.add_parser("monitor", help="Start telemetry loop and periodically print snapshots")
    monitor.add_argument("--interval", type=float, default=2.0, help="Print interval seconds")

    takeoff = sub.add_parser("takeoff", help="Arm and take off to altitude")
    takeoff.add_argument("altitude", type=float, help="Target altitude in meters")
    takeoff.add_argument("--yaw", type=float, default=float("nan"), help="Yaw angle in degrees (NaN to keep current)")

    sub.add_parser("land", help="Land at current location")
    sub.add_parser("rtl", help="Return to launch and land")

    goto = sub.add_parser("goto", help="Reposition using MAV_CMD_DO_REPOSITION")
    goto.add_argument("lat", type=float, help="Latitude in degrees")
    goto.add_argument("lon", type=float, help="Longitude in degrees")
    goto.add_argument("--alt", type=float, default=None, help="Altitude in meters (relative)")
    goto.add_argument("--yaw", type=float, default=float("nan"), help="Yaw angle in degrees")
    goto.add_argument("--groundspeed", type=float, default=float("nan"), help="Groundspeed in m/s")

    sub.add_parser("interactive", help="Interactive prompt with telemetry printing")
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    module = MavlinkModule(args.connect, baud=args.baud)

    if args.command is None or args.command == "interactive":
        _interactive_loop(module, print_interval=2.0)
        return

    module.start()
    try:
        if args.command == "monitor":
            try:
                while True:
                    module.print_snapshot()
                    time.sleep(args.interval)
            except KeyboardInterrupt:
                pass
        elif args.command == "takeoff":
            module.arm()
            ack = module.takeoff(args.altitude, yaw_deg=args.yaw)
            print(ack)
        elif args.command == "land":
            print(module.land())
        elif args.command == "rtl":
            print(module.return_to_launch())
        elif args.command == "goto":
            ack = module.reposition(
                args.lat,
                args.lon,
                alt_m=args.alt,
                yaw_deg=args.yaw,
                groundspeed=args.groundspeed,
            )
            print(ack)
    finally:
        module.close()


if __name__ == "__main__":
    main()
