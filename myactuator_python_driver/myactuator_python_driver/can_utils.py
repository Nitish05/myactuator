"""
CAN utilities for MyActuator Python Driver.

Provides CAN interface discovery and motor scanning functionality.
"""

import socket
import struct
import subprocess
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple
from enum import IntEnum


class CommandType(IntEnum):
    """MyActuator RMD command types."""
    READ_MOTOR_STATUS_1 = 0x9A
    READ_MOTOR_STATUS_2 = 0x9C
    READ_MOTOR_MODEL = 0xB5
    SHUTDOWN_MOTOR = 0x80


@dataclass
class MotorInfo:
    """Information about a discovered motor."""
    can_id: int
    temperature: int = 0
    voltage: float = 0.0
    error_code: int = 0
    model: str = ""


class CANScanner:
    """Scanner for discovering MyActuator RMD motors on a CAN bus."""

    CAN_REQUEST_BASE = 0x140
    CAN_RESPONSE_BASE = 0x240

    def __init__(self, ifname: str = "can0"):
        """Initialize the CAN scanner."""
        self.ifname = ifname
        self.socket: Optional[socket.socket] = None

    def _create_socket(self) -> bool:
        """Create and bind a raw CAN socket."""
        try:
            self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.socket.bind((self.ifname,))
            self.socket.settimeout(0.05)
            return True
        except OSError as e:
            print(f"Error creating CAN socket on {self.ifname}: {e}")
            self.socket = None
            return False

    def _close_socket(self):
        """Close the CAN socket."""
        if self.socket:
            self.socket.close()
            self.socket = None

    def _send_frame(self, can_id: int, data: bytes) -> bool:
        """Send a CAN frame."""
        if not self.socket:
            return False
        try:
            frame = struct.pack("=IB3x8s", can_id, len(data), data.ljust(8, b'\x00'))
            self.socket.send(frame)
            return True
        except OSError:
            return False

    def _receive_frame(self) -> Optional[Tuple[int, bytes]]:
        """Receive a CAN frame."""
        if not self.socket:
            return None
        try:
            frame = self.socket.recv(16)
            can_id, dlc = struct.unpack("=IB3x", frame[:8])
            data = frame[8:8+dlc]
            return can_id, data
        except socket.timeout:
            return None
        except OSError:
            return None

    def _clear_rx_buffer(self):
        """Clear any pending frames in the receive buffer."""
        if not self.socket:
            return
        self.socket.settimeout(0.001)
        try:
            while True:
                self.socket.recv(16)
        except socket.timeout:
            pass
        finally:
            self.socket.settimeout(0.05)

    def scan_motor(self, motor_id: int) -> Optional[MotorInfo]:
        """Probe a single motor ID."""
        if not self.socket:
            return None

        request_id = self.CAN_REQUEST_BASE + motor_id
        expected_response_id = self.CAN_RESPONSE_BASE + motor_id

        command = bytes([CommandType.READ_MOTOR_STATUS_1, 0, 0, 0, 0, 0, 0, 0])
        self._clear_rx_buffer()

        if not self._send_frame(request_id, command):
            return None

        for _ in range(3):
            result = self._receive_frame()
            if result:
                can_id, data = result
                if can_id == expected_response_id and len(data) >= 8:
                    if data[0] == CommandType.READ_MOTOR_STATUS_1:
                        temperature = data[1]
                        voltage = struct.unpack("<H", data[4:6])[0] * 0.1
                        error_code = struct.unpack("<H", data[6:8])[0]
                        return MotorInfo(
                            can_id=motor_id,
                            temperature=temperature,
                            voltage=voltage,
                            error_code=error_code
                        )
        return None

    def scan_all(self, id_range: Tuple[int, int] = (1, 32),
                 progress_callback=None) -> List[MotorInfo]:
        """Scan all motor IDs in the given range."""
        discovered = []
        start_id, end_id = id_range

        if not self._create_socket():
            return discovered

        try:
            total = end_id - start_id + 1
            for i, motor_id in enumerate(range(start_id, end_id + 1)):
                if progress_callback:
                    progress_callback(i + 1, total)

                info = self.scan_motor(motor_id)
                if info:
                    discovered.append(info)
                time.sleep(0.01)
        finally:
            self._close_socket()

        return discovered

    @staticmethod
    def get_available_interfaces() -> List[str]:
        """Get list of available CAN interfaces on the system."""
        interfaces = []
        
        # Check for real CAN interfaces
        try:
            result = subprocess.run(
                ['ip', 'link', 'show', 'type', 'can'],
                capture_output=True, text=True, timeout=5
            )
            for line in result.stdout.split('\n'):
                if ':' in line and '@' not in line:
                    parts = line.split(':')
                    if len(parts) >= 2:
                        iface = parts[1].strip()
                        if iface:
                            interfaces.append(iface)
        except Exception:
            pass

        # Check for virtual CAN interfaces
        try:
            result = subprocess.run(
                ['ip', 'link', 'show', 'type', 'vcan'],
                capture_output=True, text=True, timeout=5
            )
            for line in result.stdout.split('\n'):
                if ':' in line and '@' not in line:
                    parts = line.split(':')
                    if len(parts) >= 2:
                        iface = parts[1].strip()
                        if iface and iface not in interfaces:
                            interfaces.append(iface)
        except Exception:
            pass

        return sorted(interfaces)

    @staticmethod
    def is_interface_up(ifname: str) -> bool:
        """Check if a CAN interface is up."""
        try:
            result = subprocess.run(
                ['ip', 'link', 'show', ifname],
                capture_output=True, text=True, timeout=5
            )
            return 'state UP' in result.stdout or ',UP' in result.stdout
        except Exception:
            return False
