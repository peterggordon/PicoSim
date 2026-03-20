# network.py
# FINS communication module (UDP)
#
# Notes:
# - Adds a lock so polling and manual commands share one UDP socket safely.
# - write_word() now supports selecting the memory area (DM vs CIO, etc.).
# - Success for writes is based on the FINS end code (0x0000), not just "a response arrived".

import socket
import threading
from typing import List, Optional


class FINSClient:
    def __init__(self, local_ip: str = "192.168.1.2", plc_ip: str = "192.168.1.3", port: int = 9600, *, debug: bool = False):
        # Note: PC IP is hardcoded; will need to be made configurable eventually.
        self.local_ip = local_ip
        self.plc_ip = plc_ip
        self.port = port
        self.debug = debug

        # Node numbers are the last octet of the IPs in your current setup.
        self.pc_node = 2
        self.plc_node = 3
        self.plc_address = (self.plc_ip, self.port)

        # Serialize send/recv on the single UDP socket (polling + manual writes).
        self._io_lock = threading.Lock()

        self.sock = None
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.local_ip, 0))  # bind to any available local port
            self.sock.settimeout(1.0)
        except OSError as e:
            print(f"[FINSClient] Socket bind failed: {e}")
            self.sock = None

    # ----------------------------
    # Helpers
    # ----------------------------

    @staticmethod
    def _end_code_ok(resp: bytes) -> bool:
        """
        FINS/UDP frame (command frame) layout:
          - 10 bytes header (ICF..SID)
          - 2 bytes command code (MRC,SRC)
          - 2 bytes end code (EC1,EC2)
          - ...data...

        End code 0x0000 means success.
        """
        if not resp or len(resp) < 14:
            return False
        ec1 = resp[12]
        ec2 = resp[13]
        return (ec1 == 0x00) and (ec2 == 0x00)

    @staticmethod
    def _area_code(area: str) -> int:
        """
        Common FINS memory-area codes for word access.
        Extend here as you add more mappings.
        """
        area = (area or "").upper()
        if area == "DM":
            return 0x82
        if area == "CIO":
            return 0xB0
        # Safe default (keeps older behavior)
        return 0x82

    # ----------------------------
    # Reads
    # ----------------------------

    def read_cio_words(self, start_word: int, count: int, sid: int = 0x43) -> Optional[List[int]]:
        """
        Read 'count' CIO words starting at CIO[start_word].
        Returns: list[int] of length 'count', or None on timeout/error.
        """
        if not self.sock:
            return None
        if count <= 0:
            return []

        fins_header = bytes([
            0x80, 0x00, 0x02,
            0x00, self.plc_node, 0x00,
            0x00, self.pc_node, 0x00,
            sid & 0xFF
        ])

        read_command = bytes([
            0x01, 0x01,                    # Memory Area Read
            0xB0,                          # CIO area
            (start_word >> 8) & 0xFF,      # Start address (word) high
            start_word & 0xFF,             # Start address (word) low
            0x00,                          # Bit offset
            (count >> 8) & 0xFF,           # Word count high
            count & 0xFF                   # Word count low
        ])

        packet = fins_header + read_command

        try:
            with self._io_lock:
                self.sock.sendto(packet, self.plc_address)
                data, _ = self.sock.recvfrom(2048)

            if not self._end_code_ok(data):
                return None

            # Payload starts after 10(header)+2(cmd)+2(end)=14 bytes
            payload = data[14:14 + (2 * count)]
            if len(payload) < (2 * count):
                return None

            out = []
            for i in range(count):
                out.append(int.from_bytes(payload[2 * i:2 * i + 2], byteorder="big", signed=False))
            return out

        except socket.timeout:
            return None
        except OSError:
            return None

    def read_words(self, start_address: int, count: int, *, area: str = "DM", sid: int = 0x43) -> Optional[List[int]]:
        """Read a block of words from a given memory area.

        This is the general form of read_cio_words(). It is used for reading
        DM (D) words (e.g., heater limits) and CIO words (e.g., TE2 setpoint).

        Args:
            start_address: Word address (e.g., 16 for D16, 122 for CIO122).
            count: Number of words to read.
            area: "DM" or "CIO".
        """
        if not self.sock:
            return None
        if count <= 0:
            return []

        area_code = self._area_code(area)

        fins_header = bytes([
            0x80, 0x00, 0x02,
            0x00, self.plc_node, 0x00,
            0x00, self.pc_node, 0x00,
            sid & 0xFF
        ])

        read_command = bytes([
            0x01, 0x01,                    # Memory Area Read
            area_code,
            (start_address >> 8) & 0xFF,
            start_address & 0xFF,
            0x00,                          # Bit offset
            (count >> 8) & 0xFF,
            count & 0xFF
        ])

        packet = fins_header + read_command

        try:
            with self._io_lock:
                self.sock.sendto(packet, self.plc_address)
                data, _ = self.sock.recvfrom(2048)

            if not self._end_code_ok(data):
                return None

            payload = data[14:14 + (2 * count)]
            if len(payload) < (2 * count):
                return None

            out: List[int] = []
            for i in range(count):
                out.append(int.from_bytes(payload[2 * i:2 * i + 2], byteorder="big", signed=False))
            return out

        except socket.timeout:
            return None
        except OSError:
            return None

    def read_word(self, address: int, *, area: str = "DM", sid: int = 0x43) -> Optional[int]:
        """Convenience wrapper to read a single word."""
        words = self.read_words(address, 1, area=area, sid=sid)
        if not words:
            return None
        return words[0]

    def read_snapshot(self) -> Optional[bytes]:
        """
        Read the CIO110 block used for the GUI snapshot (38 words).
        """
        if not self.sock:
            return None

        fins_header = bytes([
            0x80, 0x00, 0x02,
            0x00, self.plc_node, 0x00,
            0x00, self.pc_node, 0x00,
            0x42  # SID
        ])
        read_command = bytes([
            0x01, 0x01,       # Memory Area Read
            0xB0,             # CIO area
            0x00, 0x6E,       # Starting address: CIO110
            0x00,             # Bit offset
            0x00, 0x26        # Read 38 words
        ])
        packet = fins_header + read_command

        try:
            with self._io_lock:
                self.sock.sendto(packet, self.plc_address)
                data, _ = self.sock.recvfrom(2048)

            if self.debug:
                print(f"[DEBUG] Received {len(data)} bytes: {' '.join(f'{b:02X}' for b in data)}")

            if not self._end_code_ok(data):
                return None
            return data

        except socket.timeout:
            if self.debug:
                print("[DEBUG] Timeout waiting for response from PLC")
            return None
        except OSError:
            return None

    # ----------------------------
    # Writes
    # ----------------------------

    def write_word(self, address: int, value: int, *, area: str = "DM", sid: int = 0x5A) -> bool:
        """
        Write a single word to the requested memory area.

        area:
          - "DM"  -> DM word area (0x82)
          - "CIO" -> CIO word area (0xB0)
        """
        if not self.sock:
            return False

        area_code = self._area_code(area)

        header = bytes([
            0x80, 0x00, 0x02,
            0x00, self.plc_node, 0x00,
            0x00, self.pc_node, 0x00,
            sid & 0xFF
        ])
        command = bytes([
            0x01, 0x02,       # Memory Area Write
            area_code,        # Memory area
            (address >> 8) & 0xFF,
            address & 0xFF,
            0x00,             # Bit offset
            0x00, 0x01        # Write 1 word
        ])
        value_bytes = int(value).to_bytes(2, byteorder="big", signed=False)
        packet = header + command + value_bytes

        try:
            with self._io_lock:
                self.sock.sendto(packet, self.plc_address)
                data, _ = self.sock.recvfrom(2048)
            return self._end_code_ok(data)
        except socket.timeout:
            return False
        except OSError:
            return False

    def write_bit(self, address: int, bit: int, value: int, sid: int = 0x5A) -> bool:
        """
        Write a single bit in the CIO bit area (0x30).
        """
        if not self.sock:
            return False

        header = bytes([
            0x80, 0x00, 0x02,
            0x00, self.plc_node, 0x00,
            0x00, self.pc_node, 0x00,
            sid & 0xFF
        ])
        command = bytes([
            0x01, 0x02,        # Memory Area Write
            0x30,              # CIO bit area
            (address >> 8) & 0xFF,
            address & 0xFF,
            bit & 0xFF,
            0x00, 0x01         # 1 bit
        ])
        value_bytes = bytes([0x01 if value else 0x00])
        packet = header + command + value_bytes

        try:
            with self._io_lock:
                self.sock.sendto(packet, self.plc_address)
                data, _ = self.sock.recvfrom(2048)
            return self._end_code_ok(data)
        except socket.timeout:
            return False
        except OSError:
            return False

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            finally:
                self.sock = None
