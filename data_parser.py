# data_parser.py
# Parses 132-byte FINS snapshot packets into labeled and scaled data

FIELD_MAP = {
    # Byte offset (zero-based index) : (Label, Unit, Scale)
    14:  ("MFC1", "sccm", 1),
    16:  ("MFC100", "sccm", 1),
    18:  ("MFC200", "sccm", 1),
    66:  ("MFC300", "sccm", 1),
    76:  ("MFC400", "sccm", 1),
    78:  ("MFM420", "sccm", 1),

    32:  ("MFC1 Setpoint", "sccm", 1),
    34:  ("MFC100 Setpoint", "sccm", 1),
    36:  ("MFC200 Setpoint", "sccm", 1),
    40:  ("MFC300 Setpoint", "sccm", 1),
    46:  ("MFC400 Setpoint", "sccm", 1),

    24:  ("PT1", "hPa", 1.0),
    26:  ("PT2", "hPa", 0.1),
    28:  ("PT100", "hPa", 0.1),
    30:  ("PT200", "hPa", 0.1),
    70:  ("PT300", "hPa", 0.1),
    80:  ("PT400", "hPa", 0.1),
    82:  ("PT420", "hPa", 1.0),

    # NOTE (user-verified): TE1 and TE2 were swapped in the GUI.
    # Swap these offsets so TE1 (chamber heater) and TE2 (chamber) display correctly.
    20:  ("TE1", "°C", 0.1),
    22:  ("TE2", "°C", 0.1),
    54:  ("TE110", "°C", 0.1),
    56:  ("TE210", "°C", 0.1),
    60:  ("TE310", "°C", 0.1),
    62:  ("TE211", "°C", 0.1),
}

def parse_snapshot(data_bytes):
    result = {}

    # Expected response size for 38 words:
    # 10-byte FINS header + 2-byte command + 2-byte end code + 76-byte data = 90 bytes
    if data_bytes is None or len(data_bytes) < 90:
        result["error"] = f"Packet too short: {0 if data_bytes is None else len(data_bytes)} bytes"
        return result

    # Validate end code (bytes 12:14) == 0x0000
    end = data_bytes[12:14]
    if end != b"\x00\x00":
        result["error"] = f"PLC end code not OK: {end.hex()}"
        return result

    # Data payload starts at byte 14
    payload = data_bytes[14:]
    if len(payload) < 76:
        result["error"] = f"Payload too short for 38 words: {len(payload)} bytes"
        return result

    # FIELD_MAP indices might be:
    #   A) payload-relative (0..75)
    #   B) frame-relative   (14..89 or similar)
    max_idx = max(FIELD_MAP.keys()) if FIELD_MAP else 0
    use_payload_relative = (max_idx < 76)

    base = 14 if use_payload_relative else 0

    for byte_index, (label, unit, scale) in FIELD_MAP.items():
        i = base + byte_index
        if i + 1 >= len(data_bytes):
            # If this ever triggers, the frame is malformed; treat as error to avoid UI flicker.
            result["error"] = f"Index out of range for {label}: {i} (frame len={len(data_bytes)})"
            return result

        raw = data_bytes[i:i + 2]
        value = int.from_bytes(raw, byteorder="big", signed=False)
        scaled = value * scale
        result[label] = round(scaled, 1 if scale < 1 else 0)

    return result