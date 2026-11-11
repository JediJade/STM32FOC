"""
The two wire formats the firmware speaks besides its console, with no UI and
no hardware: the one-line dashboard signal and the CAN frames.

Both are written down once here and once in the firmware (src/oneline.c,
src/canbus.c). fwsim.py builds its frames with these functions, the window
decodes what the board recorded with them, and the checks compare the two.

ONE LINE (SIF, "yi xian tong"): one wire, one direction, drive to dashboard.

    idle      low
    sync      31 T low, 1 T high
    bit 0     2 T low, 1 T high        bit 1     1 T low, 2 T high
    frame     12 bytes, most significant bit first, then low again
    T         a parameter (olt, in 50 us steps); 0.5 ms as built

    byte  0   0x08 device code          byte  1   0x61 sequence code
    byte  2   faults   bit6 hall, bit5 throttle, bit4 controller
    byte  3   state    bit6 motor turning, bit5 brake, bit3 regenerating
    byte  4   state    bit2 reverse, bit0 speed limit active
    byte  5   torque current, |iq| / 128
    byte  6   0
    byte  7,8 speed as Hall transitions per half second, high byte first
    byte  9   battery, 0: this firmware does not measure a bus voltage
    byte 10   throttle, percent
    byte 11   exclusive-or of bytes 0 to 10

Dashboards do not agree on the byte layout; this is the common one, and it
lives in one function on each side so it can be changed in one place.

CAN: 11-bit identifiers, node number n (canid), little-endian fields.

    0x100+n  to the drive   command   [mode, flags, value lo, value hi]
                 mode 0 off, 1 torque current, 2 speed, 3 throttle (0..32767),
                 0x55 go back to the USB console. flags bit0 = brake.
    0x200+n  to the drive   parameter [op, index, 0, 0, value as int32]
                 op 1 read, 2 write
    0x300+n  from the drive status 1  speed, iq, id (int16 each), state, fault
    0x400+n  from the drive status 2  vd, vq (int16), angle (uint16),
                 throttle / 128, flags
    0x500+n  from the drive parameter reply [op | 0x80, index, 0, 0, value]
    0x700+n  from the drive heartbeat [console version, vehicle state]
"""

import struct

OL_SYNC_LOW = 31
OL_SYNC_HIGH = 1
OL_GAP = 20
OL_BYTES = 12

CAN_CMD = 0x100
CAN_PARAM = 0x200
CAN_STATUS1 = 0x300
CAN_STATUS2 = 0x400
CAN_REPLY = 0x500
CAN_HEARTBEAT = 0x700
CAN_MODE_OFF, CAN_MODE_TORQUE, CAN_MODE_SPEED, CAN_MODE_THROTTLE = 0, 1, 2, 3
CAN_MODE_USB = 0x55


# ------------------------------------------------------------- one line ---

def ol_frame(speed, iq, throttle, brake, regen, limit, hall_fault,
             throttle_fault, controller_fault):
    """The 12 bytes. speed in angle units per ms, iq in Q15 counts,
    throttle 0..32767; the rest are flags."""
    a = abs(speed)
    halls = (a * 3000) >> 16            # 6 transitions per turn, per 0.5 s
    b = [0x08, 0x61,
         (0x40 if hall_fault else 0) | (0x20 if throttle_fault else 0)
         | (0x10 if controller_fault else 0),
         (0x40 if a > 20 else 0) | (0x20 if brake else 0) | (0x08 if regen else 0),
         (0x04 if speed < -20 else 0) | (0x01 if limit else 0),
         min(255, abs(iq) >> 7), 0,
         (halls >> 8) & 0xFF, halls & 0xFF,
         0,
         (throttle * 100 + 16384) >> 15]
    x = 0
    for v in b:
        x ^= v
    return b + [x]


def ol_runs(frame, t):
    """The frame as the recorder stores it: one number per stretch of
    constant level, in 50 us periods, negative while the line is low."""
    runs = [-OL_SYNC_LOW * t, OL_SYNC_HIGH * t]
    for byte in frame:
        for bit in range(7, -1, -1):
            if (byte >> bit) & 1:
                runs += [-t, 2 * t]
            else:
                runs += [-2 * t, t]
    return runs


def ol_decode(runs):
    """Runs back to a frame. Returns a dict: ok, why, t (periods), bytes and
    the fields. Tolerant of the timing, strict about the structure."""
    out = {"ok": False, "why": "", "bytes": [], "t": 0.0}
    if len(runs) < 2 + 16 * OL_BYTES:
        out["why"] = f"only {len(runs)} level changes, a frame has {2 + 16 * OL_BYTES}"
        return out
    if not (runs[0] < 0 < runs[1]) or -runs[0] < 8 * runs[1]:
        out["why"] = "no sync: it should start with a long low and a short high"
        return out
    body = runs[2:2 + 16 * OL_BYTES]
    total = sum(abs(r) for r in body)
    t = total / (3.0 * 8 * OL_BYTES)
    out["t"] = t
    data = []
    for i in range(OL_BYTES):
        byte = 0
        for k in range(8):
            low, high = body[(i * 8 + k) * 2], body[(i * 8 + k) * 2 + 1]
            if low >= 0 or high <= 0:
                out["why"] = f"bit {i * 8 + k} is not a low followed by a high"
                return out
            if abs(-low + high - 3 * t) > 0.6 * t:
                out["why"] = f"bit {i * 8 + k} is {(-low + high) / t:.1f} T long, not 3"
                return out
            byte = (byte << 1) | (1 if high > -low else 0)
        data.append(byte)
    out["bytes"] = data
    x = 0
    for v in data[:-1]:
        x ^= v
    if x != data[-1]:
        out["why"] = f"checksum is 0x{data[-1]:02x}, the bytes give 0x{x:02x}"
        return out
    halls = (data[7] << 8) | data[8]
    out.update(ok=True,
               halls=halls,
               speed=halls * 65536 // 3000,          # angle units per ms
               current=data[5] << 7,
               throttle=data[10],
               hall_fault=bool(data[2] & 0x40),
               throttle_fault=bool(data[2] & 0x20),
               controller_fault=bool(data[2] & 0x10),
               turning=bool(data[3] & 0x40),
               brake=bool(data[3] & 0x20),
               regen=bool(data[3] & 0x08),
               reverse=bool(data[4] & 0x04),
               limit=bool(data[4] & 0x01))
    return out


# ------------------------------------------------------------------ CAN ---

def can_command(node, mode, value=0, brake=False):
    return (CAN_CMD + node, struct.pack("<BBhxxxx", mode, 1 if brake else 0, value))


def can_param(node, op, index, value=0):
    return (CAN_PARAM + node, struct.pack("<BBxxi", op, index, value))


def can_status1(node, speed, iq, i_d, state, fault):
    return (CAN_STATUS1 + node, struct.pack("<hhhBB", speed, iq, i_d, state, fault))


def can_status2(node, vd, vq, angle, throttle, flags):
    return (CAN_STATUS2 + node,
            struct.pack("<hhHBB", vd, vq, angle & 0xFFFF, throttle >> 7, flags))


def can_decode(ident, data, node=1):
    """One frame as (name, fields), or (None, {}) if it is not one of ours."""
    kind = ident - node
    data = bytes(data)
    if kind == CAN_CMD and len(data) >= 4:
        mode, flags, value = struct.unpack("<BBh", data[:4])
        return "command", {"mode": mode, "brake": bool(flags & 1), "value": value}
    if kind == CAN_PARAM and len(data) == 8:
        op, index, value = struct.unpack("<BBxxi", data)
        return "parameter", {"op": op, "index": index, "value": value}
    if kind == CAN_STATUS1 and len(data) == 8:
        speed, iq, i_d, state, fault = struct.unpack("<hhhBB", data)
        return "status 1", {"speed": speed, "iq": iq, "id": i_d, "loop": state & 3,
                            "outer": (state >> 2) & 3, "sensor": (state >> 4) & 3,
                            "outputs": (state >> 6) & 1, "vehicle": (state >> 7) & 1,
                            "fault": fault & 15, "vehicle_state": fault >> 4}
    if kind == CAN_STATUS2 and len(data) == 8:
        vd, vq, angle, throttle, flags = struct.unpack("<hhHBB", data)
        return "status 2", {"vd": vd, "vq": vq, "angle": angle, "throttle": throttle << 7,
                            "brake": bool(flags & 1), "weakening": bool(flags & 2),
                            "hall": (flags >> 2) & 7, "phase": flags >> 5}
    if kind == CAN_REPLY and len(data) == 8:
        op, index, value = struct.unpack("<BBxxi", data)
        return "reply", {"op": op & 0x7F, "error": op == 0xFF, "index": index, "value": value}
    if kind == CAN_HEARTBEAT and len(data) >= 2:
        return "heartbeat", {"console": data[0], "vehicle_state": data[1]}
    return None, {}


def selfcheck():
    rows = []
    f = ol_frame(3280, 4000, 16384, True, True, False, False, True, False)
    d = ol_decode(ol_runs(f, 10))
    rows.append(("one-line frame survives its own waveform",
                 d["ok"] and d["bytes"] == f and abs(d["t"] - 10) < 0.01, d["why"]))
    rows.append(("and its fields come back", d.get("brake") and d.get("regen")
                 and d.get("throttle_fault") and d.get("throttle") == 50
                 and abs(d.get("speed", 0) - 3280) < 30, str(d.get("speed"))))
    bad = ol_runs(f, 10)
    bad[40] -= 10
    bad[41] -= 10
    rows.append(("a flipped bit fails the checksum", not ol_decode(bad)["ok"],
                 ol_decode(bad)["why"]))
    ident, data = can_status1(1, -1200, 300, -5, 0x96, 0x21)
    name, fields = can_decode(ident, data)
    rows.append(("CAN status 1 round trip", name == "status 1" and fields["speed"] == -1200
                 and fields["sensor"] == 1 and fields["vehicle"] == 1
                 and fields["vehicle_state"] == 2, str(fields)))
    name, fields = can_decode(*can_command(1, CAN_MODE_THROTTLE, 20000, brake=True))
    rows.append(("CAN command round trip", name == "command" and fields["value"] == 20000
                 and fields["brake"], str(fields)))
    return rows


if __name__ == "__main__":
    bad = 0
    for name, ok, detail in selfcheck():
        print(("ok    " if ok else "FAIL  ") + f"{name:44s} {detail}")
        bad += not ok
    raise SystemExit(1 if bad else 0)
