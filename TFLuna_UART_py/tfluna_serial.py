#!/usr/bin/env python3
"""
TF‑Luna UART serial utility

This utility can:
- Read continuous 9-byte TF‑Luna data frames (header 0x59 0x59) and print parsed values.
- Send TF‑Luna command frames (header 0x5A) and optionally wait for a response.

Usage examples:
  Read continuous measurements:
    python3 tfluna_serial.py --port /dev/ttyUSB0

  Send a single trigger command and wait for response:
    python3 tfluna_serial.py --port /dev/ttyUSB0 --cmd trigger

  Set output frequency to 100Hz:
    python3 tfluna_serial.py --port /dev/ttyUSB0 --cmd set-freq 100

Dependencies:
  pip install -r requirements.txt
"""

import argparse
import serial
import struct
import sys
import time
import signal
from typing import Optional, Tuple

EXIT = False
FRAME_UNITS = 'cm'  # 'cm' or 'mm' — set from args


def signal_handler(sig, frame):
    global EXIT
    EXIT = True


signal.signal(signal.SIGINT, signal_handler)


def parse_data_frame(frame: bytes) -> Optional[dict]:
    """Parse a standard 9-byte data frame (header 0x59 0x59).

    Returns a dict or None on checksum/header failure.
    """
    if len(frame) != 9:
        return None
    if frame[0] != 0x59 or frame[1] != 0x59:
        return None
    chk = sum(frame[0:8]) & 0xFF
    if chk != frame[8]:
        return None

    dist = frame[2] | (frame[3] << 8)
    strength = frame[4] | (frame[5] << 8)
    temp_raw = frame[6] | (frame[7] << 8)
    # Per datasheet: Temperature (°C) = (Temp_H<<8 | Temp_L)/8 - 256
    temp_val = (frame[7] << 8) | frame[6]
    temp_c = temp_val / 8.0 - 256.0

    # Interpret distance according to configured units
    if FRAME_UNITS == 'mm':
        distance_raw = dist
        unit = 'mm'
        distance_mm = dist
        distance_m = dist / 1000.0
    else:
        # assume 'cm'
        distance_raw = dist
        unit = 'cm'
        distance_mm = dist * 10
        distance_m = dist / 100.0

    return {
        'distance_raw': distance_raw,
        'unit': unit,
        'distance_mm': distance_mm,
        'distance_m': distance_m,
        'strength': strength,
        'temp_raw': temp_raw,
        'temp_c': temp_c,
        'frame': frame,
    }


def build_command(cmd_id: int, payload: bytes = b'') -> bytes:
    """Build a TF-Luna command frame (header 0x5A).

    Frame structure:
      [0] Head = 0x5A
      [1] Len = total length of frame in bytes (including checksum)
      [2] ID
      [3..N] Payload
      [Len-1] Checksum = low 8 bits of sum(bytes[0..Len-2])
    """
    frame = bytearray()
    frame.append(0x5A)  # head
    frame.append(0x00)  # placeholder for len
    frame.append(cmd_id & 0xFF)
    frame.extend(payload)
    total_len = len(frame) + 1  # include checksum
    frame[1] = total_len & 0xFF
    checksum = sum(frame[0:total_len-1]) & 0xFF
    frame.append(checksum)
    return bytes(frame)


def parse_response_frames(buf: bytearray) -> Tuple[list, bytearray]:
    """Extract 0x5A response frames from buffer and return (frames, remaining_buf).

    This parser looks for frames that begin with 0x5A and uses the length byte to
    determine full frame size.
    """
    frames = []
    i = 0
    while i <= len(buf) - 2:
        if buf[i] != 0x5A:
            i += 1
            continue
        if i + 2 > len(buf):
            break
        length = buf[i+1]
        if length < 4:
            # minimum frame: head(1) len(1) id(1) checksum(1) => 4
            i += 1
            continue
        if i + length > len(buf):
            break
        frame = bytes(buf[i:i+length])
        # verify checksum
        chk = sum(frame[0:length-1]) & 0xFF
        if chk == frame[-1]:
            frames.append(frame)
        else:
            # append invalid frame as bytes for debugging
            frames.append(frame)
        i += length

    return frames, buf[i:]


def send_command(ser: serial.Serial, cmd_id: int, payload: bytes = b'') -> bytes:
    frame = build_command(cmd_id, payload)
    ser.write(frame)
    ser.flush()
    return frame


def wait_for_response(ser: serial.Serial, timeout: float = 1.0) -> list:
    """Read serial and return any 0x5A response frames received within timeout."""
    end = time.time() + timeout
    buf = bytearray()
    responses = []
    while time.time() < end:
        b = ser.read(ser.in_waiting or 1)
        if b:
            buf.extend(b)
            frames, buf = parse_response_frames(buf)
            if frames:
                responses.extend(frames)
    return responses


def open_serial(port: str, baud: int, timeout: float = 0.2) -> serial.Serial:
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        return ser
    except Exception as e:
        print(f'Failed to open serial port {port}: {e}', file=sys.stderr)
        raise


def main():
    p = argparse.ArgumentParser(description='TF-Luna UART utility')
    p.add_argument('--port', '-p', required=True, help='Serial port (e.g. /dev/ttyUSB0)')
    p.add_argument('--baud', '-b', type=int, default=115200, help='Serial baud rate (default: 115200)')
    p.add_argument('--units', choices=['cm','mm'], default='cm', help='Sensor output units; selects set-format and how frames are interpreted (default: cm)')
    p.add_argument('--hz', type=int, help='If set, send Set Output Frequency (Hz) before entering read loop; 0 = trigger mode')
    p.add_argument('--raw', action='store_true', help='Print raw frame hex on each valid/invalid frame')
    p.add_argument('--timeout', type=float, default=0.2, help='Serial read timeout (seconds)')
    p.add_argument('--cmd', nargs='+', help='Send a command then exit. Examples: trigger | set-freq 100 | set-format 1 | set-baud 115200 | save | restore | enable | disable')
    args = p.parse_args()

    ser = None
    try:
        ser = open_serial(args.port, args.baud, timeout=args.timeout)
    except Exception:
        sys.exit(2)

    # set global FRAME_UNITS according to args
    global FRAME_UNITS
    FRAME_UNITS = args.units

    # Send Set Output Format based on requested units
    try:
        if args.units == 'mm':
            fmt_id = 0x06
            fmt_name = '9-byte/mm'
        else:
            fmt_id = 0x01
            fmt_name = '9-byte/cm'
        out = send_command(ser, 0x05, bytes([fmt_id]))
        print(f'Sent set-format ({fmt_name}) command: {out.hex()}')
        resp = wait_for_response(ser, timeout=0.5)
        fmt_ack = False
        for r in resp:
            if len(r) >= 4 and r[0] == 0x5A:
                rid = r[2]
                payload = r[3:-1]
                if rid == 0x05:
                    print('Received format response:', r.hex())
                    fmt_ack = True
                    break
        if not fmt_ack:
            print('No explicit response for set-format received (sensor may not reply).')
    except Exception:
        print('Failed to send set-format command (ignored).')

    if args.cmd:
        # process single command and exit
        cmd = args.cmd[0]
        try:
            if cmd == 'trigger':
                out = send_command(ser, 0x04, b'')
                print('Sent trigger:', out.hex())
                resp = wait_for_response(ser, timeout=1.0)
                for r in resp:
                    print('Response:', r.hex())
            elif cmd == 'set-freq':
                if len(args.cmd) < 2:
                    print('set-freq requires a frequency value (Hz)')
                else:
                    hz = int(args.cmd[1])
                    payload = struct.pack('<H', hz)
                    out = send_command(ser, 0x03, payload)
                    print('Sent set-freq:', out.hex())
                    print('Waiting for response...')
                    for r in wait_for_response(ser, timeout=1.0):
                        print('Response:', r.hex())
            elif cmd == 'set-format':
                if len(args.cmd) < 2:
                    print('set-format requires a format id (1=9-byte/cm,2=PIX,6=9-byte/mm)')
                else:
                    fid = int(args.cmd[1])
                    out = send_command(ser, 0x05, bytes([fid]))
                    print('Sent set-format:', out.hex())
                    for r in wait_for_response(ser, timeout=1.0):
                        print('Response:', r.hex())
            elif cmd == 'set-baud':
                if len(args.cmd) < 2:
                    print('set-baud requires a baud value (e.g. 115200)')
                else:
                    baud = int(args.cmd[1])
                    # 4-byte little endian value
                    payload = struct.pack('<I', baud)
                    out = send_command(ser, 0x06, payload)
                    print('Sent set-baud:', out.hex())
                    print('Waiting for response...')
                    for r in wait_for_response(ser, timeout=1.0):
                        print('Response:', r.hex())
                    print('Note: If the sensor accepted the baud change you must re-open the serial port at the new baud rate to continue communicating.')
            elif cmd == 'save':
                out = send_command(ser, 0x11, b'')
                print('Sent save settings:', out.hex())
                for r in wait_for_response(ser, timeout=1.0):
                    print('Response:', r.hex())
            elif cmd == 'restore':
                out = send_command(ser, 0x10, b'')
                print('Sent restore defaults:', out.hex())
                for r in wait_for_response(ser, timeout=1.0):
                    print('Response:', r.hex())
            elif cmd == 'enable':
                out = send_command(ser, 0x07, bytes([0x01]))
                print('Sent enable output:', out.hex())
                for r in wait_for_response(ser, timeout=1.0):
                    print('Response:', r.hex())
            elif cmd == 'disable':
                out = send_command(ser, 0x07, bytes([0x00]))
                print('Sent disable output:', out.hex())
                for r in wait_for_response(ser, timeout=1.0):
                    print('Response:', r.hex())
            else:
                print('Unknown command:', cmd)
        finally:
            try:
                ser.close()
            except Exception:
                pass
        return

    # If the user requested a frequency, send Set Output Frequency before entering the read loop
    if args.hz is not None:
        try:
            hz = int(args.hz)
            payload = struct.pack('<H', hz)
            out = send_command(ser, 0x03, payload)
            print(f'Sent set-freq {hz}Hz command: {out.hex()}')
            resp = wait_for_response(ser, timeout=0.5)
            if resp:
                for r in resp:
                    print('Response:', r.hex())
            else:
                print('No response to set-freq (sensor may not reply).')
        except Exception:
            print('Failed to send set-freq command (ignored).')

    print(f'Opened {args.port} at {args.baud} baud. Waiting for TF-Luna data frames...')

    buf = bytearray()
    good = 0
    bad = 0

    while not EXIT:
        try:
            b = ser.read(1)
            if not b:
                continue
            buf += b
            # prevent runaway buffer
            if len(buf) > 1024:
                buf = buf[-256:]

            # look for data header 0x59 0x59 (data frames)
            if len(buf) >= 2:
                idx = None
                for i in range(len(buf) - 1):
                    if buf[i] == 0x59 and buf[i+1] == 0x59:
                        idx = i
                        break
                if idx is None:
                    # drop oldest
                    if len(buf) > 4:
                        buf = buf[-2:]
                    continue
                if len(buf) - idx < 9:
                    continue
                frame = bytes(buf[idx:idx+9])
                buf = buf[idx+9:]
                parsed = parse_data_frame(frame)
                if parsed is None:
                    bad += 1
                    if args.raw:
                        print('INVALID DATA FRAME:', frame.hex())
                    continue
                good += 1
                ts = time.time()
                if args.raw:
                    print('FRAME:', frame.hex())
                print(f'Distance: {parsed["distance_raw"]} {parsed["unit"]} ({parsed["distance_m"]:.3f} m)  Strength: {parsed["strength"]}  Temp: {parsed["temp_c"]:.2f} °C')

        except serial.SerialException as e:
            print('Serial error:', e, file=sys.stderr)
            break
        except Exception as e:
            print('Error:', e, file=sys.stderr)
            break

    print('\nExiting. Parsed frames:', good, '  Invalid frames:', bad)
    try:
        ser.close()
    except Exception:
        pass


if __name__ == '__main__':
    main()
