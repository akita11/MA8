#!/usr/bin/env python3
"""MA8 USB-serial receiver: binary packets -> WAV

usage:
    pip install pyserial numpy
    python ma8_serial_rx.py COM5 -t 10 -o rec.wav
    python ma8_serial_rx.py COM5 -t 3 --sync-check      # A/B alignment check (CH1 vs CH5)
    python ma8_serial_rx.py COM5 --info

Packet format: see src/serial_stream.h
"""
import argparse
import json
import struct
import sys
import time
import wave
import zlib

import serial

MAGIC = b"MA8D"
HDR = struct.Struct("<4sIIHBB")  # magic, seq, rate, frames, channels, bytes


def read_info(ser):
    ser.reset_input_buffer()
    ser.write(b"i\n")
    t0 = time.time()
    while time.time() - t0 < 2:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("{"):
            return json.loads(line)
    return None


def receive(ser, seconds, on_packet):
    buf = bytearray()
    t_end = time.time() + seconds
    expected_seq = 0
    stats = dict(packets=0, lost=0, crc_err=0, frames=0)
    ser.write(b"s")
    try:
        while time.time() < t_end:
            chunk = ser.read(ser.in_waiting or 4096)
            if chunk:
                buf += chunk
            while True:
                i = buf.find(MAGIC)
                if i < 0:
                    del buf[: max(0, len(buf) - 3)]
                    break
                if i > 0:
                    del buf[:i]
                if len(buf) < HDR.size:
                    break
                magic, seq, rate, frames, ch, nbytes = HDR.unpack_from(buf, 0)
                plen = frames * ch * nbytes
                total = HDR.size + plen + 4
                if frames == 0 or frames > 4096 or ch not in (4, 8) or nbytes not in (2, 3, 4):
                    del buf[:4]  # false sync
                    continue
                if len(buf) < total:
                    break
                crc_rx = struct.unpack_from("<I", buf, HDR.size + plen)[0]
                if zlib.crc32(bytes(buf[: HDR.size + plen])) & 0xFFFFFFFF != crc_rx:
                    stats["crc_err"] += 1
                    del buf[:4]
                    continue
                if seq > expected_seq:
                    stats["lost"] += seq - expected_seq
                expected_seq = seq + 1
                on_packet(rate, ch, nbytes, bytes(buf[HDR.size : HDR.size + plen]))
                stats["packets"] += 1
                stats["frames"] += frames
                del buf[:total]
    finally:
        ser.write(b"x")
    return stats


def sync_check(raw, ch, nbytes):
    import numpy as np

    dt = {2: "<i2", 4: "<i4"}.get(nbytes)
    if nbytes == 3:
        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
        x = (b[:, 0].astype(np.int32) << 8 | b[:, 1].astype(np.int32) << 16 | b[:, 2].astype(np.int32) << 24) >> 8
    else:
        x = np.frombuffer(raw, dtype=dt).astype(np.int64)
    x = x.reshape(-1, ch).astype(np.float64)
    a = x[:, 0] - x[:, 0].mean()
    b = x[:, 4] - x[:, 4].mean()
    n = len(a)
    m = 32
    best = max(range(-m, m + 1), key=lambda k: np.dot(a[max(0, k) : n + min(0, k)], b[max(0, -k) : n - max(0, k)]))
    print(f"CH5 delay vs CH1 = {-best} samples  (0 = aligned; feed the same sound to MIC1 of A and B)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("-t", "--seconds", type=float, default=10)
    ap.add_argument("-o", "--out", default="ma8.wav")
    ap.add_argument("--info", action="store_true")
    ap.add_argument("--sync-check", action="store_true")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    ser.dtr = True
    time.sleep(0.2)
    ser.write(b"x")
    time.sleep(0.1)
    info = read_info(ser)
    print("device:", info)
    if args.info:
        return

    wav = None
    raw_all = bytearray()

    def on_packet(rate, ch, nbytes, payload):
        nonlocal wav
        if wav is None:
            wav = wave.open(args.out, "wb")
            wav.setnchannels(ch)
            wav.setsampwidth(nbytes)
            wav.setframerate(rate)
        wav.writeframesraw(payload)
        if args.sync_check:
            raw_all.extend(payload)

    st = receive(ser, args.seconds, on_packet)
    if wav:
        wav.close()
    print(f"packets={st['packets']} frames={st['frames']} lost_packets={st['lost']} crc_err={st['crc_err']}")
    if info and st["frames"]:
        print(f"saved {args.out}: {st['frames'] / info['rate']:.2f} s")
    if args.sync_check and raw_all and info:
        if info["channels"] == 8:
            sync_check(bytes(raw_all), 8, info["bytes"])
        else:
            print("sync check needs 8ch (device is built with SINGLE_ES7210)")


if __name__ == "__main__":
    sys.exit(main())
