#!/usr/bin/env python3
"""
USB Audio デバイス（8chマイクアレイなど）から録音して WAV 形式で保存する。

必要なパッケージ:
    pip install sounddevice soundfile numpy

使い方の例:
    # デバイス一覧を表示
    python record_8ch.py --list

    # デバイス名の一部で指定して 10 秒録音（32kHz / 16bit / 8ch 固定）
    python record_8ch.py -d "MA8" -t 10 -o out.wav

    # デバイス番号で指定、時間指定なし（Ctrl+C で停止）
    python record_8ch.py -d 12

    # WASAPI 排他モード（Windows の音声処理を通さない。WASAPI のデバイス番号を指定）
    python record_8ch.py -d 12 -t 10 --exclusive

    # チャンネルごとに別ファイル（out_ch1.wav ～ out_ch8.wav）も書き出す
    python record_8ch.py -d "MA8" -t 5 --split
"""

import argparse
import queue
import sys
import time
from datetime import datetime

import numpy as np
import sounddevice as sd
import soundfile as sf

# 録音フォーマット（固定）
SAMPLE_RATE = 32000   # Hz (firmware SAMPLE_RATE_HZ)
BITS = 16
DTYPE = "int16"       # sounddevice の型
SUBTYPE = "PCM_16"    # soundfile の WAV サブタイプ


def list_devices():
    print("=== 入力デバイス一覧 ===")
    for i, dev in enumerate(sd.query_devices()):
        if dev["max_input_channels"] > 0:
            hostapi = sd.query_hostapis(dev["hostapi"])["name"]
            print(f"[{i:2d}] {dev['name']}  "
                  f"(in={dev['max_input_channels']}ch, "
                  f"default {dev['default_samplerate']:.0f}Hz, {hostapi})")


def find_device(spec, channels):
    """番号またはデバイス名の一部から入力デバイス番号を返す"""
    if spec is None:
        return None  # 既定の入力デバイス
    try:
        return int(spec)
    except ValueError:
        pass
    matches = [
        i for i, dev in enumerate(sd.query_devices())
        if spec.lower() in dev["name"].lower()
        and dev["max_input_channels"] >= channels
    ]
    if not matches:
        sys.exit(f"'{spec}' に一致し、{channels}ch 以上入力できるデバイスが見つかりません。"
                 " --list で確認してください。")
    if len(matches) > 1:
        print(f"複数一致したため [{matches[0]}] を使用します: {matches}")
    return matches[0]


def main():
    p = argparse.ArgumentParser(description="USB Audio 多チャンネル録音 → WAV")
    p.add_argument("--list", action="store_true", help="入力デバイス一覧を表示して終了")
    p.add_argument("-d", "--device", help="デバイス番号 または 名前の一部")
    p.add_argument("-c", "--channels", type=int, default=8, help="チャンネル数 (既定: 8)")
    p.add_argument("-t", "--duration", type=float, default=None,
                   help="録音秒数（省略時は Ctrl+C まで）")
    p.add_argument("-o", "--output", default=None,
                   help="出力ファイル名（既定: rec_YYYYmmdd_HHMMSS.wav）")
    p.add_argument("--split", action="store_true",
                   help="録音後、チャンネルごとのモノラル WAV も書き出す")
    p.add_argument("--blocksize", type=int, default=0,
                   help="1回のコールバックで受け取るフレーム数 (0=自動)")
    p.add_argument("--exclusive", action="store_true",
                   help="WASAPI 排他モード（Windows の共有ミキサー・音声効果を通さず、デバイスのデータをそのまま受け取る）")
    args = p.parse_args()

    if args.list:
        list_devices()
        return

    device = find_device(args.device, args.channels)
    dtype, subtype, rate = DTYPE, SUBTYPE, SAMPLE_RATE
    out = args.output or datetime.now().strftime("rec_%Y%m%d_%H%M%S.wav")

    extra = None
    if args.exclusive:
        hostapi = sd.query_hostapis(sd.query_devices(device, "input")["hostapi"])["name"]
        if "WASAPI" not in hostapi:
            sys.exit(f"--exclusive は WASAPI のデバイスでのみ使えます（指定デバイス: {hostapi}）")
        extra = sd.WasapiSettings(exclusive=True)

    # デバイスが指定条件に対応しているか事前チェック
    try:
        sd.check_input_settings(device=device, channels=args.channels,
                                samplerate=rate, dtype=dtype, extra_settings=extra)
    except Exception as e:
        sys.exit(f"デバイスがこの設定に対応していません: {e}")

    dev_name = sd.query_devices(device, "input")["name"]
    print(f"デバイス : {dev_name}")
    print(f"設定     : {args.channels}ch / {rate}Hz / {BITS}bit"
          + ("（WASAPI 排他モード）" if args.exclusive else ""))
    print(f"出力     : {out}")

    q = queue.Queue()
    status_log = []

    def callback(indata, frames, time_info, status):
        if status:
            status_log.append(str(status))  # オーバーフロー等を記録
        q.put(indata.copy())

    total_frames = int(args.duration * rate) if args.duration else None
    written = 0

    # 録音しながら逐次ファイルへ書き出す（長時間録音でもメモリを消費しない）
    with sf.SoundFile(out, mode="w", samplerate=rate,
                      channels=args.channels, subtype=subtype) as f:
        with sd.InputStream(device=device, channels=args.channels,
                            samplerate=rate, dtype=dtype, extra_settings=extra,
                            blocksize=args.blocksize, callback=callback):
            print("録音中... (Ctrl+C で停止)")
            t0 = time.time()
            try:
                while total_frames is None or written < total_frames:
                    # Windows ではタイムアウトなしの q.get() が Ctrl+C を
                    # 受け付けないため、短いタイムアウトで待機を繰り返す
                    try:
                        block = q.get(timeout=0.1)
                    except queue.Empty:
                        # データが一度も届かない場合は警告表示
                        if written == 0 and time.time() - t0 > 2.0:
                            print(f"\r{time.time() - t0:7.1f}s  デバイスからデータが届きません"
                                  "（probe_device.py で設定を確認してください）", end="")
                        continue
                    if total_frames is not None:
                        block = block[: total_frames - written]
                    f.write(block)
                    written += len(block)
                    # 簡易レベルメーター（各chのピーク dBFS）
                    full = np.iinfo(block.dtype).max
                    peak = np.abs(block).max(axis=0) / full
                    db = 20 * np.log10(np.maximum(peak, 1e-9))
                    meter = " ".join(f"{v:6.1f}" for v in db)
                    print(f"\r{time.time() - t0:7.1f}s  [{meter}] dBFS", end="")
            except KeyboardInterrupt:
                pass
        # ストリーム停止後、キューに残ったデータも書き出す
        while not q.empty() and (total_frames is None or written < total_frames):
            block = q.get()
            if total_frames is not None:
                block = block[: total_frames - written]
            f.write(block)
            written += len(block)

    print(f"\n保存完了: {out}  ({written / rate:.2f} 秒, {written} フレーム)")
    if status_log:
        print(f"警告: ストリームで {len(status_log)} 回の異常を検出 "
              f"(例: {status_log[0]})。blocksize を大きくすると改善する場合があります。")

    if args.split:
        data, sr = sf.read(out, dtype=dtype, always_2d=True)
        base = out[:-4] if out.lower().endswith(".wav") else out
        for ch in range(data.shape[1]):
            name = f"{base}_ch{ch + 1}.wav"
            sf.write(name, data[:, ch], sr, subtype=subtype)
        print(f"チャンネル別ファイルを {data.shape[1]} 個書き出しました: {base}_chN.wav")


if __name__ == "__main__":
    main()
