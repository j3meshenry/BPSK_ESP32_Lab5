#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UART Waveform Plotter for ESP32 TX/RX Logic Analyzer Simulation
Author: ChatGPT (2025)
"""

import re
import sys
import time
import threading
from collections import deque
from typing import List, Tuple, Optional

import serial
from serial.tools import list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# -------------------- CONFIG --------------------
TX_MON_PORT = None            # Leave None for auto-detect
RX_MON_PORT = None            # e.g. "COM7" if you have a second ESP32 receiver
TX_MON_BAUD = 115200
RX_MON_BAUD = 115200

UART_BAUD = 9600              # Serial2 link speed between ESP32s (TXD2/RXD2)
MAX_FRAMES = 5                # How many TX frames to capture

CSV_OUT_TX = "tx_waveform.csv"
CSV_OUT_RX = "rx_waveform.csv"
PNG_OUT    = "tx_rx_waveforms.png"
# ------------------------------------------------


# --- Auto-detect COM ports for ESP32/USB-UART ---
KNOWN_VIDPIDS = {
    ("10C4", "EA60"),  # Silicon Labs CP210x
    ("1A86", "7523"),  # CH340/CH341
    ("1A86", "55D4"),  # CH9102
    ("0403", "6001"),  # FTDI FT232
    ("303A", "1001"),  # Espressif S2/S3
    ("303A", "0002"),  # Espressif UART/JTAG
}


def pick_esp32_port(preferred=None):
    ports = list(list_ports.comports())
    if preferred:
        for p in ports:
            if p.device.upper() == preferred.upper():
                return p.device
    for p in ports:
        if p.vid and p.pid:
            vid = f"{p.vid:04X}"
            pid = f"{p.pid:04X}"
            if (vid, pid) in KNOWN_VIDPIDS:
                return p.device
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ["cp210", "ch340", "ch9102", "ftdi", "espressif", "usb serial"]):
            return p.device
    return ports[0].device if ports else None


# --- UART bitstream synthesis ---
def byte_to_bits_lsb_first(b: int) -> List[int]:
    return [(b >> i) & 1 for i in range(8)]


def synth_uart_bits_for_byte(b: int) -> List[int]:
    bits = [0]
    bits.extend(byte_to_bits_lsb_first(b))
    bits.append(1)
    return bits


def synth_uart_waveform(bytes_seq: List[int],
                        baud: int,
                        t0_us: int = 0,
                        interbyte_gap_us: int = 0) -> Tuple[List[float], List[int]]:
    bit_us = 1_000_000.0 / baud
    times_us = [float(t0_us)]
    levels = [1]
    t = float(t0_us)
    t += bit_us
    times_us.append(t)
    levels.append(1)
    for b in bytes_seq:
        bit_levels = synth_uart_bits_for_byte(b)
        for lvl in bit_levels:
            times_us.append(t)
            levels.append(lvl)
            t += bit_us
            times_us.append(t)
            levels.append(lvl)
        t += interbyte_gap_us or bit_us
        times_us.append(t)
        levels.append(1)
    return times_us, levels


# --- Regexes for ESP32 console parsing ---
TX_FRAME_RE = re.compile(
    r'\[SYNC\]\[Length\]\[Data0-7\]\[CheckSum\]\s*=\s*\[([0-9A-F]{2})\]\[([0-9A-F]{2})\]\[((?:[0-9A-F]{2}(?:\s*)?)*)\]\[([0-9A-F]{2})\]',
    re.IGNORECASE
)
TX_TIMING_RE = re.compile(
    r't_start\(us\)=(\d+)\s+t_end\(us\)=(\d+)\s+duration\(us\)=(\d+)',
    re.IGNORECASE
)
RX_HEX_RE = re.compile(
    r'RX_HEX:\s*([0-9A-F]{2}(?:\s+[0-9A-F]{2})*)',
    re.IGNORECASE
)


def parse_hex_bytes(blob: str) -> List[int]:
    blob = blob.strip()
    if not blob:
        return []
    return [int(x, 16) for x in re.split(r'\s+', blob) if x]


# --- Serial reader threads ---
class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, name: str):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.name = name
        self.queue = deque(maxlen=500)
        self.stop_flag = False
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            buf = bytearray()
            while not self.stop_flag:
                data = self.ser.read(512)
                if data:
                    buf.extend(data)
                    while b'\n' in buf:
                        line, _, buf = buf.partition(b'\n')
                        try:
                            s = line.decode('utf-8', errors='ignore').strip()
                        except Exception:
                            s = str(line)
                        self.queue.append(s)
                else:
                    time.sleep(0.01)
        except serial.SerialException as e:
            print(f"[{self.name}] Serial error:", e)
        finally:
            if self.ser:
                self.ser.close()

    def get_line(self, timeout=0.0):
        t0 = time.time()
        while True:
            if self.queue:
                return self.queue.popleft()
            if timeout and (time.time() - t0) > timeout:
                return None
            time.sleep(0.01)

    def stop(self):
        self.stop_flag = True


# --- Frame collection ---
def collect_frames():
    tx_port = TX_MON_PORT or pick_esp32_port()
    if not tx_port:
        print("❌ No TX port found. Check USB connection or driver.")
        sys.exit(1)
    print("✅ Using TX port:", tx_port)

    tx_reader = SerialReader(tx_port, TX_MON_BAUD, "TX_MON")
    tx_reader.start()

    rx_reader = None
    if RX_MON_PORT:
        rx_port = RX_MON_PORT or pick_esp32_port()
        print("✅ Using RX port:", rx_port)
        rx_reader = SerialReader(rx_port, RX_MON_BAUD, "RX_MON")
        rx_reader.start()

    frames_tx, frames_rx = [], []

    try:
        print("\nListening for TX frames... Press Ctrl+C to stop.\n")
        while len(frames_tx) < MAX_FRAMES:
            line = tx_reader.get_line(timeout=0.2)
            if line:
                m = TX_FRAME_RE.search(line)
                if m:
                    sync_hex, len_hex, data_blob, cs_hex = m.groups()
                    data_bytes = parse_hex_bytes(data_blob)
                    frame = [int(sync_hex, 16), int(len_hex, 16)] + data_bytes + [int(cs_hex, 16)]
                    t_start = None
                    # Look ahead for timing
                    for _ in range(10):
                        tline = tx_reader.get_line(timeout=0.1)
                        if not tline:
                            continue
                        tm = TX_TIMING_RE.search(tline)
                        if tm:
                            t_start = int(tm.group(1))
                            break
                    if not t_start:
                        t_start = int(time.time() * 1_000_000)
                    frames_tx.append((t_start, frame))
                    print(f"[TX#{len(frames_tx)}] {frame}  @ {t_start} µs")

            if rx_reader:
                rline = rx_reader.get_line(timeout=0.0)
                if rline:
                    rm = RX_HEX_RE.search(rline)
                    if rm:
                        rx_bytes = parse_hex_bytes(rm.group(1))
                        tref = frames_tx[-1][0] if frames_tx else int(time.time() * 1_000_000)
                        frames_rx.append((tref, rx_bytes))
                        print(f"[RX#{len(frames_rx)}] {rx_bytes}")

    except KeyboardInterrupt:
        print("\nStopped by user.\n")
    finally:
        tx_reader.stop()
        if rx_reader:
            rx_reader.stop()

    return frames_tx, frames_rx


def save_csv(times, levels, path):
    with open(path, "w") as f:
        f.write("time_us,level\n")
        for t, l in zip(times, levels):
            f.write(f"{t:.3f},{l}\n")
    print("💾 Saved", path)


# --- Main ---
def main():
    frames_tx, frames_rx = collect_frames()
    if not frames_tx:
        print("No TX frames captured. Exiting.")
        return

    first_start = frames_tx[0][0]
    bit_us = 1_000_000.0 / UART_BAUD

    tx_t, tx_l = [], []
    for (t_us, bytes_seq) in frames_tx:
        times, levels = synth_uart_waveform(bytes_seq, UART_BAUD, t0_us=t_us - first_start)
        if tx_t:
            offset = (tx_t[-1] + 5 * bit_us) - times[0]
            times = [t + offset for t in times]
        tx_t += times
        tx_l += levels

    rx_t, rx_l = [], []
    if frames_rx:
        for (t_us, bytes_seq) in frames_rx:
            times, levels = synth_uart_waveform(bytes_seq, UART_BAUD, t0_us=t_us - first_start)
            if rx_t:
                offset = (rx_t[-1] + 5 * bit_us) - times[0]
                times = [t + offset for t in times]
            rx_t += times
            rx_l += levels

    if CSV_OUT_TX:
        save_csv(tx_t, tx_l, CSV_OUT_TX)
    if frames_rx and CSV_OUT_RX:
        save_csv(rx_t, rx_l, CSV_OUT_RX)

    # --- Plot ---
    plt.figure(figsize=(10, 5))
    plt.step(tx_t, [v + 2 for v in tx_l], where='post', label='TX (offset +2)')
    if frames_rx:
        plt.step(rx_t, rx_l, where='post', label='RX')
    plt.xlabel("Time (µs, relative)")
    plt.ylabel("Logic Level")
    plt.title(f"UART 8N1 @ {UART_BAUD} bps — TX/RX Waveforms")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(PNG_OUT, dpi=180)
    print("📈 Saved", PNG_OUT)
    plt.show()


if __name__ == "__main__":
    main()
