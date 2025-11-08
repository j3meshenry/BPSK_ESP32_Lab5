'''

Lab # 5 
Class: CECS 460 - System on Chip Design 
Students: James Henry, Emily Gomez, Eric Santana
Functionality: BPSK Logic Analyzer that generates static PNG, animated GIF, and VCD files. This is 
based on the ESP32's I/O for the tx to rx signal. 

'''

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from pathlib import Path

# ------------------------
# Simple DSP helpers (no SciPy)
# ------------------------
def awgn(x, snr_db):
    if np.isinf(snr_db):
        return x
    p_sig = np.mean(np.abs(x)**2)
    snr_lin = 10**(snr_db/10.0)
    p_noise = p_sig / snr_lin
    noise = np.sqrt(p_noise/2.0) * np.random.randn(*x.shape)
    return x + noise

def stretch_bits(bits, sps):
    return np.repeat(bits, sps)

def bit_edges(bits, sps):
    edges = np.zeros(bits.size * sps, dtype=int)
    tb = np.concatenate([[bits[0]], bits[:-1]])
    transitions = np.nonzero(bits != tb)[0]
    for idx in transitions:
        pos = idx * sps
        if pos < edges.size:
            edges[pos] = 1
    return edges

def make_symbol_clock(n_samples, sps):
    clk = np.zeros(n_samples, dtype=int)
    half = sps // 2
    for k in range(0, n_samples, sps):
        clk[k:k+half] = 1
    return clk

def bpsk_mod(bits_pm, sps, fc, fs):
    bb = stretch_bits(bits_pm, sps)
    n = np.arange(bb.size)
    carrier = np.cos(2*np.pi*fc*n/fs)
    rf = bb * carrier
    return rf

def lowpass_ma(x, taps=129):
    # Moving-average low-pass (simple and dependency-free)
    if taps % 2 == 0:
        taps += 1
    h = np.ones(taps, dtype=float) / float(taps)
    return np.convolve(x, h, mode='same')

def demod_coherent(rf, fc, fs, lp_len=129):
    n = np.arange(rf.size)
    lo = np.cos(2*np.pi*fc*n/fs)
    mixed = rf * lo
    bb = lowpass_ma(mixed, taps=lp_len)
    return bb

def integrate_and_dump(bb, sps):
    n = (bb.size // sps) * sps
    if n == 0:
        return np.array([]), np.array([], dtype=int)
    bb_reshaped = bb[:n].reshape(-1, sps)
    samples = bb_reshaped.mean(axis=1)
    hard = (samples > 0).astype(int)
    return samples, hard

def to_logic_levels(x):
    return (x > 0).astype(int)

def write_vcd(channels, fs, path, timescale='1us'):
    # Minimal VCD writer for 1-bit channels
    ids = {}
    printable = [chr(i) for i in range(33, 126)]
    for i, k in enumerate(channels.keys()):
        ids[k] = printable[i % len(printable)]

    dt_s = 1.0 / fs
    ts_factor = {
        '1ns':1e-9,'10ns':10e-9,'100ns':100e-9,
        '1us':1e-6,'10us':10e-6,'100us':100e-6,
        '1ms':1e-3
    }[timescale]
    tick_per_sample = max(1, int(round(dt_s / ts_factor)))

    with open(path, 'w') as f:
        f.write("$date\n  generated\n$end\n")
        f.write("$version\n  logic_analyzer.py\n$end\n")
        f.write(f"$timescale {timescale} $end\n")
        f.write("$scope module logic $end\n")
        for name, sym in ids.items():
            f.write(f"$var wire 1 {sym} {name} $end\n")
        f.write("$upscope $end\n$enddefinitions $end\n")

        # Initial values
        f.write("#0\n")
        prev = {}
        for name, sym in ids.items():
            v = int(channels[name][0])
            f.write(f"{v}{sym}\n")
            prev[name] = v

        # Changes
        for i in range(1, len(next(iter(channels.values())))):
            changed = []
            for k, arr in channels.items():
                v = int(arr[i])
                if v != prev[k]:
                    changed.append((k, v))
                    prev[k] = v
            if changed:
                t = i * tick_per_sample
                f.write(f"#{t}\n")
                for k, v in changed:
                    f.write(f"{v}{ids[k]}\n")

# ------------------------
# Main (generate PNG, GIF, VCD)
# ------------------------
def main():
    # Parameters chosen to run fast and look clean
    Rs = 50_000.0     # symbol rate (baud)
    fs = 1_000_000.0  # sample rate (Hz)
    fc = 200_000.0    # carrier (Hz)
    snr_db = 12.0
    sps = int(round(fs / Rs))  # 20 samples/symbol
    n_bits = 80

    # ----- One static capture -----
    tx_bits = np.random.randint(0, 2, size=n_bits).astype(int)
    tx_pm = 2*tx_bits - 1

    rf = bpsk_mod(tx_pm, sps, fc, fs)
    rx = awgn(rf, snr_db)
    bb = demod_coherent(rx, fc, fs)

    rx_sign = to_logic_levels(bb)
    _, rx_bits = integrate_and_dump(bb, sps)

    tx_bit_line = stretch_bits(tx_bits, sps)
    rx_bit_line = stretch_bits(rx_bits, sps)
    sym_clk = make_symbol_clock(tx_bit_line.size, sps)
    edge = bit_edges(tx_bits, sps)

    m = min(rx_bits.size, tx_bits.size)
    ber = float(np.sum(rx_bits[:m] != tx_bits[:m])) / float(m) if m else 0.0

    # ----- Static stacked PNG (single axis) -----
    N_syms_view = 20
    N = N_syms_view * sps
    t = np.arange(N) / fs

    channels = [
        ("TX_BIT",   tx_bit_line[:N]),
        ("SYM_CLK",  sym_clk[:N]),
        ("RX_SIGN",  rx_sign[:N]),
        ("RX_BIT",   rx_bit_line[:N]),
        ("BIT_EDGE", edge[:N]),
    ]
    labels = [name for name, _ in channels]
    offset_step = 2.0

    fig1 = plt.figure(figsize=(10, 4))
    ax = plt.gca()
    for i, (_, arr) in enumerate(channels):
        ax.step(t, arr + i*offset_step, where='post')  # no explicit colors
    ax.set_yticks([i*offset_step for i in range(len(labels))])
    ax.set_yticklabels(labels)
    ax.set_xlabel("Time (s)")
    ax.set_title(f"BPSK Logic Analyzer (PC3)  |  Rs={Rs:.0f} Bd  fs={fs:.0f} Hz  fc={fc:.0f} Hz  SNR={snr_db} dB  |  BER≈{ber:.3g}")
    ax.grid(True, which='both', alpha=0.3)
    fig1.tight_layout()
    static_png = Path("bpsk_logic_static.png")
    fig1.savefig(static_png)
    plt.close(fig1)

    # ----- Short realtime-style GIF (single axis) -----
    win_syms = 120
    Nwin = win_syms * sps
    x = np.arange(Nwin) / fs
    buf = np.zeros((len(channels), Nwin), dtype=float)

    fig2 = plt.figure(figsize=(10, 4))
    ax2 = plt.gca()
    lines = []
    for i in range(len(channels)):
        (ln,) = ax2.step(x, buf[i] + i*offset_step, where='post')
        lines.append(ln)
    ax2.set_yticks([i*offset_step for i in range(len(labels))])
    ax2.set_yticklabels(labels)
    ax2.set_xlabel("Time (s)")
    ax2.set_title("BPSK Logic Analyzer (PC3)")
    ax2.grid(True, which='both', alpha=0.3)
    fig2.tight_layout()

    one_sym_clk = np.concatenate([np.ones(sps//2, dtype=int),
                                  np.zeros(sps - sps//2, dtype=int)])
    chunk_syms = 8
    chunk_samps = chunk_syms * sps

    def anim_step(_i):
        tx_bits_chunk = np.random.randint(0, 2, size=chunk_syms).astype(int)
        tx_pm_chunk = 2*tx_bits_chunk - 1

        rf_chunk = bpsk_mod(tx_pm_chunk, sps, fc, fs)
        rx_chunk = awgn(rf_chunk, snr_db)
        bb_chunk = demod_coherent(rx_chunk, fc, fs)

        rx_sign_chunk = to_logic_levels(bb_chunk)
        _, rx_bits_chunk = integrate_and_dump(bb_chunk, sps)

        tx_line = stretch_bits(tx_bits_chunk, sps)
        rx_line = stretch_bits(rx_bits_chunk, sps)
        clk_line = np.tile(one_sym_clk, chunk_syms)
        edge_line = bit_edges(tx_bits_chunk, sps)

        new_rows = [tx_line, clk_line, rx_sign_chunk[:chunk_samps], rx_line, edge_line]

        # scroll buffers
        for i in range(len(channels)):
            buf[i, :-chunk_samps] = buf[i, chunk_samps:]
            buf[i, -chunk_samps:] = new_rows[i]

        for i, ln in enumerate(lines):
            ln.set_ydata(buf[i] + i*offset_step)

        ax2.set_title("BPSK Logic Analyzer (PC3)")
        return lines

    ani = FuncAnimation(fig2, anim_step, frames=60, interval=80, blit=False)
    gif_path = Path("BPSK_Logic_Analyzer_PC3.gif")
    try:
        ani.save(gif_path, writer=PillowWriter(fps=12))
    except Exception as e:
        print("GIF save failed (install pillow). Error:", e)
    plt.close(fig2)

    # ----- Tiny VCD sample -----
    vcd_channels = {
        "TX_BIT":   tx_bit_line[:N].astype(int),
        "SYM_CLK":  sym_clk[:N].astype(int),
        "RX_SIGN":  to_logic_levels(bb[:N]).astype(int),
        "RX_BIT":   rx_bit_line[:N].astype(int),
        "BIT_EDGE": edge[:N].astype(int),
    }
    vcd_path = Path("bpsk_demo.vcd")
    write_vcd(vcd_channels, fs=fs, path=vcd_path, timescale='1us')

    print("\nGenerated:")
    print(" -", static_png.resolve())
    print(" -", gif_path.resolve())
    print(" -", vcd_path.resolve())

if __name__ == "__main__":
    main()
