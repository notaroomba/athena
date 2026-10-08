#!/usr/bin/env python3
"""lora_check.py capture.cu8 [fs=1024000] [tune_offset_hz=250000]
Verifies Athena's LoRa downlink from an RTL-SDR capture (rtl_sdr -f 433.25e6 -s 1024000 -g 40 -n 10240000 capture.cu8):
bursts per second, their length, carrier offset, and whether each burst starts with LoRa SF7/125k upchirps
followed by the 0x12 sync word (symbols 8 and 16) - i.e. it really is the SX1278 at the configured settings."""
import sys, numpy as np
from scipy.signal import resample_poly
fn = sys.argv[1]; fs = float(sys.argv[2]) if len(sys.argv) > 2 else 1.024e6; off = float(sys.argv[3]) if len(sys.argv) > 3 else 250e3
raw = np.fromfile(fn, dtype=np.uint8).astype(np.float32); iq = (raw[0::2] - 127.5) + 1j * (raw[1::2] - 127.5)
n = np.arange(len(iq)); iq *= np.exp(2j * np.pi * off / fs * n)          # bring 433.000 MHz to baseband (tuned 250 kHz above it)
BW, SF = 125e3, 7; N = 1 << SF
x = resample_poly(iq, int(BW), int(fs))                                   # 125 kS/s: one sample per chip, 128 per symbol
x = x[: len(x) // N * N]
p = np.abs(x) ** 2; blk = p.reshape(-1, N).mean(axis=1)                   # power per symbol time (1.024 ms)
thr = np.median(blk) * 6
on = blk > thr
edges = np.flatnonzero(np.diff(on.astype(int)) == 1) + 1
bursts = []
for e in edges:
    end = e
    while end < len(on) and on[end]: end += 1
    if end - e >= 12: bursts.append((e, end))
print(f"{len(bursts)} bursts in {len(x) / BW:.1f} s; noise floor {10*np.log10(np.median(blk)):.1f} dB, threshold {10*np.log10(thr):.1f} dB")
if len(bursts) > 1:
    starts = np.array([b[0] for b in bursts]) * N / BW
    print(f"period {np.median(np.diff(starts))*1000:.0f} ms (TPU sends telemetry every 500 ms, SPU status every 2 s)")
k = np.arange(N); up = np.exp(1j * np.pi * k * k / N)                      # base upchirp, BW/N per sample
for i, (s, e) in enumerate(bursts[:6]):
    seg = x[s * N:e * N]
    dur = len(seg) / BW * 1000
    peak = 10 * np.log10(np.mean(np.abs(seg) ** 2))
    cfo = np.angle(np.mean(seg[1:] * np.conj(seg[:-1]))) * BW / (2 * np.pi)
    # dechirp symbol by symbol, allow for an unknown start within the first symbol: pick the alignment with the sharpest FFT peaks
    best = None
    for shift in range(0, N, 8):
        sym = seg[shift: shift + 12 * N]
        if len(sym) < 12 * N: break
        d = sym.reshape(12, N) * np.conj(up)
        F = np.abs(np.fft.fft(d, axis=1)); bins = F.argmax(axis=1); q = F.max(axis=1) / (F.mean(axis=1) + 1e-9)
        score = q[:8].sum()
        if best is None or score > best[0]: best = (score, shift, bins, q)
    _, shift, bins, q = best
    pre = bins[:8]; pre_ok = np.ptp(pre) <= 2
    sync = (bins[8:10] - pre[0]) % N
    sync_ok = all(abs(((v - t + N // 2) % N) - N // 2) <= 2 for v, t in zip(sync, (8, 16)))
    print(f"burst {i}: t={s*N/BW:6.2f}s len={dur:5.1f} ms level={peak:5.1f} dB cfo={cfo/1e3:+.1f} kHz | preamble bins {pre.tolist()} {'OK' if pre_ok else '??'} | sync {sync.tolist()} {'= 0x12 OK' if sync_ok else '(expect 8,16)'}")
