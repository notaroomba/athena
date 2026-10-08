"""
lora_phy.py - minimal LoRa PHY receiver for Athena's downlink (SF7, 125 kHz, CR 4/5, explicit header,
sync word 0x12) on top of an RTL-SDR IQ stream. numpy/scipy only.

Pipeline: shift to baseband -> resample to 1 sample/chip (125 kS/s) -> power-gate bursts -> locate the
preamble (8 identical dechirped bins) -> sync word check -> dechirp the header block (8 symbols at the
reduced rate SF-2) -> gray/deinterleave/Hamming -> payload length -> dechirp the payload blocks ->
dewhiten -> bytes. The conventions LoRa leaves open (symbol offset, gray mapping, interleaver rotation,
bit orders) are learned once from real traffic with learn(): the Athena link frame CRC inside the payload
is the oracle, so no guesswork survives a wrong convention.
"""
import itertools, json, os
import numpy as np
from scipy.signal import resample_poly

SF, N, BW = 7, 128, 125e3
OS = 8                                           # samples per chip: eighth-chip timing alignment
FS = BW * OS
NS = N * OS                                      # samples per symbol
_t = np.arange(NS) / OS                          # time in chips
UP = np.exp(1j * np.pi * _t * _t / N).astype(np.complex64)      # base upchirp over one symbol
DOWN = np.conj(UP)


def whitening_seq(n=255):
    """LoRa payload whitening: 8-bit LFSR x^8+x^6+x^5+x^4+1, seed 0xFF (0xFF 0xFE 0xFC 0xF8 0xF0 0xE1 ...)."""
    s, out = 0xFF, []
    for _ in range(n):
        out.append(s)
        s = ((s << 1) | (bin(s & 0xB8).count("1") & 1)) & 0xFF
    return out


WHITE = whitening_seq()


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def athena_frames(payload):
    """Returns the Athena link frames found in a decoded LoRa payload (SOF 0xA5, type, len, payload, crc16)."""
    out, i = [], 0
    while i + 5 <= len(payload):
        if payload[i] == 0xA5 and payload[i + 2] <= 200 and i + 5 + payload[i + 2] <= len(payload):
            n = payload[i + 2]
            if crc16(payload[i + 1:i + 3 + n]) == payload[i + 3 + n] | (payload[i + 4 + n] << 8):
                out.append(bytes(payload[i:i + 5 + n]))
                i += 5 + n
                continue
        i += 1
    return out


# ---------------------------------------------------------------- front end
_LUT = np.arange(256, dtype=np.float32) - 127.5
_LO = {}                                         # mixer table, reused while the window length is constant


def to_baseband(iq_u8, fs, offset_hz):
    """RTL-SDR cu8 bytes -> complex at FS (8 samples/chip) with the LoRa channel at DC (tuned `offset_hz` away)."""
    raw = np.frombuffer(iq_u8, dtype=np.uint8)
    raw = raw[: len(raw) & ~1]                   # a truncated file can end mid-pair
    iq = _LUT[raw].view(np.complex64)            # interleaved float32 I,Q pairs are complex64 already
    key = (len(iq), fs, offset_hz)
    lo = _LO.get(key)
    if lo is None:
        _LO.clear()
        lo = _LO[key] = np.exp(2j * np.pi * offset_hz / fs * np.arange(len(iq))).astype(np.complex64)
    iq = iq * lo
    if int(fs) == int(FS):
        return iq                                # live: the dongle runs at FS, nothing to resample
    return resample_poly(iq, int(FS), int(fs)).astype(np.complex64)


def find_bursts(x, min_syms=12, thr_db=8.0):
    """Power-gated burst spans [(start, end)] in samples (symbol granularity)."""
    n = len(x) // NS
    p = (np.abs(x[: n * NS]) ** 2).reshape(n, NS).mean(axis=1)
    floor = np.median(p) + 1e-12
    on = p > floor * 10 ** (thr_db / 10)
    spans, i = [], 0
    while i < n:
        if on[i]:
            j = i
            while j < n and on[j]:
                j += 1
            if j - i >= min_syms:
                spans.append((max(0, i - 1) * NS, min(n, j + 1) * NS))
            i = j
        else:
            i += 1
    return spans


def _fft_fold(seg, chirp):
    """Dechirp (count, NS) samples and fold the OS-times-longer spectrum back onto N bins."""
    F = np.fft.fft(seg * chirp, axis=1)
    return np.abs(F).reshape(seg.shape[0], OS, N).sum(axis=1), F


def dechirp(x, start, count, down=False):
    """Folded FFT magnitudes (count, N) of `count` consecutive symbols starting at sample `start`."""
    seg = x[start:start + count * NS]
    if len(seg) < count * NS:
        return None
    return _fft_fold(seg.reshape(count, NS), UP if down else DOWN)[0]


def fine_cfo(x, p0, ref):
    """Fractional carrier offset in bins from the phase advance of the preamble peak, symbol to symbol."""
    seg = x[p0:p0 + 8 * NS].reshape(8, NS)
    F = np.fft.fft(seg * DOWN, axis=1)
    c = F[:, ref]                                   # complex peak of each preamble symbol (same bin)
    dphi = np.angle(np.sum(c[1:] * np.conj(c[:-1])))
    return dphi / (2 * np.pi)                      # one symbol = one bin of phase rotation per cycle


def locate_preamble(x, s, e):
    """Quarter-chip symbol alignment inside a burst: returns (preamble_start, ref_bin, quality) or None."""
    def score_at(shift):
        F = dechirp(x, s + shift, 12)
        if F is None:
            return None
        bins = F.argmax(axis=1)
        q = F.max(axis=1) / (F.mean(axis=1) + 1e-9)
        best = None
        for k in range(0, 4):
            run = bins[k:k + 8]
            agree = int(np.sum(run == np.bincount(run).argmax()))
            sc = (agree, float(q[k:k + 8].sum()))
            if best is None or sc > best[0]:
                best = (sc, s + shift + k * NS, int(np.bincount(run).argmax()))
        return best
    best = None
    coarse = OS                                        # one chip steps first, then refine to one sample
    for shift in range(0, NS, coarse):
        b = score_at(shift)
        if b and (best is None or b[0] > best[0]):
            best, bshift = b, shift
    if best is None:
        return None
    for shift in range(max(0, bshift - coarse), min(NS, bshift + coarse)):
        b = score_at(shift)
        if b and b[0] > best[0]:
            best = b
    if best[0][0] < 6:
        return None
    return best[1], best[2], best[0]


def sync_ok(x, p0, ref):
    F = dechirp(x, p0 + 8 * NS, 2)
    if F is None:
        return False
    b = (F.argmax(axis=1) - ref) % N
    return all(min(abs(b[i] - t), N - abs(b[i] - t)) <= 2 for i, t in enumerate((8, 16)))


def track_bins(seg, start, count, ref):
    """Dechirp `count` symbols from `start`, re-aligning each symbol by up to one sample (quarter chip) so a
    clock mismatch between transmitter and SDR cannot walk the sampling point off the bin centre.
    Returns (bins, final_position)."""
    bins = []
    pos = start
    for _ in range(count):
        best = None
        for sh in (-1, 0, 1):
            a = pos + sh
            if a < 0 or a + NS > len(seg):
                continue
            F = _fft_fold(seg[a:a + NS].reshape(1, NS), DOWN)[0][0]
            k = int(F.argmax())
            score = F[k] / (F.mean() + 1e-9)
            if best is None or score > best[0]:
                best = (score, sh, k)
        if best is None:
            break
        bins.append(best[2])
        pos += NS + best[1]
    return np.array(bins, dtype=int), pos


def compensate(x, p0, ref, length):
    """Remove the fractional carrier offset over one packet so every symbol lands on a bin centre."""
    f = fine_cfo(x, p0, ref)
    seg = x[p0:p0 + length]
    return seg * np.exp(-2j * np.pi * f / NS * np.arange(len(seg))).astype(np.complex64), f


# ---------------------------------------------------------------- bit level
def gray_full(v):
    v ^= v >> 4
    v ^= v >> 2
    v ^= v >> 1
    return v


GRAYS = {"none": lambda v: v, "g1": lambda v: v ^ (v >> 1), "full": gray_full}
ROTS = {"i-j-1": lambda i, j: i - j - 1, "i-j": lambda i, j: i - j, "i+j": lambda i, j: i + j, "i+j+1": lambda i, j: i + j + 1, "j-i": lambda i, j: j - i}
HDR_REDUCE = {">>2": lambda v: v >> 2, "round": lambda v: int(round(v / 4)) % 32, "+2>>2": lambda v: ((v + 2) >> 2) % 32}


def deinterleave(vals, nbits, rot, msb_first):
    cw_len = len(vals)
    bits = [[((v >> (nbits - 1 - j)) & 1) if msb_first else ((v >> j) & 1) for j in range(nbits)] for v in vals]
    cws = [[0] * cw_len for _ in range(nbits)]
    for i in range(cw_len):
        for j in range(nbits):
            cws[rot(i, j) % nbits][i] = bits[i][j]
    return cws


def cw_nibble(cwbits, cw_msb_first, data_high, nib_rev):
    L = len(cwbits)
    cw = 0
    for i, b in enumerate(cwbits):
        cw |= b << ((L - 1 - i) if cw_msb_first else i)
    d = (cw >> (L - 4)) & 0xF if data_high else cw & 0xF
    if nib_rev:
        d = int(f"{d:04b}"[::-1], 2)
    return d


def nibbles_to_bytes(nibs, low_first):
    out = []
    for a, b in zip(nibs[0::2], nibs[1::2]):
        out.append((b << 4 | a) if low_first else (a << 4 | b))
    return out


CONV_KEYS = ("off", "gray", "gray_first", "hdr_reduce", "rot", "msb_first", "cw_msb_first", "data_high", "nib_rev", "low_first")
CONV_SPACE = {
    "off": [0, 1, -1, 2, -2], "gray": list(GRAYS), "gray_first": [True, False], "hdr_reduce": list(HDR_REDUCE),
    "rot": list(ROTS), "msb_first": [True, False], "cw_msb_first": [True, False], "data_high": [True, False],
    "nib_rev": [False, True], "low_first": [True, False],
}


def header_from_bins(bins, ref, c):
    vals = [(int(b) - ref + c["off"]) % N for b in bins[:8]]
    g = GRAYS[c["gray"]]
    red = HDR_REDUCE[c["hdr_reduce"]]
    hv = [red(g(v)) if c["gray_first"] else g(red(v)) & 31 for v in vals]
    cws = deinterleave(hv, SF - 2, ROTS[c["rot"]], c["msb_first"])
    nib = [cw_nibble(cw, c["cw_msb_first"], c["data_high"], c["nib_rev"]) for cw in cws]
    length = (nib[0] << 4) | nib[1]
    cr, crc = nib[2] >> 1, nib[2] & 1
    return length, cr, crc, nib


def payload_symbols(length, cr, crc):
    nibs = 2 * (length + (2 if crc else 0))
    blocks = -(-nibs // SF)
    return blocks * (4 + cr), blocks


def payload_from_bins(bins, ref, c, length, cr, crc):
    cw_len = 4 + cr
    g = GRAYS[c["gray"]]
    nibs = []
    for b0 in range(0, len(bins) - cw_len + 1, cw_len):
        vals = [g((int(b) - ref + c["off"]) % N) for b in bins[b0:b0 + cw_len]]
        cws = deinterleave(vals, SF, ROTS[c["rot"]], c["msb_first"])
        nibs += [cw_nibble(cw, c["cw_msb_first"], c["data_high"], c["nib_rev"]) for cw in cws]
    raw = nibbles_to_bytes(nibs, c["low_first"])
    data = [raw[i] ^ WHITE[i % 255] for i in range(min(length, len(raw)))]
    return data


def decode_packet(x, p0, ref, conv):
    """One LoRa packet whose preamble starts at p0 (already aligned). Returns the result dict."""
    res = {"ref": ref, "p0": p0, "payload": [], "frames": [], "ok": False, "length": 0, "cr": 0, "crc": 0,
           "end": p0 + int(round(12.25 * NS)) + 8 * NS, "cfo": 0.0}            # at least past the header, so a caller always advances
    # a first pass on the raw signal gives the header; then re-dechirp the whole packet with the fine CFO removed
    d0 = p0 + int(round(12.25 * NS))
    F = dechirp(x, d0, 8)
    if F is None:
        return res
    length, cr, crc, _ = header_from_bins(F.argmax(axis=1), ref, conv)
    if not (1 <= length <= 255 and 1 <= cr <= 4):
        return res
    nsym, _ = payload_symbols(length, cr, crc)
    total = int(round(12.25 * NS)) + (8 + nsym) * NS
    seg, f = compensate(x, p0, ref, total)
    res["cfo"] = f
    if len(seg) < total:
        return res
    F = dechirp(seg, 0, 8)
    ref2 = int(np.bincount(F.argmax(axis=1)).argmax())            # preamble bin after compensation
    hb, pos = track_bins(seg, int(round(12.25 * NS)), 8, ref2)
    if len(hb) < 8:
        return res
    length, cr, crc, _ = header_from_bins(hb, ref2, conv)
    res.update(length=length, cr=cr, crc=crc)
    if not (1 <= length <= 255 and 1 <= cr <= 4):
        return res
    nsym, _ = payload_symbols(length, cr, crc)
    pb, _ = track_bins(seg, pos, nsym, ref2)
    if len(pb) < nsym:
        return res
    res["payload"] = payload_from_bins(pb, ref2, conv, length, cr, crc)
    res["frames"] = athena_frames(res["payload"])
    res["ok"] = bool(res["frames"])
    res["end"] = p0 + int(round(12.25 * NS)) + (8 + nsym) * NS
    return res


def decode_burst(x, span, conv):
    """All packets inside a power-gated span (two Athena frames ride back to back every 2 s)."""
    s, e = span
    out = []
    while s + 14 * NS < e:
        loc = locate_preamble(x, s, e)
        if not loc:
            break
        p0, ref, _ = loc
        if not sync_ok(x, p0, ref):
            break
        r = decode_packet(x, p0, ref, conv)
        out.append(r)
        nxt = r["end"] - NS                                 # the next preamble may start right away
        if nxt <= s or len(out) > 4:
            break
        s = nxt
    return out


def learn(x, spans, max_bursts=4):
    """Brute-force the convention space on a few bursts; returns (conv, hits) for the best one."""
    keys = CONV_KEYS
    best, best_hits = None, 0
    pre = []
    for span in spans[:max_bursts]:
        loc = locate_preamble(x, span[0], span[1])
        if not loc:
            continue
        p0, ref, _ = loc
        if not sync_ok(x, p0, ref):
            continue
        seg, _ = compensate(x, p0, ref, int(round(12.25 * NS)) + 88 * NS)   # enough symbols for a 49-byte frame
        F = dechirp(seg, int(round(12.25 * NS)), 88)
        if F is None:
            continue
        ref2 = int(np.bincount(dechirp(seg, 0, 8).argmax(axis=1)).argmax())
        pre.append((F.argmax(axis=1), ref2))
    if not pre:
        return None, 0
    for combo in itertools.product(*[CONV_SPACE[k] for k in keys]):
        c = dict(zip(keys, combo))
        hits = 0
        for bins, ref in pre:
            length, cr, crc, _ = header_from_bins(bins, ref, c)
            if not (5 <= length <= 200 and 1 <= cr <= 4):
                continue
            nsym, _ = payload_symbols(length, cr, crc)
            if 8 + nsym > len(bins):
                continue
            payload = payload_from_bins(bins[8:8 + nsym], ref, c, length, cr, crc)
            if athena_frames(payload):
                hits += 1
        if hits > best_hits:
            best, best_hits = c, hits
            if hits == len(pre):
                break
    return best, best_hits


CONV_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lora_conv.json")


def load_conv():
    try:
        return json.load(open(CONV_FILE))
    except Exception:
        return None


def save_conv(c):
    json.dump(c, open(CONV_FILE, "w"), indent=1)
