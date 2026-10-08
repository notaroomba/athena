#!/usr/bin/env python3
"""
lora_rx.py - Athena ground station on an RTL-SDR: receives the TPU's LoRa downlink (433 MHz, SF7/125k,
sync 0x12), decodes the Athena frames and shows them in a terminal UI. Every decoded frame is appended
to athena-lora-<stamp>.bin (same raw format as the SD/flash logs, replayable in the dashboard) and,
with --relay, pushed to the WebSocket relay so the website shows the flight live.

  python3 tools/lora_rx.py                              # live; opens the web dashboard fed by this process
  python3 tools/lora_rx.py --tui                        # curses terminal UI instead
  python3 tools/lora_rx.py --relay wss://api.athena.notaroomba.dev/ws --password ...   # also feed the public site
  python3 tools/lora_rx.py --file capture.cu8           # replay an rtl_sdr capture
  python3 tools/lora_rx.py --learn                      # re-learn the PHY bit conventions from live traffic

needs: python3 with numpy, scipy, websockets (pip install numpy scipy websockets websocket-client) and
rtl_sdr from librtlsdr (brew install librtlsdr / apt install rtl-sdr / Windows release zip). macOS, Linux, Windows.
"""
import argparse, collections, curses, json, math, os, struct, subprocess, sys, threading, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import lora_phy as L

PHASES = ["pad", "boost", "coast", "apogee", "descent", "landed"]
PD_MODES = ["none", "PTCH", "APP", "BOOT"]
FIX = ["none", "DR", "2D", "3D", "3D+DR", "time"]


# ------------------------------------------------------------------ frame parsing (mirror of athena_link.h)
def parse_telem(p):
    if len(p) < 38:
        return None
    t_ms, lat, lon, alt, baro = struct.unpack_from("<IiIff", p, 0) if False else struct.unpack_from("<Iiiff", p, 0)
    vn, ve, vd, q0, q1, q2, q3 = struct.unpack_from("<7h", p, 20)
    fix, sv, flags, imu = struct.unpack_from("<4B", p, 34)
    return dict(t_ms=t_ms, lat=lat * 1e-7, lon=lon * 1e-7, alt=alt, baro=baro, vel=(vn / 10, ve / 10, vd / 10),
                q=(q0 / 32767, q1 / 32767, q2 / 32767, q3 / 32767), fix=fix, sv=sv, flags=flags, imu=imu)


def parse_spu(p):
    if len(p) < 44:
        return None
    t_ms, vbat, vsys, vbus, ibat, iin, chg, main_alt = struct.unpack_from("<IHHHhHHH", p, 0)
    phase, flags, fired, on, pd_mode, pd_status = struct.unpack_from("<6B", p, 18)
    servo = struct.unpack_from("<6H", p, 24)
    apogee, vmax = struct.unpack_from("<ff", p, 36)
    return dict(t_ms=t_ms, vbat=vbat, vsys=vsys, vbus=vbus, ibat=ibat, iin=iin, chg=chg, main_alt=main_alt, phase=phase,
                flags=flags, fired=fired, on=on, pd_mode=pd_mode, pd_status=pd_status, servo=servo, apogee=apogee, vmax=vmax)


def rpy(q):
    w, x, y, z = q
    r = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    p = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
    yw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return r, p, yw


# ------------------------------------------------------------------ receiver
class Receiver:
    def __init__(self, args):
        self.args = args
        self.fs_in = 1_024_000
        self.window_s = 4.0
        self.raw = bytearray()                       # rolling raw IQ (u8 pairs)
        self.raw_global0 = 0                         # global input-sample index of raw[0]
        self.done_until = 0                          # global baseband sample index already decoded
        self.conv = L.load_conv()
        self.lock = threading.Lock()
        self.stats = dict(bursts=0, packets=0, ok=0, bad_crc=0, last_rx=0.0, level_db=0.0, cfo=0.0, noise_db=0.0)
        self.telem = None
        self.spu = None
        self.events = collections.deque(maxlen=12)
        self.log = collections.deque(maxlen=200)
        self.prev_phase = None
        self.prev_fired = 0
        self.running = True
        self.eof = False
        self.local_clients = set()                   # browsers attached to the local relay (--gui)
        self.local_loop = None
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.outfile = open(f"athena-lora-{stamp}.bin", "ab")
        self.ws = None
        if args.relay:
            threading.Thread(target=self.relay_loop, daemon=True).start()

    # -- input --
    def source(self):
        if self.args.file:
            data = open(self.args.file, "rb").read()
            step = int(self.fs_in * 2 * 0.25)        # 0.25 s per chunk
            for i in range(0, len(data), step):
                if not self.running:
                    return
                yield data[i:i + step]
                if not self.args.fast:
                    time.sleep(0.25)
            self.note("end of file")
            time.sleep(1.0)
            self.process()
            self.eof = True
        else:
            cmd = ["rtl_sdr", "-f", str(int(self.args.freq + self.args.offset)), "-s", str(self.fs_in), "-g", str(self.args.gain), "-"]
            try:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
            except FileNotFoundError:
                self.note("rtl_sdr not found (brew install librtlsdr)")
                return
            self.note(f"rtl_sdr tuned {self.args.freq + self.args.offset:.0f} Hz, channel {self.args.freq:.3f} MHz" if False else
                      f"rtl_sdr started: channel {self.args.freq / 1e6:.3f} MHz, gain {self.args.gain} dB")
            while self.running:
                chunk = proc.stdout.read(int(self.fs_in * 2 * 0.25))
                if not chunk:
                    self.note("rtl_sdr stopped (dongle busy or unplugged?)")
                    return
                yield chunk

    def run(self):
        last_proc = 0.0
        for chunk in self.source():
            self.raw += chunk
            maxlen = int(self.fs_in * 2 * self.window_s)
            if len(self.raw) > maxlen:
                drop = len(self.raw) - maxlen
                drop -= drop % 2
                del self.raw[:drop]
                self.raw_global0 += drop // 2
            if time.time() - last_proc >= 0.5 or self.args.file:
                last_proc = time.time()
                self.process()

    def process(self):
        x = L.to_baseband(bytes(self.raw), self.fs_in, self.args.offset)
        g0 = int(self.raw_global0 * L.FS / self.fs_in)      # baseband index of x[0] in global baseband samples
        n = len(x) // L.NS
        p = (np.abs(x[: n * L.NS]) ** 2).reshape(n, L.NS).mean(axis=1)
        noise = float(np.median(p)) + 1e-12
        self.stats["noise_db"] = 10 * math.log10(noise)
        margin = int(0.15 * L.FS)                             # a burst still growing at the window end waits for the next pass
        for s, e in L.find_bursts(x):
            if g0 + s < self.done_until or e > len(x) - margin:
                continue
            self.stats["bursts"] += 1
            self.stats["level_db"] = 10 * math.log10(float((np.abs(x[s:e]) ** 2).mean()) / noise)
            if self.conv is None:
                conv, hits = L.learn(x, [(s, e)], 1)
                if conv and hits:
                    self.conv = conv
                    L.save_conv(conv)
                    self.note("PHY conventions learned from live traffic")
                else:
                    continue
            for r in L.decode_burst(x, (s, e), self.conv):
                self.stats["packets"] += 1
                self.stats["cfo"] = r["cfo"]
                if r["ok"]:
                    self.stats["ok"] += 1
                    self.stats["last_rx"] = time.time()
                    for f in r["frames"]:
                        self.handle(f)
                else:
                    self.stats["bad_crc"] += 1
            self.done_until = g0 + e

    # -- local relay (graphical mode): same protocol as software/server, so the web dashboard is the UI --
    def local_relay(self, port):
        import asyncio
        try:
            import websockets
        except ImportError:
            self.note("pip install websockets for the graphical mode")
            return

        async def status():
            return json.dumps({"type": "status", "viewers": len(self.local_clients), "adminConnected": True})

        async def client(ws):
            self.local_clients.add(ws)
            try:
                await ws.send(await status())
                async for _ in ws:                       # browsers send auth/pings; nothing to do with them here
                    pass
            except Exception:
                pass
            finally:
                self.local_clients.discard(ws)

        async def main():
            self.local_loop = asyncio.get_running_loop()
            async with websockets.serve(client, "127.0.0.1", port, max_size=1 << 20):
                self.note(f"local relay ws://localhost:{port}/ws")
                while self.running:
                    await asyncio.sleep(0.5)

        asyncio.run(main())

    def local_send(self, frame):
        loop = self.local_loop
        if not loop or not self.local_clients:
            return
        import asyncio

        async def fanout():
            for ws in list(self.local_clients):
                try:
                    await ws.send(frame)
                except Exception:
                    self.local_clients.discard(ws)

        asyncio.run_coroutine_threadsafe(fanout(), loop)

    # -- frames --
    def handle(self, frame):
        self.outfile.write(frame)
        self.outfile.flush()
        self.local_send(bytes(frame))
        if self.ws:
            try:
                self.ws.send_binary(frame)
            except Exception:
                self.ws = None
        t, n, p = frame[1], frame[2], frame[3:3 + frame[2]]
        with self.lock:
            if t == 0x03:
                d = parse_telem(p)
                if d:
                    self.telem = d
            elif t == 0x04:
                d = parse_spu(p)
                if d:
                    self.spu = d
                    if d["phase"] != self.prev_phase:
                        if self.prev_phase is not None:
                            self.event(f"{PHASES[d['phase']] if d['phase'] < 6 else d['phase']}  apogee {d['apogee']:.0f} m")
                        self.prev_phase = d["phase"]
                    new = d["fired"] & ~self.prev_fired
                    for ch in range(6):
                        if new & (1 << ch):
                            self.event(f"PYRO {ch + 1} FIRED")
                    self.prev_fired = d["fired"]
            elif t == 0x7F:
                self.note("rocket: " + p.decode("ascii", "replace"))

    def event(self, s):
        self.events.append(time.strftime("%H:%M:%S ") + s)

    def note(self, s):
        self.log.append(time.strftime("%H:%M:%S ") + s)

    # -- relay --
    def relay_loop(self):
        try:
            import websocket
        except ImportError:
            self.note("pip install websocket-client for --relay")
            return
        while self.running:
            try:
                ws = websocket.create_connection(self.args.relay, timeout=10)
                ws.send(json.dumps({"type": "auth", "password": self.args.password or ""}))
                self.ws = ws
                self.note("relay connected: " + self.args.relay)
                while self.running and self.ws is ws:
                    try:
                        ws.settimeout(5)
                        ws.recv()
                    except Exception as e:
                        if "timed out" in str(e):
                            continue
                        raise
            except Exception as e:
                self.ws = None
                self.note(f"relay: {e}")
                time.sleep(3)


# ------------------------------------------------------------------ UI
def draw(stdscr, rx):
    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_YELLOW, -1)
    curses.init_pair(2, curses.COLOR_CYAN, -1)
    curses.init_pair(3, curses.COLOR_RED, -1)
    curses.init_pair(4, curses.COLOR_GREEN, -1)
    Y, C, R, G = (curses.color_pair(i) for i in (1, 2, 3, 4))
    while rx.running:
        try:
            if stdscr.getch() in (ord("q"), 27):
                rx.running = False
                break
        except Exception:
            pass
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        st = rx.stats
        age = time.time() - st["last_rx"] if st["last_rx"] else None
        head = f" ATHENA ground station  {rx.args.freq / 1e6:.3f} MHz SF7/125k  |  packets {st['ok']}/{st['packets']}  bad {st['bad_crc']}  |  level {st['level_db']:.0f} dB  cfo {st['cfo'] * L.BW / L.N / 1e3:+.2f} kHz  |  last rx {('%.1f s' % age) if age is not None else '-'}"
        stdscr.addnstr(0, 0, head.ljust(w), w - 1, curses.A_REVERSE)
        with rx.lock:
            t, s = rx.telem, rx.spu
        col2 = w // 2
        y = 2
        stdscr.addstr(y, 1, "FLIGHT", Y | curses.A_BOLD)
        stdscr.addstr(y, col2, "RECOVERY / POWER (SPU)", Y | curses.A_BOLD)
        y += 1
        if t:
            r, p, yw = rpy(t["q"])
            rows = [
                ("altitude", f"{t['alt']:9.1f} m above pad", G if t['alt'] > 20 else 0),
                ("baro alt", f"{t['baro']:9.1f} m", 0),
                ("vertical", f"{-t['vel'][2]:9.1f} m/s", 0),
                ("ground", f"{math.hypot(t['vel'][0], t['vel'][1]):9.1f} m/s", 0),
                ("roll/pitch/yaw", f"{r:6.1f} {p:6.1f} {yw:6.1f} deg", 0),
                ("position", f"{t['lat']:.6f}  {t['lon']:.6f}", 0),
                ("gps", f"{FIX[t['fix']] if t['fix'] < 6 else t['fix']}  {t['sv']} sv", 0),
                ("flags", " ".join(n for b, n in ((1, "FLIGHT"), (2, "GPS"), (4, "BARO"), (8, "MAG"), (16, "ORIGIN")) if t["flags"] & b) + ("" if t["flags"] & 2 else "  DEAD-RECKONING"), 0),
                ("imus", "".join(str(i + 1) if t["imu"] & (1 << i) else "-" for i in range(3)), 0),
                ("rocket time", f"{t['t_ms'] / 1000:9.1f} s", 0),
            ]
            for k, v, attr in rows:
                stdscr.addnstr(y, 1, f"{k:>15}  ", col2 - 2)
                stdscr.addnstr(y, 18, v, col2 - 19, attr)
                y += 1
        else:
            stdscr.addstr(y, 1, "waiting for telemetry frames...", C)
        y2 = 3
        if s:
            ph = PHASES[s["phase"]] if s["phase"] < 6 else str(s["phase"])
            armed = bool(s["flags"] & 1)
            pyro = " ".join(("*" if s["on"] & (1 << i) else "F" if s["fired"] & (1 << i) else ".") + str(i + 1) for i in range(6))
            pdm = PD_MODES[s["pd_mode"]] if s["pd_mode"] < 4 else str(s["pd_mode"])
            bq = bool(s["flags"] & 64)
            rows = [
                ("phase", ph.upper(), G if s["phase"] else 0),
                ("armed", "ARMED" if armed else "safe", R | curses.A_BOLD if armed else 0),
                ("pyros", pyro + "   (F fired, * firing)", 0),
                ("main at", f"{s['main_alt']} m", 0),
                ("apogee / vmax", f"{s['apogee']:.0f} m / {s['vmax']:.0f} m/s", 0),
                ("mpu link", "ok" if s["flags"] & 2 else "LOST", 0 if s["flags"] & 2 else R),
                ("usb-pd", pdm + (f"  {s['vbus'] / 1000:.0f} V/{s['iin'] / 1000:.1f} A contract" if s["vbus"] and not bq else ""), 0),
                ("battery", f"{s['vbat'] / 1000:.2f} V {s['ibat'] / 1000:+.2f} A" if bq and s["vbat"] else "no charger bus", 0),
                ("servos", " ".join(str(v) for v in s["servo"]), 0),
            ]
            for k, v, attr in rows:
                stdscr.addnstr(y2, col2, f"{k:>14}  ", w - col2 - 1)
                stdscr.addnstr(y2, col2 + 16, v, w - col2 - 17, attr)
                y2 += 1
        else:
            stdscr.addstr(y2, col2, "waiting for SPU status frames...", C)
        y = max(y, y2) + 1
        stdscr.addstr(y, 1, "EVENTS", Y | curses.A_BOLD)
        stdscr.addstr(y, col2, "LOG", Y | curses.A_BOLD)
        y += 1
        ev = list(rx.events)[-(h - y - 1):]
        lg = list(rx.log)[-(h - y - 1):]
        for i in range(max(len(ev), len(lg))):
            if y + i >= h - 1:
                break
            if i < len(ev):
                stdscr.addnstr(y + i, 1, ev[i], col2 - 2)
            if i < len(lg):
                stdscr.addnstr(y + i, col2, lg[i], w - col2 - 1)
        stdscr.addnstr(h - 1, 0, " q quit   log: " + rx.outfile.name + (f"   relay: {'on' if rx.ws else 'connecting'}" if rx.args.relay else ""), w - 1, curses.A_DIM)
        stdscr.refresh()
        time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--freq", type=float, default=433_000_000, help="LoRa channel in Hz (default 433 MHz)")
    ap.add_argument("--offset", type=float, default=250_000, help="tune this far above the channel to dodge the SDR's DC spike")
    ap.add_argument("--gain", type=float, default=40)
    ap.add_argument("--file", help="replay an rtl_sdr .cu8 capture instead of the live dongle")
    ap.add_argument("--fast", action="store_true", help="replay without real-time pacing")
    ap.add_argument("--relay", help="WebSocket relay URL (wss://api.athena.notaroomba.dev/ws)")
    ap.add_argument("--password", help="relay admin password")
    ap.add_argument("--learn", action="store_true", help="forget the stored PHY conventions and learn them again")
    ap.add_argument("--no-tui", action="store_true", help="plain text output instead of a UI")
    ap.add_argument("--tui", action="store_true", help="curses terminal UI instead of the browser dashboard")
    ap.add_argument("--port", type=int, default=3001, help="local relay WebSocket port (graphical mode)")
    ap.add_argument("--http", type=int, default=8787, help="port for the dashboard files (graphical mode)")
    ap.add_argument("--dashboard", default="https://athena.notaroomba.dev/", help="dashboard URL to open when docs/ is not next to this script")
    ap.add_argument("--no-browser", action="store_true", help="graphical mode without opening a browser window")
    args = ap.parse_args()
    if args.learn and os.path.exists(L.CONV_FILE):
        os.remove(L.CONV_FILE)
    rx = Receiver(args)
    th = threading.Thread(target=rx.run, daemon=True)
    th.start()
    if not args.no_tui and not args.tui:
        # graphical mode: this process is the relay, the web dashboard (served locally when the repo is here,
        # otherwise the public site) is the UI. Works wherever Python + rtl_sdr run.
        threading.Thread(target=rx.local_relay, args=(args.port,), daemon=True).start()
        docs = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs")
        url = f"{args.dashboard}?ws=ws://localhost:{args.port}/ws"
        if os.path.isfile(os.path.join(docs, "index.html")):
            import functools, http.server, socketserver

            class Quiet(http.server.SimpleHTTPRequestHandler):
                def log_message(self, *a, **k):
                    pass

            handler = functools.partial(Quiet, directory=docs)
            socketserver.TCPServer.allow_reuse_address = True
            try:
                httpd = socketserver.TCPServer(("127.0.0.1", args.http), handler)
                threading.Thread(target=httpd.serve_forever, daemon=True).start()
                url = f"http://localhost:{args.http}/?ws=ws://localhost:{args.port}/ws"
            except OSError as e:
                rx.note(f"dashboard server: {e}; using {args.dashboard}")
        rx.note("dashboard: " + url)
        if not args.no_browser:
            import webbrowser
            webbrowser.open(url)
        print(f"Athena ground station running. Dashboard: {url}\nLog: {rx.outfile.name}\nCtrl-C to stop.", flush=True)
        try:
            while rx.running and th.is_alive() and not rx.eof:
                time.sleep(0.5)
                while rx.log:
                    print("  " + rx.log.popleft(), flush=True)
        except KeyboardInterrupt:
            pass
        rx.running = False
    elif args.no_tui:
        seen = 0
        try:
            while rx.running and th.is_alive() and not rx.eof:
                time.sleep(0.5)
                with rx.lock:
                    t, s = rx.telem, rx.spu
                if rx.stats["ok"] != seen:
                    seen = rx.stats["ok"]
                    msg = f"[{time.strftime('%H:%M:%S')}] ok {rx.stats['ok']}/{rx.stats['packets']} level {rx.stats['level_db']:.0f} dB"
                    if t:
                        msg += f" | alt {t['alt']:.1f} m vz {-t['vel'][2]:.1f} m/s fix {t['fix']} sv {t['sv']} lat {t['lat']:.5f} lon {t['lon']:.5f}"
                    if s:
                        msg += f" | spu {PHASES[s['phase']] if s['phase'] < 6 else s['phase']} armed={bool(s['flags'] & 1)} fired=0x{s['fired']:02X} pd {PD_MODES[s['pd_mode']] if s['pd_mode'] < 4 else '?'}"
                    print(msg, flush=True)
                while rx.log:
                    print("   " + rx.log.popleft(), flush=True)
            if args.file and args.fast:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        rx.running = False
    else:
        curses.wrapper(draw, rx)
        rx.running = False


if __name__ == "__main__":
    main()
