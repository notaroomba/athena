#!/usr/bin/env python3
"""Athena log tool (stdlib only).

  athlog.py dump /dev/cu.usbmodemXXXX out.bin    pull the TPU's flash log over USB ('D' command)
  athlog.py csv  log.bin                          split a log (SD file or flash dump) into CSVs
  athlog.py tail /dev/cu.usbmodemXXXX             print decoded frames live from a USB port

Log files are the raw link-frame stream (see firmware/Athena/athena_link.h):
  A5 type len payload... crc16lo crc16hi      CRC-16/CCITT-FALSE over type,len,payload
"""
import os, struct, sys, time, select, termios

SOF, MAXP = 0xA5, 200
PKT_GPS, PKT_STATE, PKT_TELEM, PKT_TEXT = 1, 2, 3, 0x7F

def crc16(b):
    c = 0xFFFF
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c

def frames(data):
    """yield (type, payload, text_before) from a byte stream; resyncs on bad CRC"""
    i, n, text = 0, len(data), bytearray()
    while i < n:
        b = data[i]
        if b != SOF:
            if 32 <= b < 127 or b in (10, 13): text.append(b)
            i += 1; continue
        if i + 3 > n: break
        t, ln = data[i + 1], data[i + 2]
        if ln > MAXP or i + 5 + ln > n: i += 1; continue
        pl = data[i + 3:i + 3 + ln]
        crc = data[i + 3 + ln] | data[i + 4 + ln] << 8
        if crc16(bytes([t, ln]) + pl) != crc: i += 1; continue
        yield t, bytes(pl), bytes(text); text = bytearray()
        i += 5 + ln

STATE = struct.Struct('<I4f3f3f3f3f3ffiiBBH')      # 96 B
GPS   = struct.Struct('<IBBBBiii3iIII')            # 44 B
TELEM = struct.Struct('<Iiiff3h4hBBBB')            # 38 B

def decode(t, p):
    if t == PKT_STATE and len(p) >= 96:
        v = STATE.unpack_from(p)
        return 'state', dict(t_us=v[0], qw=v[1], qx=v[2], qy=v[3], qz=v[4], pN=v[5], pE=v[6], pD=v[7], vN=v[8], vE=v[9], vD=v[10],
                             ax=v[11], ay=v[12], az=v[13], gx=v[14], gy=v[15], gz=v[16], mx=v[17], my=v[18], mz=v[19],
                             baro_alt=v[20], lat0=v[21] / 1e7, lon0=v[22] / 1e7, imu_mask=v[23], flags=v[24], loop_hz=v[25])
    if t == PKT_GPS and len(p) >= 44:
        v = GPS.unpack_from(p)
        return 'gps', dict(itow=v[0], fix=v[1], sv=v[2], ok=v[3], lat=v[5] / 1e7, lon=v[6] / 1e7, hmsl=v[7] / 1e3,
                           vN=v[8] / 1e3, vE=v[9] / 1e3, vD=v[10] / 1e3, hacc=v[11] / 1e3, vacc=v[12] / 1e3, sacc=v[13] / 1e3)
    if t == PKT_TELEM and len(p) >= 38:
        v = TELEM.unpack_from(p)
        return 'telem', dict(t_ms=v[0], lat=v[1] / 1e7, lon=v[2] / 1e7, alt=v[3], baro_alt=v[4], vN=v[5] / 10, vE=v[6] / 10, vD=v[7] / 10,
                             qw=v[8] / 32767, qx=v[9] / 32767, qy=v[10] / 32767, qz=v[11] / 32767, fix=v[12], sv=v[13], flags=v[14], imu_mask=v[15])
    if t == PKT_TEXT:
        return 'text', dict(text=p.decode('ascii', 'replace'))
    return None, None

def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
    a = termios.tcgetattr(fd); a[3] &= ~(termios.ECHO | termios.ICANON); a[0] &= ~termios.ICRNL; termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd

def read_for(fd, seconds):
    buf, t0 = bytearray(), time.time()
    while time.time() - t0 < seconds:
        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            try: buf += os.read(fd, 65536)
            except BlockingIOError: pass
    return bytes(buf)

def cmd_dump(port, out):
    fd = open_port(port); os.write(fd, b'D')
    buf, t0, last = bytearray(), time.time(), time.time()
    while time.time() - t0 < 600:
        r, _, _ = select.select([fd], [], [], 0.5)
        if r:
            try: chunk = os.read(fd, 65536)
            except BlockingIOError: continue
            buf += chunk; last = time.time()
            if b'LOGEND' in buf: break
        elif time.time() - last > 5 and b'LOGDUMP' in buf: break
    os.close(fd)
    i = buf.find(b'LOGDUMP')
    if i < 0: sys.exit('no LOGDUMP header seen (is this the TPU port?)')
    j = buf.index(b'\n', i) + 1; size = int(buf[i:j].split()[1])
    k = buf.find(b'\r\nLOGEND', j); body = bytes(buf[j:k if k > 0 else j + size])
    open(out, 'wb').write(body)
    n = sum(1 for _ in frames(body))
    print(f'wrote {len(body)} bytes ({size} reported) to {out}: {n} frames')

def cmd_csv(path):
    data = open(path, 'rb').read(); base = os.path.splitext(path)[0]
    files, counts = {}, {}
    for t, p, text in frames(data):
        kind, row = decode(t, p)
        if not kind: continue
        if kind not in files:
            files[kind] = open(f'{base}_{kind}.csv', 'w'); files[kind].write(','.join(row.keys()) + '\n')
        files[kind].write(','.join(f'{v:.6f}' if isinstance(v, float) else str(v).replace(',', ';') for v in row.values()) + '\n')
        counts[kind] = counts.get(kind, 0) + 1
    for f in files.values(): f.close()
    print({k: v for k, v in counts.items()}, '->', ', '.join(f'{base}_{k}.csv' for k in files))

def cmd_tail(port):
    fd = open_port(port)
    try:
        while True:
            for t, p, text in frames(read_for(fd, 1.0)):
                kind, row = decode(t, p)
                if kind == 'state': print(f"state t={row['t_us']/1e6:8.2f}s alt={-row['pD']:7.1f} vD={row['vD']:6.1f} imu={row['imu_mask']} flags=0x{row['flags']:02x} hz={row['loop_hz']}")
                elif kind == 'gps':  print(f"gps   fix={row['fix']} sv={row['sv']} {row['lat']:.6f},{row['lon']:.6f} h={row['hmsl']:.1f}")
                elif kind == 'text': print('text ', row['text'][:120])
    except KeyboardInterrupt: pass
    finally: os.close(fd)

if __name__ == '__main__':
    a = sys.argv[1:]
    if len(a) == 3 and a[0] == 'dump': cmd_dump(a[1], a[2])
    elif len(a) == 2 and a[0] == 'csv': cmd_csv(a[1])
    elif len(a) == 2 and a[0] == 'tail': cmd_tail(a[1])
    else: print(__doc__); sys.exit(1)
