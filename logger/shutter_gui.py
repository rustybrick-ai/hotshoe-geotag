#!/usr/bin/env python3
"""Big-screen shutter monitor: the window flashes and counts every photo, and shows
the GPS time and position of each exposure, timed by the Arduino against the
u-blox's once-per-second satellite pulse (PPS).

Usage: python3 shutter_gui.py [/dev/ttyACM0]
Saves every photo to logs/photos_YYYYmmdd_HHMMSS.csv and every raw Arduino line
to logs/raw_YYYYmmdd_HHMMSS.txt. Opening the port restarts the Arduino, so the
photo count starts at 0.
"""
import collections, csv, datetime as dt, io, os, queue, select, struct, sys, termios, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "pi"))
import pi_logger as pl                    # shared: device finder, u-blox USB config, sessions

PORT = sys.argv[1] if len(sys.argv) > 1 else None    # None: find the Arduino by its USB id
IDLE_BG, FLASH_BG, PHOTO_BG = "#1b1f24", "#ffffff", "#1f9d55"
GIVE_UP_S = 6.0            # the Arduino answers within ~1.1 s (3 s worst); then wait for the position
FIX = {0: "no fix", 1: "DR", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "time only"}
CARR = {0: "", 1: "RTK float", 2: "RTK fixed"}
TL_BG, TL_SPAN_S = "#0f1317", 2.5     # timeline: background, seconds of GPS time shown
GPS_EPOCH, GPS_UTC_LEAP_S, WEEK_S = dt.datetime(1980, 1, 6, tzinfo=dt.timezone.utc), 18, 604800
CSV_HEAD = ("pc_time_utc,n,t_fall_us,width_us,gps_week,tow_s,utc_time,pps_period_us,"
            "lat_deg,lon_deg,hmsl_m,fix,rtk,hacc_m,note\n")


def gps_to_utc(week, tow):
    return GPS_EPOCH + dt.timedelta(weeks=week, seconds=tow - GPS_UTC_LEAP_S)


def gps_week_for(tow, now_utc):
    """GPS week of a time-of-week, taken from the PC clock (only needs to be right within days)."""
    gps_now = (now_utc - GPS_EPOCH).total_seconds() + GPS_UTC_LEAP_S
    return round((gps_now - tow) / WEEK_S)


class Photo:
    def __init__(self, n, t_us, width, wall, arrived):
        self.n, self.t_us, self.width, self.wall, self.arrived = n, t_us, width, wall, arrived
        self.answered = False                     # the Arduino sent the PHOTO line
        self.week = self.tow = self.period = None  # GPS week, time of week (s), Arduino µs per GPS second
        self.pos = None                            # (lat, lon, hmsl, fix, carr, hacc)
        self.note, self.done = "", False


class Tracker:
    """Collects MARK / PHOTO / PVT lines into finished photos with GPS time and an
    interpolated position. Pure logic (no GUI, no serial) so it can be tested."""

    def __init__(self):
        self.photos, self.pvts = {}, collections.deque(maxlen=200)
        self.pps = collections.deque(maxlen=10)    # GPS seconds of recent pulses
        self.f9p_ok = self.pps_ok = False

    def mark(self, n, t_us, width, wall, now):
        self.photos[n] = Photo(n, t_us, width, wall, now)
        return self.photos[n]

    def photo(self, n, sec, frac_ns, period_us, now_utc):
        p = self.photos.get(n)
        if p is None:
            return None
        p.answered = True
        if sec < 0:
            p.note = "no GPS time (no satellite pulse around it: no fix, or PPS wire)"
        else:
            p.tow = sec + frac_ns / 1e9
            p.week = gps_week_for(p.tow, now_utc)
            p.period = period_us
        return p

    def pvt(self, itow_ms, fix, carr, sv, lat_e7, lon_e7, hmsl_mm, hacc_mm):
        self.pvts.append((itow_ms / 1e3, fix, carr, sv, lat_e7 / 1e7, lon_e7 / 1e7, hmsl_mm / 1e3, hacc_mm / 1e3))

    def position_at(self, tow):
        """Linear interpolation between the NAV-PVT epochs either side of tow, or None if not yet available."""
        near = lambda e: e if abs(e[0] - tow) < WEEK_S / 2 else (e[0] + WEEK_S * (1 if e[0] < tow else -1),) + e[1:]
        pvts = [near(e) for e in self.pvts]                  # unwrap the weekly time-of-week rollover
        for a, b in zip(pvts, pvts[1:]):
            if a[0] <= tow <= b[0] and b[0] - a[0] < 1.0:
                f = (tow - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                lerp = lambda i: a[i] + f * (b[i] - a[i])
                worse = a if (a[1], a[2]) < (b[1], b[2]) else b      # report the weaker fix of the two
                return (lerp(4), lerp(5), lerp(6), worse[1], worse[2], max(a[7], b[7]))
        return None

    def flush(self, note):
        """The Arduino restarted (its photo numbers start again at 1): finish everything pending now."""
        out = [self.photos[n] for n in sorted(self.photos)]
        for p in out:
            p.note = p.note or note
            p.done = True
        self.photos.clear()
        return out

    def finished(self, now):
        """Photos that are complete (or timed out); each is returned once, in order."""
        out = []
        for n in sorted(self.photos):
            p = self.photos[n]
            if p.tow is not None and p.pos is None:
                p.pos = self.position_at(p.tow)
            ready = p.answered and (p.tow is None or p.pos is not None)
            if ready or now - p.arrived > GIVE_UP_S:
                if not p.answered:
                    p.note = "no answer from the Arduino about GPS time"
                elif p.tow is not None and p.pos is None:
                    p.note = "no position at that time"
                p.done = True
                out.append(p)
        for p in out:
            del self.photos[p.n]
        return out


def csv_row(p):
    pc = p.wall.astimezone(dt.timezone.utc).isoformat(timespec="milliseconds")
    lat, lon, h, fix, carr, hacc = p.pos or ("",) * 6
    utc = gps_to_utc(p.week, p.tow).strftime("%Y-%m-%dT%H:%M:%S.%fZ") if p.tow is not None else ""
    f = lambda v, fmt: "" if v in (None, "") else format(v, fmt)
    row = [pc, p.n, p.t_us, p.width, f(p.week, "d"), f(p.tow, ".9f"), utc, f(p.period, "d"),
           f(lat, ".9f"), f(lon, ".9f"), f(h, ".3f"), FIX.get(fix, fix), CARR.get(carr, carr), f(hacc, ".3f"), p.note]
    out = io.StringIO()
    csv.writer(out, lineterminator="\n").writerow(row)
    return out.getvalue()


def open_port(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY)
    try:
        a = termios.tcgetattr(fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.HUPCL   # HUPCL: each open restarts the Mega
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN], a[6][termios.VTIME] = 1, 0   # block until data (other tools may leave VMIN=0)
        termios.tcsetattr(fd, termios.TCSANOW, a)
    except termios.error:
        os.close(fd)
        raise
    return os.fdopen(fd, "rb", buffering=0)


def reader(events):
    """Serial thread: pushes ('status', text) and ('line', text, wall_time, monotonic) events."""
    while True:
        try:
            port = PORT or pl.find_device("arduino", "2341")
            if not port:
                raise OSError("Arduino USB not found")
            with open_port(port) as ser:
                events.put(("status", "Arduino connected — waiting for the shutter…"))
                buf = b""
                while True:
                    chunk = ser.read(256)
                    if not chunk:
                        raise OSError("port closed")
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode(errors="replace").strip()
                        if line:
                            events.put(("line", line, dt.datetime.now().astimezone(), time.monotonic()))
        except (OSError, termios.error) as e:
            events.put(("status", f"Arduino NOT connected ({getattr(e, 'strerror', None) or e}) — retrying…"))
            time.sleep(2)


def ublox_reader(events, session):
    """Never dies silently: any unexpected error is shown in the window and recording restarts."""
    while True:
        try:
            _ublox_reader(events, session)
        except Exception as e:
            events.put(("raw", f"PPK raw data: ERROR {e!r}, restarting…"))
            time.sleep(2)


def _ublox_reader(events, session):
    """Records the u-blox's raw satellite data (from its own USB port) to session/rover.ubx for PPK,
    exactly like the field Pi does. Pushes ('raw', text) status events."""
    out = open(os.path.join(session, "rover.ubx"), "ab", buffering=0)
    total = 0
    while True:
        dev = pl.find_device("u-blox", "1546")
        if not dev:
            events.put(("raw", "PPK raw data: u-blox USB NOT connected (plug the ArduSimple USB-C into the laptop)"))
            time.sleep(2)
            continue
        try:
            fd = pl.open_serial(dev)
        except OSError as e:
            events.put(("raw", f"PPK raw data: cannot open u-blox USB ({e})"))
            time.sleep(2)
            continue
        scan, acked, rawx, last_cfg, last_sync, last_ev = pl.UbxScanner(), False, 0, 0.0, 0.0, 0.0
        try:
            while True:
                now = time.monotonic()
                if not acked and now - last_cfg > 3:
                    os.write(fd, pl.F9P_USB_CONFIG)      # RAM only; resent until acknowledged
                    last_cfg = now
                if select.select([fd], [], [], 0.5)[0]:
                    data = os.read(fd, 65536)
                    if not data:
                        raise OSError("u-blox unplugged")
                    out.write(data)
                    total += len(data)
                    for cls, mid, p in scan.feed(data):
                        if cls == 0x05 and mid == 0x01 and p[:2] == b"\x06\x8a":
                            acked = True
                        elif cls == 0x02 and mid == 0x15:
                            rawx += 1
                if now - last_sync > 2:
                    os.fsync(out.fileno())
                    last_sync = now
                if now - last_ev > 1:
                    last_ev = now
                    state = "RECORDING" if acked and rawx else ("configuring u-blox…" if not acked else "waiting for raw data…")
                    events.put(("raw", f"PPK raw data: {state}  {rawx} epochs, {total / 1e6:.1f} MB"))
        except OSError as e:
            events.put(("raw", f"PPK raw data: u-blox connection lost ({e}), retrying…"))
            time.sleep(1)
        finally:
            os.close(fd)


class App:
    def __init__(self, root, events, session):
        import tkinter as tk
        self.root, self.events, self.m = root, events, Tracker()
        self.last_n = None
        self.session = session
        self.csv_path = os.path.join(session, "photos_live.csv")
        self.csv = open(self.csv_path, "w", buffering=1)
        self.csv.write(CSV_HEAD)
        self.raw = open(os.path.join(session, "arduino.txt"), "a", buffering=1)   # same format as the Pi's

        root.title("Shutter monitor")
        root.geometry("1050x800")
        root.configure(bg=IDLE_BG)
        self.widgets = []

        def label(**kw):
            w = tk.Label(root, **{"bg": IDLE_BG, "fg": "white", **kw})
            self.widgets.append(w)
            return w

        label(text="Photos detected from the camera's hot shoe", font=("Helvetica", 20)).pack(pady=(20, 0))
        self.big = label(text="0", font=("Helvetica", 140, "bold"))
        self.big.pack()
        self.msg = label(text="Take a photo", font=("Helvetica", 30, "bold"))
        self.msg.pack()
        self.where = label(text="", font=("Helvetica", 18))
        self.where.pack()
        self.history = label(text="", font=("Courier", 12), justify="left")
        self.history.pack(pady=12)
        self.status = label(text="Connecting…", font=("Helvetica", 13), fg="#dddddd")
        self.status.pack(side="bottom", pady=(0, 10))
        self.rawlbl = label(text="PPK raw data: starting…", font=("Helvetica", 13), fg="#dddddd")
        self.rawlbl.pack(side="bottom", pady=(0, 2))
        self.gnss = label(text="GPS: waiting…", font=("Helvetica", 16, "bold"), fg="#dddddd")
        self.gnss.pack(side="bottom", pady=(0, 4))
        self.tl = tk.Canvas(root, height=240, bg=TL_BG, highlightthickness=0)
        self.tl.pack(side="bottom", fill="x", padx=20, pady=(0, 8))
        self.tl_marks = collections.deque(maxlen=20)           # (tow, label) of recent photos
        self.tl_caption = ""
        self.lines = []
        root.after(200, self.draw_timeline)
        root.after(20, self.poll)

    def set_bg(self, color, fg="white"):
        self.root.configure(bg=color)
        for w in self.widgets:
            w.configure(bg=color, fg=fg if w not in (self.status, self.gnss, self.rawlbl) else "#dddddd")

    def flash(self, title):
        self.msg.configure(text=title)
        self.where.configure(text="getting GPS time (waits for the next satellite pulse)…" if self.m.pps_ok else "")
        self.root.bell()
        self.set_bg(FLASH_BG, fg="black")                      # camera-flash white…
        self.root.after(120, lambda: self.set_bg(PHOTO_BG))    # …then green…
        if hasattr(self, "_fade"):
            self.root.after_cancel(self._fade)
        self._fade = self.root.after(1500, lambda: self.set_bg(IDLE_BG))  # …then back

    def on_line(self, line, wall, now):
        self.raw.write(f"{wall.astimezone(dt.timezone.utc).isoformat(timespec='milliseconds')} {line}\n")
        if line.startswith("#"):
            if line.startswith("# hotshoe_logger"):
                self.last_n = None                             # Arduino restarted: its count restarts at 1
                self.m.f9p_ok = self.m.pps_ok = False
                for p in self.m.flush("Arduino restarted before GPS time arrived"):
                    self.on_done(p)
            elif not line.startswith("# MARK"):                # Arduino status/diagnostic message
                self.status.configure(text=line[2:] + f"   |   session: {self.session}")
                if line.startswith("# F9P"):
                    self.m.f9p_ok = "configured" in line
            return
        f = line.split(",")
        try:
            if f[0] == "MARK":
                n = int(f[1])
                gap = self.last_n is not None and n != self.last_n + 1
                self.last_n = n
                self.m.mark(n, int(f[2]), int(f[3]), wall, now)
                self.big.configure(text=str(n))
                self.flash(f"PHOTO {n} TAKEN  {wall:%H:%M:%S}" + ("   (COUNT GAP!)" if gap else ""))
            elif f[0] == "PHOTO":
                self.m.photo(int(f[1]), int(f[2]), int(f[3]), int(f[4]), wall.astimezone(dt.timezone.utc))
            elif f[0] == "PPS":
                self.m.pps.append(int(f[2]))
                self.m.pps_ok = True
            elif f[0] == "PVT" and len(f) >= 9:
                self.m.pvt(*map(int, f[1:9]))
                self.m.f9p_ok = True
                _, fix, carr, sv, *_rest, hacc = self.m.pvts[-1]
                rtk = f", {CARR[carr]}" if CARR.get(carr) else ""
                pulse = "satellite pulse OK" if self.m.pps_ok else "NO satellite pulse on D3"
                self.gnss.configure(text=f"GPS: {FIX.get(fix, fix)}{rtk}, {sv} satellites, ±{hacc * 100:.1f} cm   |   {pulse}")
            elif f[0] == "ALIVE":
                self.m.f9p_ok = len(f) > 4 and f[4] == "1"
                self.m.pps_ok = len(f) > 5 and f[5] == "1"
                if not self.m.f9p_ok:
                    self.gnss.configure(text="GPS: NOT connected (check TX1→19, RX1→18, IOREF→5V, GND, power)")
        except (ValueError, IndexError, TypeError):
            pass                                               # partial/garbled line (e.g. after a reset)

    def on_done(self, p):
        self.csv.write(csv_row(p))
        txt = f"#{p.n:<4} {p.wall:%H:%M:%S.%f}"[:-3]
        if p.tow is not None:
            txt += f"  GPS {gps_to_utc(p.week, p.tow):%H:%M:%S.%f} UTC"
            self.explain(p)
        if p.pos:
            lat, lon, h, fix, carr, hacc = p.pos
            txt += f"  {lat:.7f}, {lon:.7f}  {h:.2f} m"
            self.where.configure(text=f"{lat:.7f}, {lon:.7f}   height {h:.2f} m   "
                                      f"{CARR.get(carr) or FIX.get(fix, fix)} ±{hacc * 100:.1f} cm")
        elif p.tow is not None:
            self.where.configure(text=f"GPS time {gps_to_utc(p.week, p.tow):%H:%M:%S.%f} UTC (no position)")
        else:
            self.where.configure(text=p.note)
        if p.note:
            txt += f"  [{p.note}]"
        self.lines = ([txt] + self.lines)[:7]
        self.history.configure(text="\n".join(self.lines))

    def explain(self, p):
        """Put the photo on the timeline and caption, in plain words, how its time was found."""
        name = f"Photo {p.n}"
        self.tl_marks.append((p.tow, name))
        sec = int(p.tow)
        utc_sec, utc = gps_to_utc(p.week, sec), gps_to_utc(p.week, p.tow)
        drift = (p.period - 1_000_000) / 1e4                  # % the Arduino clock is off
        txt = (f"{name} was taken {(p.tow - sec) * 1000:.3f} ms after the satellite pulse for "
               f"{utc_sec:%H:%M:%S} UTC  →  {utc:%H:%M:%S.%f} UTC.   The Arduino's own clock measured that "
               f"satellite second as {p.period:,} µs ({drift:+.3f} %), and the time is corrected for it.")
        before = [e[0] for e in self.m.pvts if e[0] <= p.tow]
        after = [e[0] for e in self.m.pvts if e[0] > p.tow]
        if before and after:
            a, b = before[-1], after[0]
            txt += (f"  Position: {(p.tow - a) / (b - a) * 100:.0f}% of the way between the GPS fixes "
                    f"{(p.tow - a) * 1000:.0f} ms before and {(b - p.tow) * 1000:.0f} ms after.")
        self.tl_caption = txt
        self.draw_timeline(reschedule=False)

    def draw_timeline(self, reschedule=True):
        c = self.tl
        c.delete("all")
        W = max(c.winfo_width(), 400)
        c.create_text(12, 14, anchor="w", fill="#9aa4ad", font=("Helvetica", 12),
                      text="Satellite time:  white line = GPS second pulse   •   dot = GPS position (5/s)"
                           "   •   blue = photo taken")
        if not self.m.pvts:
            c.create_text(W / 2, 90, fill="#9aa4ad", font=("Helvetica", 16),
                          text="Waiting for data from the u-blox receiver…")
        else:
            end = self.m.pvts[-1][0] + 0.2
            start = end - TL_SPAN_S
            x = lambda t: 20 + (t - start) / (end - start) * (W - 40)
            y = 90
            c.create_line(20, y, W - 20, y, fill="#3a434c", width=2)
            for sec in self.m.pps:
                if start <= sec <= end:
                    c.create_line(x(sec), y - 40, x(sec), y + 30, fill="white", width=2)
                    c.create_text(x(sec), y + 42, fill="white", font=("Helvetica", 10, "bold"),
                                  text=f"pulse {gps_to_utc(gps_week_for(sec, dt.datetime.now(dt.timezone.utc)), sec):%H:%M:%S}")
            for tow, fix, carr, *_ in self.m.pvts:
                if start <= tow <= end:
                    col = "#2ecc71" if carr == 2 else "#f1c40f" if fix >= 3 or carr == 1 else "#e74c3c"
                    c.create_oval(x(tow) - 5, y - 5, x(tow) + 5, y + 5, fill=col, outline="")
            for tow, name in self.tl_marks:
                if start <= tow <= end:
                    X = x(tow)
                    c.create_line(X, y - 45, X, y + 30, fill="#3da5ff", width=3)
                    c.create_polygon(X, y - 12, X + 10, y, X, y + 12, X - 10, y, fill="#3da5ff", outline="white")
                    c.create_text(X, y - 55, fill="#3da5ff", font=("Helvetica", 13, "bold"), text=name)
            c.create_text(W - 12, 150, anchor="e", fill="#7f8a94", font=("Helvetica", 10),
                          text="dots: green = RTK fixed (cm)   yellow = normal GPS (m)   red = no fix")
        if self.tl_caption:
            c.create_text(12, 200, anchor="w", width=W - 24, fill="white", font=("Helvetica", 12),
                          text=self.tl_caption)
        if reschedule:
            self.root.after(100, self.draw_timeline)

    def poll(self):
        try:
            while True:
                ev = self.events.get_nowait()
                if ev[0] == "status":
                    self.status.configure(text=f"{ev[1]}   |   session: {self.session}")
                elif ev[0] == "raw":
                    self.rawlbl.configure(text=ev[1])
                elif ev[0] == "line":
                    self.on_line(*ev[1:])
        except queue.Empty:
            pass
        except Exception as e:                                 # never let one bad line stop the window
            self.status.configure(text=f"error handling a line: {e!r}")
        try:
            for p in self.m.finished(time.monotonic()):
                self.on_done(p)
        except Exception as e:
            self.status.configure(text=f"error finishing a photo: {e!r}")
        finally:
            self.root.after(20, self.poll)


if __name__ == "__main__":
    import tkinter as tk
    events = queue.Queue()
    session = pl.new_session()                 # flights/NNN_<UTC>: arduino.txt + rover.ubx, ready for ppk.py
    threading.Thread(target=reader, args=(events,), daemon=True).start()
    threading.Thread(target=ublox_reader, args=(events, session), daemon=True).start()
    root = tk.Tk()
    App(root, events, session)
    root.mainloop()
