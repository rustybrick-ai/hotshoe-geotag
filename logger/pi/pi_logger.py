#!/usr/bin/env python3
"""Field logger for the Raspberry Pi: records the u-blox raw satellite data and the
Arduino's photo times, with no laptop. Standard library only.

  u-blox (ArduSimple USB-C)  -> <session>/rover.ubx   raw UBX (RXM-RAWX, RXM-SFRBX, NAV-PVT), 5 Hz
  Arduino Mega (USB)          -> <session>/arduino.txt every line, prefixed with the Pi's UTC time
  at the start of the next run, or with `convert` -> <session>/rover.obs + rover.nav (RINEX 3.04)

Usage:
  pi_logger.py run        log until stopped (systemd runs this at boot; a new session per start)
  pi_logger.py status     what is being logged right now
  pi_logger.py convert    make RINEX for every session that has none yet (stop the service first,
                          or name the sessions: pi_logger.py convert ~/flights/003_...)

The u-blox is configured over its USB port in RAM only (nothing saved to its flash).
Sessions live in ~/flights/<NNN>_<UTC start>/. Data is flushed to the card every 2 s,
so pulling the power loses at most the last ~2 s.
"""
import datetime as dt, glob, json, os, pwd, select, struct, subprocess, sys, termios, time

# Paths follow the user who owns this script (the service runs as root, but files belong to that user):
#   field Pi (running as a root systemd service): ~owner/flights, ~owner/bin/convbin
#   laptop / desktop (normal user): <repo>/flights, ~/.local/bin/convbin
# Each can be overridden with HOTSHOE_FLIGHTS / HOTSHOE_STATUS / HOTSHOE_CONVBIN.
_SCRIPT = os.path.abspath(__file__)
_OWNER = pwd.getpwuid(os.stat(_SCRIPT).st_uid)
SERVICE = os.geteuid() == 0                     # running as the boot-time service
_PROJECT = os.path.dirname(os.path.dirname(os.path.dirname(_SCRIPT)))   # repo root (logger/pi/ -> ../..)
FLIGHTS = os.environ.get("HOTSHOE_FLIGHTS",
                         os.path.join(_OWNER.pw_dir, "flights") if SERVICE else os.path.join(_PROJECT, "flights"))
STATUS = os.environ.get("HOTSHOE_STATUS", "/run/hotshoe/status.json" if SERVICE else os.path.join(FLIGHTS, "status.json"))
CONVBIN = os.environ.get("HOTSHOE_CONVBIN", next(
    (c for c in (os.path.join(_OWNER.pw_dir, "bin", "convbin"), os.path.join(_OWNER.pw_dir, ".local", "bin", "convbin"))
     if os.path.exists(c)), "convbin"))
RATE_MS = 200                                  # 5 Hz raw (must match CFG-RATE-MEAS in the Arduino sketch)
SYNC_S = 2.0


def own(path):
    """Files made by the root service still belong to the normal user, so they can be copied/deleted."""
    if os.geteuid() == 0:
        try:
            os.chown(path, _OWNER.pw_uid, _OWNER.pw_gid)
        except (KeyError, OSError):
            pass


def log(msg):
    print(f"{dt.datetime.now(dt.timezone.utc):%H:%M:%S} {msg}", flush=True)


# ---------- UBX helpers ----------
def ubx_frame(cls, mid, payload):
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def valset_ram(items):
    """UBX-CFG-VALSET, RAM layer only. items: (key, value, size_bytes)."""
    payload = bytes([0, 0x01, 0, 0])
    for key, value, size in items:
        payload += struct.pack("<I", key) + value.to_bytes(size, "little")
    return ubx_frame(0x06, 0x8A, payload)


F9P_USB_CONFIG = valset_ram([
    (0x10770001, 1, 1),        # CFG-USBINPROT-UBX = 1
    (0x10780001, 1, 1),        # CFG-USBOUTPROT-UBX = 1
    (0x10780002, 0, 1),        # CFG-USBOUTPROT-NMEA = 0
    (0x209102A7, 1, 1),        # CFG-MSGOUT-UBX_RXM_RAWX_USB  = every epoch
    (0x20910234, 1, 1),        # CFG-MSGOUT-UBX_RXM_SFRBX_USB = every message
    (0x20910009, 1, 1),        # CFG-MSGOUT-UBX_NAV_PVT_USB   = every epoch
    (0x30210001, RATE_MS, 2),  # CFG-RATE-MEAS
])


class UbxScanner:
    """Finds complete, checksum-valid UBX frames in a byte stream (for status only;
    the raw file always gets every byte unchanged)."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\xb5\x62")
            if i < 0:
                del self.buf[:-1]
                return out
            del self.buf[:i]
            if len(self.buf) < 6:
                return out
            n = struct.unpack_from("<H", self.buf, 4)[0]
            if n > 8192:
                del self.buf[:2]
                continue
            if len(self.buf) < n + 8:
                return out
            fr = bytes(self.buf[:n + 8])
            a = b = 0
            for x in fr[2:n + 6]:
                a = (a + x) & 0xFF
                b = (b + a) & 0xFF
            if fr[-2] == a and fr[-1] == b:
                out.append((fr[2], fr[3], fr[6:n + 6]))
                del self.buf[:n + 8]
            else:
                del self.buf[:2]


# ---------- serial ----------
def open_serial(path, baud=termios.B115200):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        a = termios.tcgetattr(fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.HUPCL
        a[4] = a[5] = baud
        a[6][termios.VMIN], a[6][termios.VTIME] = 0, 0
        termios.tcsetattr(fd, termios.TCSANOW, a)
    except termios.error as e:                       # not an OSError subclass: callers only catch OSError
        os.close(fd)
        raise OSError(f"cannot set up {path}: {e}") from e
    return fd


def find_device(*needles):
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if any(s.lower() in p.lower() for s in needles):
            return p
    return None


class Device:
    def __init__(self, name, needles, path_out, on_open=None):
        self.name, self.needles, self.path_out, self.on_open = name, needles, path_out, on_open
        self.fd, self.out, self.bytes, self.last_rx, self.last_try = None, None, 0, 0.0, 0.0

    def ensure_open(self, now):
        if self.fd is not None or now - self.last_try < 2:
            return
        self.last_try = now
        path = find_device(*self.needles)
        if not path:
            return
        try:
            self.fd = open_serial(path)
        except OSError as e:
            log(f"{self.name}: cannot open {path}: {e}")
            return
        if self.out is None:
            self.out = open(self.path_out, "ab", buffering=0)
            own(self.path_out)
        log(f"{self.name}: logging {path} -> {self.path_out}")
        if self.on_open:
            self.on_open(self)

    def close(self, why):
        if self.fd is not None:
            log(f"{self.name}: closed ({why})")
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None


# ---------- logging ----------
def new_session():
    os.makedirs(FLIGHTS, exist_ok=True)
    own(FLIGHTS)
    nums = [int(os.path.basename(d).split("_")[0]) for d in glob.glob(os.path.join(FLIGHTS, "[0-9]*_*"))
            if os.path.basename(d).split("_")[0].isdigit()]
    name = f"{max(nums, default=0) + 1:03d}_{dt.datetime.now(dt.timezone.utc):%Y%m%d_%H%M%S}"
    path = os.path.join(FLIGHTS, name)
    os.makedirs(path)
    own(path)
    return path


def write_status(st):
    try:
        os.makedirs(os.path.dirname(STATUS), exist_ok=True)
        tmp = STATUS + ".tmp"
        with open(tmp, "w") as f:
            json.dump(st, f, indent=1)
        os.replace(tmp, STATUS)
    except OSError:
        pass


def maybe_set_clock(year, month, day, hour, minute, sec, valid):
    """The Pi has no clock battery: set it from the GPS once (needs root) so session names are right."""
    if (valid & 0x07) != 0x07 or os.geteuid() != 0:
        return False
    gps = dt.datetime(year, month, day, hour, minute, sec, tzinfo=dt.timezone.utc)
    if abs((dt.datetime.now(dt.timezone.utc) - gps).total_seconds()) > 2:
        subprocess.run(["date", "-u", "-s", gps.strftime("%Y-%m-%d %H:%M:%S")], capture_output=True)
        log(f"system clock set from GPS: {gps:%Y-%m-%d %H:%M:%S} UTC")
    return True


def run():
    earlier = pending_sessions()                      # decided BEFORE this run's session exists, so the
    if earlier:                                       # flight being recorded is never converted half-written
        subprocess.Popen(["nice", "-n", "15", sys.executable, os.path.abspath(__file__), "convert", *earlier])
    session = new_session()
    log(f"session {session}")
    t_start = time.monotonic()
    ubx_scan, clock_set, renamed = UbxScanner(), False, False
    st = {"session": session, "started_utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
          "ublox": {}, "arduino": {}}

    def config_f9p(dev):
        dev.last_cfg = time.monotonic()
        try:
            os.write(dev.fd, F9P_USB_CONFIG)
        except OSError as e:
            dev.close(f"write failed: {e.strerror}")

    def on_f9p_open(dev):
        dev.acked = False                              # a re-plugged receiver is back at factory settings
        config_f9p(dev)

    def rename_to_gps_time(gps_now):
        """The Pi has no clock battery: once GPS time is known, give the session its true start date."""
        nonlocal session
        start = gps_now - dt.timedelta(seconds=time.monotonic() - t_start)
        new = os.path.join(FLIGHTS, f"{os.path.basename(session).split('_')[0]}_{start:%Y%m%d_%H%M%S}")
        if new != session and not os.path.exists(new):
            try:
                os.rename(session, new)                # open files keep working after a rename
            except OSError as e:
                log(f"could not rename session: {e}")
                return
            log(f"session renamed to {os.path.basename(new)} (GPS time)")
            session = new
            for d in devices:
                d.path_out = os.path.join(session, os.path.basename(d.path_out))
            st["session"], st["started_utc"] = session, start.isoformat(timespec="seconds")

    f9p = Device("u-blox", ["u-blox", "1546"], os.path.join(session, "rover.ubx"), on_f9p_open)
    f9p.acked, f9p.last_cfg = False, 0.0
    ard = Device("arduino", ["arduino", "2341"], os.path.join(session, "arduino.txt"))
    devices = [f9p, ard]
    line_buf, last_sync, last_status, rawx = b"", 0.0, 0.0, 0
    while True:
        now = time.monotonic()
        for d in devices:
            d.ensure_open(now)
        fds = [d.fd for d in devices if d.fd is not None]
        ready = select.select(fds, [], [], 0.5)[0] if fds else (time.sleep(0.5) or [])
        for d in devices:
            if d.fd is None or d.fd not in ready:
                continue
            try:
                data = os.read(d.fd, 65536)
            except OSError as e:
                d.close(e.strerror)
                continue
            if not data:
                d.close("unplugged")
                continue
            d.bytes += len(data)
            d.last_rx = now
            if d is f9p:
                f9p.out.write(data)
                for cls, mid, pl in ubx_scan.feed(data):
                    if cls == 0x02 and mid == 0x15:
                        rawx += 1
                        st["ublox"]["satellite_signals"] = pl[11] if len(pl) > 11 else None
                    elif cls == 0x05 and mid == 0x01 and pl[:2] == b"\x06\x8a":
                        f9p.acked = True
                    elif cls == 0x01 and mid == 0x07 and len(pl) >= 92:
                        y, mo, da, h, mi, s, valid = struct.unpack_from("<HBBBBBB", pl, 4)
                        fix, flags, _, sv = pl[20], pl[21], pl[22], pl[23]
                        st["ublox"].update(fix=fix, rtk=flags >> 6, sats=sv,
                                           hacc_m=struct.unpack_from("<I", pl, 40)[0] / 1000)
                        if not clock_set:
                            clock_set = maybe_set_clock(y, mo, da, h, mi, s, valid)
                        if not renamed and (valid & 0x07) == 0x07:
                            renamed = True
                            rename_to_gps_time(dt.datetime(y, mo, da, h, mi, s, tzinfo=dt.timezone.utc))
            else:
                line_buf += data
                stamp = dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")
                while b"\n" in line_buf:
                    raw, line_buf = line_buf.split(b"\n", 1)
                    line = raw.decode(errors="replace").strip()
                    if line:
                        ard.out.write(f"{stamp} {line}\n".encode())
                        if line.startswith(("MARK,", "PHOTO,")):
                            st["arduino"]["last_" + line.split(",")[0].lower()] = line
                        elif line.startswith("ALIVE,"):
                            st["arduino"]["alive"] = line
        if now - last_sync > SYNC_S:                   # power-cut safety
            last_sync = now
            for d in devices:
                if d.out:
                    os.fsync(d.out.fileno())
        if now - last_status > 1:
            last_status = now
            if f9p.fd is not None and not f9p.acked and now - f9p.last_cfg > 3:
                config_f9p(f9p)                        # resend until the receiver acknowledges
            for d, key in ((f9p, "ublox"), (ard, "arduino")):
                st[key].update(connected=d.fd is not None, bytes=d.bytes,
                               receiving=d.fd is not None and now - d.last_rx < 3)
            st["ublox"].update(configured=f9p.fd is not None and f9p.acked, raw_epochs=rawx)
            st["updated_utc"] = dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")
            write_status(st)


# ---------- RINEX ----------
def convert(session):
    ubx = os.path.join(session, "rover.ubx")
    if not os.path.exists(ubx) or os.path.getsize(ubx) == 0:
        return False
    obs, nav = os.path.join(session, "rover.obs"), os.path.join(session, "rover.nav")
    try:   # write to .tmp, rename only when complete: a power cut mid-way never leaves a 'finished' partial file
        r = subprocess.run([CONVBIN, "-r", "ubx", "-v", "3.04", "-od", "-os", "-oi", "-ot",
                            "-o", obs + ".tmp", "-n", nav + ".tmp", ubx], capture_output=True, text=True)
    except OSError as e:
        log(f"cannot run convbin: {e}")
        return False
    ok = r.returncode == 0 and os.path.exists(obs + ".tmp")
    if ok:
        for f in (nav, obs):                           # obs last: its presence means "done"
            if os.path.exists(f + ".tmp"):
                with open(f + ".tmp", "rb") as fh:
                    os.fsync(fh.fileno())
                os.replace(f + ".tmp", f)
                own(f)
    log(f"RINEX {'made' if ok else 'FAILED'} for {os.path.basename(session)}")
    return ok


def pending_sessions():
    """Sessions with raw data but no RINEX, skipping any still being written (changed in the last minute)."""
    out = []
    for s in sorted(glob.glob(os.path.join(FLIGHTS, "[0-9]*_*"))):
        ubx = os.path.join(s, "rover.ubx")
        if (os.path.exists(ubx) and not os.path.exists(os.path.join(s, "rover.obs"))
                and time.time() - os.path.getmtime(ubx) > 60):
            out.append(s)
    return out


def convert_all(sessions=None):
    for s in sessions or pending_sessions():
        convert(s)


def status():
    try:
        st = json.load(open(STATUS))
    except (OSError, ValueError):
        print("not running (no status file)")
        return
    print(json.dumps(st, indent=1))


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "run"
    if cmd == "convert":
        convert_all(sys.argv[2:])     # given sessions, or every session without RINEX (don't run while logging)
    else:
        {"run": run, "status": status}[cmd]()
