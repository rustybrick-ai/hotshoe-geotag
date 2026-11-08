#!/usr/bin/env python3
"""PPK geotagging: a logged session + your base station + the photo folder -> Agisoft Metashape.

  rover raw   (session/rover.ubx)                  --convbin-->  RINEX
  base raw    (RTKBase .ubx / .ubx.zip, or RINEX)  --convbin-->  RINEX
  rnx2rtkp (RTKLIB demo5, kinematic, forward+backward)  -->  session/ppk.pos   (5 Hz trajectory)
  photo GPS times (session/arduino.txt PHOTO lines)     -->  position interpolated at each exposure
  photo files, paired with pulses by EXIF capture time  -->  metashape.csv  (+ geotagged copies)

Usage:
  python ppk.py SESSION_DIR --base BASE_FILE [BASE_FILE...] --base-pos LAT LON HEIGHT
                [--photos PHOTO_DIR] [--write-exif OUT_DIR] [--antenna-above-camera METRES] [--out CSV]

  --base-pos   the base ANTENNA position: latitude, longitude (degrees), ellipsoidal height (m, WGS84),
               e.g. from a PPP result. Every photo inherits any error in it.
  --base-pos-rinex   use the position in the base RINEX header instead (only if it is a surveyed one!)
  --photos     folder with the images (sub-folders included). Each image is paired with its shutter
               pulse by capture time (camera clock offset found automatically), so missing or extra
               photos are reported instead of shifting the rest. Without it: labels photo_0001, ...
  --write-exif write geotagged COPIES of the matched images into OUT_DIR (originals untouched)
  --antenna-above-camera  vertical distance from the antenna to the camera (m); subtracted from the
               height (assumes the drone is roughly level; horizontal offsets need attitude, not done).

Tools: RTKLIB demo5 (convbin, rnx2rtkp) and exiftool, found in ./tools next to this script
(see setup_tools.py), on the PATH, or in ~/.local/bin.
"""
import argparse, bisect, datetime as dt, glob, gzip, os, re, shutil, subprocess, sys, tempfile, time, zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import photomatch


def find_tool(name):
    exe = name + (".exe" if os.name == "nt" else "")
    for d in (os.path.join(HERE, "tools"), os.path.join(HERE, "tools", "bin")):
        if os.path.exists(os.path.join(d, exe)):
            return os.path.join(d, exe)
    return shutil.which(name) or os.path.join(os.path.expanduser("~/.local/bin"), exe)


CONVBIN, RNX2RTKP, CRX2RNX = find_tool("convbin"), find_tool("rnx2rtkp"), find_tool("crx2rnx")
photomatch.EXIFTOOL = find_tool("exiftool")
GPS_EPOCH, LEAP_S, WEEK_S = dt.datetime(1980, 1, 6, tzinfo=dt.timezone.utc), 18, 604800
F9P_CONF = os.path.join(HERE, "configs", "f9p_ppk.conf")   # rtklibexplorer's recommended F9P PPK settings
Q_NAME = {1: "fixed", 2: "float", 3: "sbas", 4: "dgps", 5: "single", 6: "ppp"}
ACC_FLOOR_M = {1: 0.01, 2: 0.10, 5: 1.0}   # smallest accuracy reported per solution type (m)

# RTKLIB demo5 settings for u-blox F9P rover + F9P base (dual frequency, all constellations).
CONF = """pos1-posmode       =kinematic
pos1-frequency     =l1+l2
pos1-soltype       =combined
pos1-elmask        =15
pos1-dynamics      =on
pos1-ionoopt       =brdc
pos1-tropopt       =saas
pos1-sateph        =brdc
pos1-navsys        =45
pos2-armode        =fix-and-hold
pos2-gloarmode     =fix-and-hold
pos2-bdsarmode     =on
pos2-arthres       =3
pos2-arfilter      =on
out-solformat      =llh
out-outhead        =on
out-timesys        =gpst
out-timeform       =tow
out-timendec       =3
out-height         =ellipsoidal
misc-timeinterp    =on
out-outstat        =off
{base}
"""


def die(msg):
    sys.exit(f"ppk: {msg}")


def check_ansi(path):
    """RTKLIB's Windows programs only understand file names in the Windows ANSI code page."""
    if os.name == "nt" and path:
        try:
            path.encode("mbcs", "strict")
        except UnicodeEncodeError:
            die(f"{path}: RTKLIB cannot open paths with these characters on Windows; "
                "move the files to a folder with a plain name, e.g. C:\\flights")


def ascii_dir():
    """Temp folder whose path RTKLIB can open (a user name like 'Łukasz' breaks %TEMP% for it)."""
    t = tempfile.gettempdir()
    if os.name != "nt":
        return None
    try:
        t.encode("mbcs", "strict")
        return None
    except UnicodeEncodeError:
        d = os.path.join(os.environ.get("SystemDrive", "C:") + os.sep, "ppk_tmp")
        os.makedirs(d, exist_ok=True)
        return d


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        die(f"{os.path.basename(cmd[0])} failed:\n{r.stderr[-2000:]}")
    return r


RINEX_GLOBS = ("obs", "nav", "rnx", "crx", "gz", "zip", "ubx")


def rinex_type(path):
    """'obs', 'nav' or None from the RINEX header (works for any file name: .26o, *_MO.rnx, rover.obs ...)."""
    try:
        with open(path, "r", errors="replace") as fh:
            first = fh.readline()
    except OSError:
        return None
    if "RINEX VERSION / TYPE" not in first:
        return "crx" if "COMPACT RINEX" in first or "CRINEX" in first else None
    kind = first[20:21].upper()
    if kind == "O":
        return "obs"
    if kind in "NGHLJCI" and kind:
        return "nav"
    return None                                        # meteo etc.


def gather(paths, work, role):
    """Rover or base input files/folders -> (list of RINEX obs, list of RINEX nav).
    Accepts raw u-blox (.ubx, .zip of .ubx), RINEX 2/3 obs+nav with any name, Hatanaka (.crx/.YYd)
    and .gz-compressed files. Other files (e.g. arduino.txt) are ignored."""
    files, obs, nav = [], [], []
    for p in paths:
        if not os.path.exists(p) and any(c in p for c in "*?"):   # Windows' cmd does not expand wildcards
            matches = sorted(glob.glob(p))
            if not matches:
                die(f"{p}: no files match")
            o, n = gather(matches, work, role + "g")
            obs += o
            nav += n
            continue
        if os.path.isdir(p):
            files += sorted(os.path.join(p, f) for f in os.listdir(p) if os.path.isfile(os.path.join(p, f)))
        elif os.path.exists(p):
            files.append(p)
        else:
            die(f"{p}: not found")
    for i, f in enumerate(files):
        tag = f"{role}_{i:03d}"
        low = f.lower()
        if low.endswith(".zip"):
            d = os.path.join(work, tag)
            with zipfile.ZipFile(f) as z:
                z.extractall(d)
            o, n = gather([os.path.join(r, x) for r, _, xs in os.walk(d) for x in xs], work, tag)
            obs += o
            nav += n
            continue
        if low.endswith(".gz"):
            dest = os.path.join(work, tag + "_" + os.path.basename(f)[:-3])
            with gzip.open(f, "rb") as src, open(dest, "wb") as out:
                shutil.copyfileobj(src, out)
            f, low = dest, dest.lower()
        if low.endswith(".ubx"):
            o, n = os.path.join(work, tag + ".obs"), os.path.join(work, tag + ".nav")
            run([CONVBIN, "-r", "ubx", "-v", "3.04", "-od", "-os", "-oi", "-ot", "-o", o, "-n", n, f])
            if os.path.exists(o) and os.path.getsize(o) > 1000:
                obs.append(o)
            else:
                print(f"note: {os.path.basename(f)} has no raw observations (RXM-RAWX), skipped", file=sys.stderr)
            if os.path.exists(n):
                nav.append(n)
            continue
        kind = rinex_type(f)
        if kind == "crx" or re.search(r"\.\d\dd$", low):
            if not os.path.exists(CRX2RNX):
                die(f"{f} is Hatanaka-compressed RINEX: crx2rnx is needed (python setup_tools.py)")
            dest = os.path.join(work, tag + ".obs")
            with open(dest, "wb") as out:
                r = subprocess.run([CRX2RNX, "-", ], stdin=open(f, "rb"), stdout=out, stderr=subprocess.PIPE)
            if r.returncode != 0:
                die(f"crx2rnx failed on {f}: {r.stderr.decode(errors='replace')[-500:]}")
            obs.append(dest)
        elif kind == "obs":
            obs.append(f)
        elif kind == "nav":
            nav.append(f)
        elif low.endswith(".z"):
            print(f"note: {os.path.basename(f)} is Unix-compressed (.Z): unpack it first (e.g. with 7-Zip)",
                  file=sys.stderr)
        elif not low.endswith((".txt", ".csv", ".pos", ".stat", ".json", ".conf")):
            print(f"note: {os.path.basename(f)} is not raw u-blox or RINEX data, skipped", file=sys.stderr)
    return obs, nav


def approx_llh(obs_file):
    """APPROX POSITION XYZ from a RINEX obs header -> (lat, lon, h) or None."""
    import math
    try:
        with open(obs_file, errors="replace") as fh:
            for line in fh:
                if "APPROX POSITION XYZ" in line:
                    x, y, z = (float(v) for v in line[:42].split())
                    break
                if "END OF HEADER" in line:
                    return None
            else:
                return None
    except (OSError, ValueError):
        return None
    if abs(x) + abs(y) + abs(z) < 1e3:
        return None
    a_, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    lon = math.atan2(y, x)
    p_ = math.hypot(x, y)
    lat = math.atan2(z, p_ * (1 - e2))
    for _ in range(6):
        n = a_ / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        h = p_ / math.cos(lat) - n
        lat = math.atan2(z, p_ * (1 - e2 * n / (n + h)))
    return math.degrees(lat), math.degrees(lon), h


def distance_m(lat1, lon1, lat2, lon2):
    import math
    k = math.pi / 180
    dx = (lon2 - lon1) * k * 6371000 * math.cos((lat1 + lat2) / 2 * k)
    dy = (lat2 - lat1) * k * 6371000
    return math.hypot(dx, dy)


def one_receiver(obs, work, role):
    """rnx2rtkp treats every obs file argument as ANOTHER receiver, so several files of one receiver
    (hourly logs, split files) are copied to work/<role>_NN.obs and passed as one wildcard, which
    RTKLIB expands itself (also on Windows)."""
    if len(obs) == 1:
        return obs[0]
    for i, o in enumerate(obs):
        dest = os.path.join(work, f"{role}_all_{i:03d}.obs")
        if os.path.abspath(o) != dest:
            shutil.copyfile(o, dest)
    return os.path.join(work, f"{role}_all_*.obs")


def read_pos(path):
    """rnx2rtkp .pos with time format tow -> sorted list of (gps_seconds, lat, lon, h, Q, ns, sdn, sde, sdu)."""
    out = []
    for line in open(path):
        if line.startswith("%") or not line.strip():
            continue
        f = line.split()
        week, tow = int(f[0]), float(f[1])
        out.append((week * WEEK_S + tow, float(f[2]), float(f[3]), float(f[4]), int(f[5]), int(f[6]),
                    float(f[7]), float(f[8]), float(f[9])))
    out.sort()
    return out


def read_photos(path):
    """arduino.txt -> list of dicts in shutter order: n, tow (or None), pi_time."""
    photos, by_n = [], {}
    for line in open(path, errors="replace"):
        m = re.match(r"(\S+) (MARK|PHOTO),(.*)", line.strip())
        if not m:
            if " # hotshoe_logger" in line:
                by_n = {}                       # Arduino restarted: its numbering starts again
            continue
        stamp, kind, rest = m.groups()
        f = rest.split(",")
        try:
            n = int(f[0])
            if kind == "MARK":
                p = {"n": n, "tow": None, "pi_time": stamp, "period_us": None}
                photos.append(p)
                by_n[n] = p
            elif n in by_n:
                sec, frac_ns, period = int(f[1]), int(f[2]), int(f[3])
                if sec >= 0:
                    by_n[n]["tow"] = sec + frac_ns / 1e9
                    by_n[n]["period_us"] = period
        except (ValueError, IndexError):
            continue
    return photos


def absolute(tow, ref_gps_s):
    """Time-of-week -> GPS seconds, choosing the week closest to ref (the trajectory)."""
    week = round((ref_gps_s - tow) / WEEK_S)
    return week * WEEK_S + tow


def interpolate(pos, times, t, max_gap):
    i = bisect.bisect_right(times, t)
    if i == 0 or i == len(times):
        return None, "outside the PPK trajectory"
    a, b = pos[i - 1], pos[i]
    if b[0] - a[0] > max_gap:
        return None, f"gap of {b[0] - a[0]:.1f} s in the trajectory"
    f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
    lerp = lambda k: a[k] + f * (b[k] - a[k])
    q = max(a[4], b[4])                         # the weaker of the two solutions (1 fixed < 2 float < 5 single)
    return (lerp(1), lerp(2), lerp(3), q, min(a[5], b[5]), max(a[6], b[6]), max(a[7], b[7]), max(a[8], b[8])), ""


def gps_to_utc(gps_s):
    return GPS_EPOCH + dt.timedelta(seconds=gps_s - LEAP_S)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", nargs="?", default=".", help="session folder (rover.ubx or RINEX + arduino.txt)")
    ap.add_argument("--rover", nargs="+", help="rover (camera u-blox) files instead of the session's: .ubx/RINEX/.crx/.gz/.zip")
    ap.add_argument("--events", help="shutter times file instead of SESSION/arduino.txt")
    ap.add_argument("--nav", nargs="+", help="extra RINEX navigation files (if the logs lack ephemerides)")
    ap.add_argument("--base", nargs="+", required=True, help="base files or folders: .ubx/.zip/RINEX/.crx/.gz")
    ap.add_argument("--base-pos", nargs=3, type=float, metavar=("LAT", "LON", "HEIGHT"))
    ap.add_argument("--base-pos-rinex", action="store_true")
    ap.add_argument("--photos")
    ap.add_argument("--write-exif", metavar="OUT_DIR")
    ap.add_argument("--antenna-above-camera", type=float, default=0.0)
    ap.add_argument("--out")
    a = ap.parse_args()
    if not a.base_pos and not a.base_pos_rinex:
        die("give --base-pos LAT LON HEIGHT (the base antenna's known position), or --base-pos-rinex")
    if a.write_exif and not a.photos:
        die("--write-exif needs --photos")
    for tool in (CONVBIN, RNX2RTKP):
        if not os.path.exists(tool):
            die(f"{tool} not found: run  python setup_tools.py  (or install RTKLIB demo5)")
    if a.photos and not os.path.exists(photomatch.EXIFTOOL):
        die(f"{photomatch.EXIFTOOL} not found: run  python setup_tools.py  (needed to read photo times)")
    if a.write_exif and a.photos:
        w, ph = os.path.normcase(os.path.abspath(a.write_exif)), os.path.normcase(os.path.abspath(a.photos))
        if w == ph or w.startswith(ph.rstrip(os.sep) + os.sep):
            die("--write-exif must be a folder OUTSIDE the photo folder (else the copies are read as photos next time)")
    ses = a.session
    if ses == "." and a.events:                        # no session folder: results go next to the shutter times
        ses = os.path.dirname(os.path.abspath(a.events))
    out = a.out or os.path.join(ses, "metashape.csv")
    try:                                               # fail now, not after minutes of PPK
        with open(out, "a", encoding="utf-8"):
            pass
    except PermissionError:
        die(f"cannot write {out}: close it in Excel (or any other program) and run again")
    a.out = out
    work = tempfile.mkdtemp(prefix="ppk_", dir=ascii_dir())
    for path in [ses, *(a.base or []), *(a.rover or []), a.events or ""]:
        check_ansi(path)
    try:
        _process(a, ses, work)
    finally:
        shutil.rmtree(work, ignore_errors=True)


def _process(a, ses, work):
    # 1. Inputs -> RINEX (rover = the u-blox on the camera; raw .ubx or RINEX in any naming)
    if a.rover:
        rover_in = a.rover
    else:
        ubx = os.path.join(ses, "rover.ubx")
        if os.path.exists(ubx):
            if time.time() - os.path.getmtime(ubx) < 10:
                die("rover.ubx is still being written (close the window / stop the logger first)")
            rover_in = [ubx]                           # the raw log is the master copy: always re-convert
        else:
            rover_in = [ses]                           # e.g. a session that only holds RINEX
    rover_obs, rover_navs = gather(rover_in, work, "rover")
    if not rover_obs:
        die("no rover observations found (expected rover.ubx or RINEX obs; or give --rover FILES)")
    base_obs, base_navs = gather(a.base, work, "base")
    if not base_obs:
        die("no base observations found in --base")
    navs = (a.nav or []) + rover_navs + base_navs
    if not navs:
        die("no navigation (ephemeris) file: give the RINEX nav with --nav")

    # 2. PPK
    approx = approx_llh(base_obs[0])
    if a.base_pos:
        lat, lon, h = a.base_pos
        if approx and distance_m(lat, lon, approx[0], approx[1]) > 500:   # header position is rough (~100 m)
            print(f"WARNING: --base-pos is {distance_m(lat, lon, approx[0], approx[1]) / 1000:.2f} km from the base "
                  f"file's own rough position ({approx[0]:.5f}, {approx[1]:.5f}). Check the base coordinates "
                  "(order is LAT LON HEIGHT); a wrong base position ruins or blocks the PPK.", file=sys.stderr)
        base_cfg = f"ant2-postype       =llh\nant2-pos1          ={lat:.9f}\nant2-pos2          ={lon:.9f}\nant2-pos3          ={h:.4f}"
    else:
        base_cfg = "ant2-postype       =rinexhead"
    conf = os.path.join(work, "ppk.conf")
    with open(conf, "w") as fh:                        # rtklibexplorer's F9P settings, then ours (last wins)
        if os.path.exists(F9P_CONF):
            fh.write(open(F9P_CONF).read() + "\n")
        fh.write(CONF.format(base=base_cfg))
    pos_file = os.path.join(ses, "ppk.pos")
    print("running PPK (forward + backward) ...", flush=True)
    run([RNX2RTKP, "-k", conf, "-o", pos_file, one_receiver(rover_obs, work, "rover"),
         one_receiver(base_obs, work, "base"), *navs])
    pos = read_pos(pos_file)
    if not pos:
        die("PPK produced no solution. Check that the rover and base logs overlap in time, and that the "
            "base position (--base-pos) is right.")
    times = [p[0] for p in pos]
    steps = sorted(b - a_ for a_, b in zip(times, times[1:]))
    rate = steps[len(steps) // 2] if steps else 1.0
    if len(pos) < 10:
        print(f"WARNING: only {len(pos)} PPK epochs were solved; check the base position and that the logs overlap.",
              file=sys.stderr)
    q_count = {}
    for p in pos:
        q_count[p[4]] = q_count.get(p[4], 0) + 1

    # 3. Shutter pulses: GPS time (absolute) and interpolated position
    events = a.events or os.path.join(ses, "arduino.txt")
    if not os.path.exists(events):
        die(f"{events}: shutter times not found (give --events FILE)")
    photos = read_photos(events)
    if not photos:
        die(f"no shutter pulses (MARK lines) in {events}")
    for p in photos:
        p["gps_s"] = absolute(p["tow"], times[0]) if p["tow"] is not None else None
        p["row"], p["note"] = (None, "no GPS time for this shutter pulse")
        if p["gps_s"] is not None:
            p["row"], p["note"] = interpolate(pos, times, p["gps_s"], max_gap=max(1.5 * rate, 0.5))
            if p["row"]:
                lat, lon, h, q, ns, sdn, sde, sdu = p["row"]
                floor = ACC_FLOOR_M.get(q, 2.0)        # RTKLIB prints 0 for a degenerate covariance:
                sdn, sde, sdu = (max(v, floor) for v in (sdn, sde, sdu))   # never tell Metashape "perfect"
                p["row"] = (lat, lon, h - a.antenna_above_camera, q, ns, sdn, sde, sdu)

    # 4. Pair image files with pulses (by capture time)
    shot_of, report, shots = {}, None, []
    if a.photos:
        shots = photomatch.read_shots(a.photos)
        pulse_utc = [gps_to_utc(p["gps_s"]).timestamp() if p["gps_s"] is not None else None for p in photos]
        shot_of, report = photomatch.match(shots, pulse_utc)

    # 5. CSV (one row per image file; pulses without an image get a placeholder label)
    out = a.out
    summary, exif_rows = {}, []
    with open(out, "w", newline="", encoding="utf-8") as f:
        f.write("label,longitude,latitude,height_ellipsoid_m,acc_east_m,acc_north_m,acc_height_m,"
                "solution,sats,gps_week,gps_tow_s,utc_time,note\n")

        def write(label, p, note):
            if p is None or p["row"] is None:
                f.write(f"{label},,,,,,,none,,,,,{note}\n")
                return
            lat, lon, h, q, ns, sdn, sde, sdu = p["row"]
            t = p["gps_s"]
            f.write(f"{label},{lon:.9f},{lat:.9f},{h:.4f},{sde:.4f},{sdn:.4f},{sdu:.4f},{Q_NAME.get(q, q)},{ns},"
                    f"{int(t // WEEK_S)},{t % WEEK_S:.6f},{gps_to_utc(t):%Y-%m-%dT%H:%M:%S.%fZ},{note}\n")

        for k, p in enumerate(photos):
            shot = shot_of.get(k)
            name = Q_NAME.get(p["row"][3], "?") if p["row"] else "missing"
            summary[name] = summary.get(name, 0) + 1
            if shot is None:
                notes = [p["note"]] if not p["row"] else []
                if a.photos:
                    notes.append("no image file for this pulse")
                write(f"pulse_{k + 1:04d}", p, "; ".join(notes))
                continue
            for file in shot.files:                    # JPG and ARW of the same shot get the same position
                write(os.path.basename(file), p, p["note"] if not p["row"] else "")
            if p["row"]:
                lat, lon, h, q, ns, sdn, sde, sdu = p["row"]
                exif_rows.append((shot, lat, lon, h, max(sdn, sde), gps_to_utc(p["gps_s"])))
        matched = set(id(s) for s in shot_of.values())
        for s in shots:
            if id(s) not in matched:
                for file in s.files:
                    write(os.path.basename(file), None, "image has no shutter pulse (not geotagged)")

    print(f"trajectory: {len(pos)} epochs, one every {rate:g} s, "
          + ", ".join(f"{Q_NAME.get(q, q)} {100 * c / len(pos):.0f}%" for q, c in sorted(q_count.items())))
    print(f"shutter pulses: {len(photos)} -> " + ", ".join(f"{k} {v}" for k, v in summary.items()))
    if report:
        print(f"photo matching ({report['method']}): {report['images']} images, {report.get('matched', len(shot_of))} paired")
        if "camera_minus_gps_s" in report:
            print(f"  camera clock = GPS {report['camera_minus_gps_s']:+.3f} s; pairing residual median "
                  f"{report['residual_median_s']} s, max {report['residual_max_s']} s (tolerance {report['tolerance_s']} s)")
        for key, what in (("images_without_pulse", "images with no shutter pulse"),
                          ("pulses_without_image", "shutter pulses with no image (pulse #)")):
            if report.get(key):
                items = report[key]
                print(f"  WARNING {len(items)} {what}: {', '.join(map(str, items[:15]))}{' ...' if len(items) > 15 else ''}")
    if a.write_exif and exif_rows:
        n = photomatch.write_gps_exif(exif_rows, a.write_exif, a.photos)
        print(f"wrote {n} geotagged image copies to {a.write_exif} (GPS altitude = ellipsoidal height)")
    print(f"wrote {out}")
    print("Metashape: Reference pane > Import Reference > this CSV; coordinate system WGS 84 (EPSG::4326); "
          "columns: Label=1, Longitude=2, Latitude=3, Altitude=4, Accuracy=5,6,7; start row 2. "
          "Heights are ellipsoidal.")


if __name__ == "__main__":
    main()
