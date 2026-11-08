#!/usr/bin/env python3
"""Pair image files with shutter pulses by TIME, not just by order.

The camera writes each photo's capture time (EXIF DateTimeOriginal + SubSecTimeOriginal) from
its own clock; the Arduino gives each shutter pulse an exact GPS time. The camera clock is off by
an unknown amount (time zone, drift, never set), so:

  1. offset: the camera-minus-GPS offset is the most common difference between image times and
     pulse times (a few real pairs are enough; checked against every pulse).
  2. alignment: a monotonic alignment (like aligning two sequences) pairs image i with pulse k only
     when (image time - offset) is within `tol` of the pulse time. An image with no pulse (e.g. a test
     shot while nothing was recording) or a pulse with no image (deleted file, card full) is skipped
     on its own; it never shifts the photos after it, which matching by order would do.

Used by ppk.py. Standard library + exiftool (reads JPG and Sony ARW alike).
"""
import collections, datetime as dt, json, os, shutil, subprocess, tempfile

EXIFTOOL = shutil.which("exiftool") or os.path.expanduser("~/.local/bin/exiftool")
IMAGE_EXT = (".jpg", ".jpeg", ".arw", ".dng", ".tif", ".tiff", ".heic", ".hif")


class Shot:
    """One exposure; may be several files (e.g. DSC00012.JPG + DSC00012.ARW)."""

    def __init__(self, files, t, subsec):
        self.files, self.t, self.subsec = files, t, subsec   # t: camera-clock seconds (naive), or None

    @property
    def label(self):
        jpg = [f for f in self.files if f.lower().endswith((".jpg", ".jpeg"))]
        return os.path.basename((jpg or self.files)[0])


def _parse_exif_time(value):
    """'2026:11:08 12:34:56.789' (with optional +02:00) -> seconds since 1970 on the camera's clock."""
    if not value or not isinstance(value, str):
        return None
    v = value.strip()
    for cut in ("+", "-", "Z"):
        i = v.rfind(cut)
        if i > 16:                       # strip a trailing time-zone suffix; the offset absorbs it anyway
            v = v[:i]
    try:
        base, _, frac = v.partition(".")
        t = dt.datetime.strptime(base, "%Y:%m:%d %H:%M:%S").replace(tzinfo=dt.timezone.utc).timestamp()
        return t + (float("0." + frac) if frac.isdigit() else 0.0)
    except ValueError:
        return None


def read_shots(folder):
    """All images under folder (recursively), grouped into shots, sorted by capture time."""
    files = [os.path.join(r, f) for r, _, fs in os.walk(folder) for f in fs if f.lower().endswith(IMAGE_EXT)]
    if not files:
        return []
    times = {}
    if os.path.exists(EXIFTOOL):
        # exiftool scans the folder itself: a list of 1000 paths would exceed Windows' command-line limit
        exts = [x for e in IMAGE_EXT for x in ("-ext", e[1:])]
        # arguments go through a UTF-8 argfile: on Windows the command line is in the ANSI code page, which
        # would garble non-ASCII folder names (e.g. D:\\Fotos\\Zürich) under -charset filename=utf8
        argfile = os.path.join(tempfile.gettempdir(), f"photomatch_{os.getpid()}.args")
        with open(argfile, "w", encoding="utf-8") as fh:
            fh.write("\n".join(["-j", "-q", "-r", *exts, "-DateTimeOriginal", "-SubSecTimeOriginal",
                                "-SubSecDateTimeOriginal", folder]) + "\n")
        try:
            r = subprocess.run([EXIFTOOL, "-charset", "filename=utf8", "-@", argfile],
                               capture_output=True, text=True, encoding="utf-8", errors="replace")
        finally:
            os.remove(argfile)
        if not r.stdout.strip():
            raise RuntimeError(f"exiftool could not read capture times from {folder}: {r.stderr.strip()[-500:]}")
        try:
            for e in json.loads(r.stdout or "[]"):
                full = e.get("SubSecDateTimeOriginal")
                t = _parse_exif_time(full) or _parse_exif_time(e.get("DateTimeOriginal"))
                if t is not None and not full and str(e.get("SubSecTimeOriginal", "")).strip().isdigit():
                    t += float("0." + str(e["SubSecTimeOriginal"]).strip())
                    full = True
                times[os.path.normcase(os.path.abspath(e["SourceFile"]))] = (t, bool(full))   # '/' vs '\\'
        except ValueError:
            pass
    groups = collections.defaultdict(list)
    for f in files:
        groups[os.path.splitext(os.path.abspath(f))[0]].append(os.path.abspath(f))
    shots = []
    for stem, fs in groups.items():
        key = lambda f: os.path.normcase(f)
        known = [times[key(f)] for f in fs if key(f) in times and times[key(f)][0] is not None]
        t, sub = (min(known) if known else (None, False))
        shots.append(Shot(sorted(fs), t, sub))
    shots.sort(key=lambda s: (s.t is None, s.t if s.t is not None else 0, s.label))
    return shots


def estimate_offset(img_t, pulse_t, tol):
    """Camera clock minus GPS time (s): the offset that pairs the most images with pulses."""
    if not img_t or not pulse_t:
        return None, 0
    step = max(len(img_t) // 200, 1)
    votes = collections.Counter()
    for t in img_t[::step]:                       # candidate offsets from a sample of images vs all pulses
        for p in pulse_t:
            votes[round((t - p) / tol)] += 1
    best, best_n = None, -1
    for bucket, _ in votes.most_common(8):
        for cand in (bucket * tol, (bucket - 0.5) * tol, (bucket + 0.5) * tol):
            n = _count_matches(img_t, pulse_t, cand, tol)
            if n > best_n:
                best, best_n = cand, n
    # refine: median residual of the matched pairs
    res = [r for r in _residuals(img_t, pulse_t, best, tol)]
    if res:
        res.sort()
        best += res[len(res) // 2]
        best_n = _count_matches(img_t, pulse_t, best, tol)
    return best, best_n


def _residuals(img_t, pulse_t, off, tol):
    j = 0
    for t in img_t:
        x = t - off
        while j + 1 < len(pulse_t) and pulse_t[j + 1] <= x:
            j += 1
        cands = [pulse_t[k] for k in (j, j + 1) if 0 <= k < len(pulse_t)]
        d = min((x - p for p in cands), key=abs, default=None)
        if d is not None and abs(d) < tol:
            yield d


def _count_matches(img_t, pulse_t, off, tol):
    return sum(1 for _ in _residuals(img_t, pulse_t, off, tol))


def align(img_t, pulse_t, off, tol):
    """Monotonic alignment maximising pairs (tie-break: smaller residuals). Returns [(i, k)].
    Full dynamic programme: O(images x pulses), fine for a few thousand photos per flight."""
    n, m = len(img_t), len(pulse_t)
    NEG = float("-inf")
    prev = [0.0] * (m + 1)
    back = []                                     # per row: bytes, 0 = skip image, 1 = skip pulse, 2 = match
    for i in range(1, n + 1):
        cur = [0.0] * (m + 1)
        row = bytearray(m + 1)
        x = img_t[i - 1] - off
        for k in range(1, m + 1):
            best, how = prev[k], 0                # image i has no pulse
            if cur[k - 1] > best:
                best, how = cur[k - 1], 1         # pulse k has no image
            r = abs(x - pulse_t[k - 1])
            if r < tol:
                s = prev[k - 1] + 1.0 - 0.5 * (r / tol) ** 2
                if s > best:
                    best, how = s, 2
            cur[k], row[k] = best, how
        back.append(row)
        prev = cur
    pairs, i, k = [], n, m
    while i > 0 and k > 0:
        how = back[i - 1][k]
        if how == 2:
            pairs.append((i - 1, k - 1))
            i, k = i - 1, k - 1
        elif how == 1:
            k -= 1
        else:
            i -= 1
    return pairs[::-1]


def match(shots, pulse_times):
    """shots: from read_shots; pulse_times: GPS UTC seconds (None = unknown) in shutter order.
    Returns dict: pulse index -> shot, plus a report. Falls back to order when images have no times."""
    timed = [(i, s) for i, s in enumerate(shots) if s.t is not None]
    pulses = [(k, t) for k, t in enumerate(pulse_times) if t is not None]
    report = {"images": len(shots), "pulses": len(pulse_times), "method": "time"}
    if len(timed) < 3 or len(pulses) < 3:
        report["method"] = "order (images have no EXIF capture time)" if len(timed) < 3 else "order (too few timed pulses)"
        return {k: shots[k] for k in range(min(len(shots), len(pulse_times)))}, report
    subsec = sum(s.subsec for _, s in timed) > len(timed) / 2
    tol = 0.35 if subsec else 1.1                 # EXIF without sub-seconds is truncated to whole seconds
    img_t = [s.t for _, s in timed]
    pul_t = [t for _, t in pulses]
    off, n = estimate_offset(img_t, pul_t, tol)
    pairs = align(img_t, pul_t, off, tol)
    out = {pulses[k][0]: timed[i][1] for i, k in pairs}
    res = sorted(abs(img_t[i] - off - pul_t[k]) for i, k in pairs)
    report.update(camera_minus_gps_s=round(off, 3), tolerance_s=tol, matched=len(pairs),
                  residual_median_s=round(res[len(res) // 2], 3) if res else None,
                  residual_max_s=round(res[-1], 3) if res else None,
                  images_without_pulse=[s.label for s in shots if s not in out.values()],
                  pulses_without_image=[k + 1 for k in range(len(pulse_times)) if k not in out])
    return out, report


def write_gps_exif(rows, out_dir, src_root):
    """Geotagged COPIES of the images (originals untouched). rows: (shot, lat, lon, h_ellipsoid,
    hacc_m, utc datetime). One exiftool run for all files."""
    os.makedirs(out_dir, exist_ok=True)
    args = []
    for shot, lat, lon, h, hacc, utc in rows:
        for f in shot.files:
            dest = os.path.join(out_dir, os.path.relpath(f, src_root))
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            if os.path.exists(dest):
                os.remove(dest)
            args += ["-o", dest,
                     f"-GPSLatitude={abs(lat):.9f}", f"-GPSLatitudeRef={'N' if lat >= 0 else 'S'}",
                     f"-GPSLongitude={abs(lon):.9f}", f"-GPSLongitudeRef={'E' if lon >= 0 else 'W'}",
                     f"-GPSAltitude={abs(h):.3f}", f"-GPSAltitudeRef={0 if h >= 0 else 1}",
                     f"-GPSHPositioningError={hacc:.3f}", "-GPSMapDatum=WGS-84", "-GPSStatus=A",
                     f"-GPSDateStamp={utc:%Y:%m:%d}", f"-GPSTimeStamp={utc:%H:%M:%S.%f}",
                     "-GPSMeasureMode=3", f, "-execute"]
    if not args:
        return 0
    argfile = os.path.join(out_dir, ".exiftool_args")
    with open(argfile, "w", encoding="utf-8") as fh:
        fh.write("\n".join(args) + "\n")
    r = subprocess.run([EXIFTOOL, "-charset", "filename=utf8", "-q", "-q", "-@", argfile],
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    os.remove(argfile)
    if r.returncode != 0:
        raise RuntimeError(f"exiftool failed: {r.stderr[-1500:]}")
    return sum(len(s.files) for s, *_ in rows)
