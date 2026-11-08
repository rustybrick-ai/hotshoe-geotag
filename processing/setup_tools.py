#!/usr/bin/env python3
"""Download the two tools the PPK processing needs into ./tools (next to this script):

  RTKLIB (rtklibexplorer "RTKLIB_EX" 2.5.1): convbin, rnx2rtkp, crx2rnx  -- raw -> RINEX 3, PPK
  ExifTool 13.61                                                       -- photo capture times, EXIF GPS

Windows: ready-made .exe files.  Linux/Mac: RTKLIB is built from source (needs gcc/clang + make;
on Mac: `xcode-select --install`), ExifTool is the Perl version (Perl is preinstalled).
Every download is checked against a fixed SHA-256 before use. Run once:

  python setup_tools.py
"""
import hashlib, io, os, platform, shutil, subprocess, sys, tarfile, urllib.request, zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(HERE, "tools")

RTKLIB_WIN = ("https://github.com/rtklibexplorer/RTKLIB/releases/download/v2.5.1/RTKLIB_EX_2.5.1.zip",
              "0b20c940b1939b0701f0ee027daf5225221120dbff50c3f563f8705fd0b496f5")
RTKLIB_SRC = ("https://github.com/rtklibexplorer/RTKLIB/archive/refs/tags/v2.5.1.tar.gz",
              "ad8e56f64dd2b71e2eb8098c27925a65b027b161eae366cf5afb71402348d2e1")
EXIFTOOL_WIN = ("https://sourceforge.net/projects/exiftool/files/exiftool-13.61_64.zip/download",
                "ab75b37515385a753aa60a58f68ffffe1ff81e43b319df9540978fb19e0580e5")
EXIFTOOL_PERL = ("https://sourceforge.net/projects/exiftool/files/Image-ExifTool-13.61.tar.gz/download",
                 "df7f42c14b4f58a60bec39feae0ee9593000ee27e44920d2b0a21c6627de0baa")


def fetch(url, sha):
    print(f"downloading {url.split('/')[-2] if url.endswith('/download') else url.split('/')[-1]} ...", flush=True)
    req = urllib.request.Request(url, headers={"User-Agent": "hotshoe-geotag-setup"})
    with urllib.request.urlopen(req, timeout=120) as r:
        data = r.read()
    got = hashlib.sha256(data).hexdigest()
    if got != sha:
        sys.exit(f"checksum mismatch for {url}\n  expected {sha}\n  got      {got}\nnot installing it.")
    return data


def windows():
    z = zipfile.ZipFile(io.BytesIO(fetch(*RTKLIB_WIN)))
    for exe in ("convbin.exe", "rnx2rtkp.exe", "crx2rnx.exe"):
        member = next(m for m in z.namelist() if m.lower().endswith("/" + exe))
        with z.open(member) as src, open(os.path.join(TOOLS, exe), "wb") as out:
            shutil.copyfileobj(src, out)
    z = zipfile.ZipFile(io.BytesIO(fetch(*EXIFTOOL_WIN)))
    for m in z.namelist():
        rel = m.split("/", 1)[1] if "/" in m else m
        if not rel or m.endswith("/"):
            continue
        dest = os.path.join(TOOLS, "exiftool.exe" if rel == "exiftool(-k).exe" else rel)  # no "press a key" pause
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        with z.open(m) as src, open(dest, "wb") as out:
            shutil.copyfileobj(src, out)


def unix():
    if not shutil.which("make") or not (shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")):
        sys.exit("a C compiler and make are needed (Linux: apt install build-essential; Mac: xcode-select --install)")
    src = os.path.join(TOOLS, "src")
    shutil.rmtree(src, ignore_errors=True)
    tarfile.open(fileobj=io.BytesIO(fetch(*RTKLIB_SRC))).extractall(src)
    root = os.path.join(src, os.listdir(src)[0])
    for app in ("convbin", "rnx2rtkp"):
        d = os.path.join(root, "app", "consapp", app, "gcc")
        print(f"building {app} ...", flush=True)
        r = subprocess.run(["make", "-j4", "LDLIBS=-lm"], cwd=d, capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit(f"building {app} failed:\n{r.stderr[-1500:]}")
        shutil.copy(os.path.join(d, app), os.path.join(TOOLS, app))
    et = os.path.join(TOOLS, "Image-ExifTool")
    shutil.rmtree(et, ignore_errors=True)
    tarfile.open(fileobj=io.BytesIO(fetch(*EXIFTOOL_PERL))).extractall(TOOLS)
    os.rename(next(os.path.join(TOOLS, d) for d in os.listdir(TOOLS) if d.startswith("Image-ExifTool-")), et)
    link = os.path.join(TOOLS, "exiftool")
    with open(link, "w") as fh:                         # small wrapper, so ppk.py finds tools/exiftool
        fh.write(f'#!/bin/sh\nexec perl "{et}/exiftool" "$@"\n')
    os.chmod(link, 0o755)
    shutil.rmtree(src, ignore_errors=True)


def main():
    os.makedirs(TOOLS, exist_ok=True)
    (windows if platform.system() == "Windows" else unix)()
    exe = ".exe" if platform.system() == "Windows" else ""
    missing = [t for t in ("convbin", "rnx2rtkp", "exiftool") if not os.path.exists(os.path.join(TOOLS, t + exe))]
    if missing:
        sys.exit(f"setup incomplete, missing: {missing}")
    print(f"tools ready in {TOOLS}")


if __name__ == "__main__":
    main()
