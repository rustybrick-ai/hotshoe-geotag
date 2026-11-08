# Processing: PPK + photo matching → Metashape (Windows guide)

You get three things from a flight:

| What | Where it comes from | Example |
|---|---|---|
| **Flight session folder** | the logger (laptop window or Raspberry Pi) | `001_20261108_112032\` with `rover.ubx` (or rover RINEX) + `arduino.txt` |
| **Base station files** | your base (e.g. RTKBase), covering the flight time | `2026-11-08_12-00-00_GNSS-1.ubx.zip`, or RINEX `.obs` / `.rnx` / `.26o` / `.crx` / `.gz` |
| **Photo folder** | the camera's SD card | `DCIM\100MSDCF\DSC04321.JPG …` (JPG and/or ARW) |

plus the **base antenna's exact position** (latitude, longitude, ellipsoidal height, WGS84).

The tool turns them into `metashape.csv` (one line per photo, with position and accuracy) and,
optionally, geotagged copies of the photos.

## One-time setup

1. Install **Python 3** from <https://www.python.org/downloads/> . In the installer, tick
   **"Add python.exe to PATH"**.
2. Download this repository: green **Code** button → **Download ZIP**, and unzip it, e.g. to `C:\hotshoe-geotag`.
3. Open the `processing` folder, click the address bar, type `cmd`, press Enter, and run:
   ```
   python setup_tools.py
   ```
   This downloads RTKLIB (rtklibexplorer 2.5.1: `convbin`, `rnx2rtkp`, `crx2rnx`) and ExifTool 13.61
   into `processing\tools\` (each download is checked against a fixed SHA-256). About 40 MB, once.

## Each flight

**Double-click `ppk_gui.py`** (or run `python ppk_gui.py`), then:

1. **Flight session folder**: Browse to the session folder.
2. **Base station files**: Browse and select all base files that cover the flight (hourly files are fine; they are joined).
3. **Base antenna position**: latitude, longitude (decimal degrees) and **ellipsoidal** height (m).
4. **Photo folder**: the SD card's folder (sub-folders are included).
5. Leave **"Also write geotagged copies"** ticked if you want photos with GPS in their EXIF (written to a new folder; originals are never changed).
6. Press **Run PPK and match photos** and read the summary at the bottom.

The same from the command line:
```
python ppk.py D:\flights\001_20261108_112032 --base D:\base\*.zip --base-pos 46.5012345 7.5023456 612.34 --photos E:\DCIM --write-exif D:\geotagged
```
Other options: `--rover FILES` (rover RINEX/raw from elsewhere instead of the session's), `--events FILE`
(shutter times file), `--nav FILES` (extra ephemeris), `--antenna-above-camera METRES`, `--out CSV`,
`--base-pos-rinex` (use the base RINEX header position, only if it is a surveyed one). `python ppk.py -h` for all.

## Reading the summary

```
trajectory: 3546 epochs, one every 0.2 s, fixed 94%, float 6%
shutter pulses: 212 -> fixed 205, float 7
photo matching (time): 213 images, 212 paired
  camera clock = GPS -3.412 s; pairing residual median 0.021 s, max 0.090 s (tolerance 0.35 s)
  WARNING 1 images with no shutter pulse: DSC04327.JPG
```

* **fixed** = centimetre solution; **float** = decimetre-ish; **single/dgps** = metres. Aim for >90 % fixed
  (open sky at rover and base, base close by, base position correct).
* **Photo matching** pairs each image with its shutter pulse by **capture time**: the camera clock
  offset is found automatically, so a missing or extra photo is reported and skipped instead of
  shifting every later photo. Check any WARNING lines: an image "with no shutter pulse" gets no
  coordinates (e.g. a test shot on the ground while nothing was recording); a pulse "with no image"
  means a photo file is missing.
* `WARNING: --base-pos is … km from the base file's own rough position` almost always means a typo in
  the base coordinates (or LAT/LON swapped).

## Into Metashape

**Reference** pane → **Import Reference** → `metashape.csv`:

* Coordinate system: **WGS 84 (EPSG::4326)**
* Columns: **Label 1, Longitude 2, Latitude 3, Altitude 4, Accuracy 5 6 7** (east, north, height)
* Start import at row **2**, delimiter comma.

Heights are **ellipsoidal** (WGS84). To get heights above sea level, set the coordinate system in
Metashape to WGS 84 + your geoid model after import. Rows with empty coordinates (photos without
a shutter pulse) are imported as unreferenced cameras.

The `--antenna-above-camera` option subtracts a vertical antenna-to-camera offset (assumes a level
drone). Horizontal lever-arm and camera orientation are not applied yet.

## What happens inside

1. Rover and base raw `.ubx` → RINEX 3.04 with RTKLIB `convbin` (RINEX inputs are used as they are,
   recognised by content, whatever their name; `.crx` and `.gz` are decompressed).
2. `rnx2rtkp` (kinematic, forward + backward, rtklibexplorer's `configs/f9p_ppk.conf` for the
   u-blox ZED-F9P) computes the trajectory at 5 Hz → `ppk.pos` in the session folder.
3. Each shutter pulse's exact GPS time (from `arduino.txt`, measured by the Arduino against the
   u-blox's 1-pulse-per-second signal) is placed on that trajectory by linear interpolation between
   the surrounding epochs.
4. Images are paired with pulses by EXIF capture time (ExifTool), then written to the CSV
   (and EXIF GPS tags of the copies).
