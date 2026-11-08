#!/usr/bin/env python3
"""Simple window around ppk.py: pick the folders, type the base position, press Run.
Double-click on Windows (or: python ppk_gui.py). Remembers the last values in ppk_gui_settings.json."""
import json, os, queue, subprocess, sys, threading
import tkinter as tk
from tkinter import filedialog, messagebox

HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS = os.path.join(HERE, "ppk_gui_settings.json")
FIELDS = ["session", "base", "lat", "lon", "height", "photos", "antenna", "exif", "exif_dir"]


def default_exif_dir(photos):
    """Next to the photo folder, never inside it; for a drive root (the SD card E:\\) use Documents."""
    photos = os.path.normpath(photos)
    parent, name = os.path.dirname(photos), os.path.basename(photos)
    if not name or parent == photos:
        return os.path.join(os.path.expanduser("~"), "Documents", "geotagged_photos")
    return os.path.join(parent, name + "_geotagged")


class App:
    def __init__(self, root):
        self.root, self.q, self.proc = root, queue.Queue(), None
        root.title("PPK photo geotagging")
        self.v = {k: tk.StringVar() for k in FIELDS}
        self.v["antenna"].set("0")
        self.v["exif"] = tk.BooleanVar(value=True)
        self.load()
        f = tk.Frame(root, padx=12, pady=10)
        f.pack(fill="both", expand=True)
        r = 0

        def row(label, key, browse=None, hint=""):
            nonlocal r
            tk.Label(f, text=label, anchor="w").grid(row=r, column=0, sticky="w", pady=3)
            tk.Entry(f, textvariable=self.v[key], width=70).grid(row=r, column=1, sticky="we")
            if browse:
                tk.Button(f, text="Browse…", command=browse).grid(row=r, column=2, padx=4)
            if hint:
                tk.Label(f, text=hint, fg="#666", anchor="w").grid(row=r + 1, column=1, sticky="w")
                r += 1
            r += 1

        row("1. Flight session folder", "session", lambda: self.pick_dir("session"),
            "the folder from the logger: rover.ubx (or rover RINEX) + arduino.txt")
        row("2. Base station files", "base", self.pick_base,
            "base raw .ubx / .zip or RINEX (.obs/.rnx/.YYo, .crx, .gz); several files: separate with ;")
        tk.Label(f, text="3. Base antenna position", anchor="w").grid(row=r, column=0, sticky="w", pady=3)
        pos = tk.Frame(f)
        pos.grid(row=r, column=1, sticky="w")
        for key, lab, w in (("lat", "latitude °", 14), ("lon", "longitude °", 14), ("height", "ellipsoidal height m", 10)):
            tk.Label(pos, text=lab).pack(side="left")
            tk.Entry(pos, textvariable=self.v[key], width=w).pack(side="left", padx=(2, 10))
        r += 1
        tk.Label(f, text="WGS84, e.g. from the base's PPP result; every photo inherits any error in it",
                 fg="#666").grid(row=r, column=1, sticky="w")
        r += 1
        row("4. Photo folder (SD card)", "photos", lambda: self.pick_dir("photos"),
            "all images under it (JPG/ARW); paired with the shutter pulses by capture time")
        row("Antenna above camera (m)", "antenna", None, "vertical antenna-to-camera distance; 0 if unknown")
        tk.Checkbutton(f, text="Also write geotagged copies of the photos to:", variable=self.v["exif"]).grid(
            row=r, column=0, columnspan=2, sticky="w")
        r += 1
        row("", "exif_dir", lambda: self.pick_dir("exif_dir"), "originals are never changed")
        self.run_btn = tk.Button(f, text="Run PPK and match photos", command=self.run, bg="#1f9d55", fg="white",
                                 font=("Helvetica", 12, "bold"), padx=10, pady=4)
        self.run_btn.grid(row=r, column=0, columnspan=3, pady=8)
        r += 1
        self.log = tk.Text(f, height=18, width=110, font=("Courier", 9))
        self.log.grid(row=r, column=0, columnspan=3, sticky="nsew")
        f.rowconfigure(r, weight=1)
        f.columnconfigure(1, weight=1)
        root.after(100, self.pump)

    def pick_dir(self, key):
        d = filedialog.askdirectory(initialdir=self.v[key].get() or HERE)
        if d:
            self.v[key].set(d)
            if key == "photos" and not self.v["exif_dir"].get():
                self.v["exif_dir"].set(default_exif_dir(d))

    def pick_base(self):
        fs = filedialog.askopenfilenames(title="Base station files")
        if fs:
            self.v["base"].set(";".join(fs))

    def load(self):
        try:
            for k, val in json.load(open(SETTINGS)).items():
                if k in self.v:
                    self.v[k].set(val)
        except (OSError, ValueError):
            pass

    def save(self):
        try:
            json.dump({k: self.v[k].get() for k in FIELDS}, open(SETTINGS, "w"), indent=1)
        except OSError:
            pass

    def run(self):
        g = {k: self.v[k].get() for k in FIELDS}
        try:
            lat, lon, h, ant = float(g["lat"]), float(g["lon"]), float(g["height"]), float(g["antenna"] or 0)
        except ValueError:
            messagebox.showerror("Base position", "Enter the base latitude, longitude and height as numbers.")
            return
        if not os.path.isdir(g["session"]) or not g["base"].strip():
            messagebox.showerror("Missing input", "Choose the flight session folder and the base station files.")
            return
        cmd = [sys.executable, "-u", os.path.join(HERE, "ppk.py"), g["session"],
               "--base", *[b.strip() for b in g["base"].split(";") if b.strip()],
               "--base-pos", str(lat), str(lon), str(h), "--antenna-above-camera", str(ant)]
        if g["photos"]:
            cmd += ["--photos", g["photos"]]
            if g["exif"] and g["exif_dir"]:
                cmd += ["--write-exif", g["exif_dir"]]
        self.save()
        self.log.delete("1.0", "end")
        self.run_btn.config(state="disabled", text="Running…")
        threading.Thread(target=self.worker, args=(cmd,), daemon=True).start()

    def worker(self, cmd):
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                 encoding="utf-8", errors="replace", env={**os.environ, "PYTHONIOENCODING": "utf-8"},
                                 creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            for line in p.stdout:
                self.q.put(line)
            p.wait()
            self.q.put(("done", p.returncode))
        except Exception as e:
            self.q.put(f"ERROR: {e}\n")
            self.q.put(("done", 1))

    def pump(self):
        try:
            while True:
                item = self.q.get_nowait()
                if isinstance(item, tuple):
                    self.run_btn.config(state="normal", text="Run PPK and match photos")
                    self.log.insert("end", "\n" + ("FINISHED: import the metashape.csv shown above into Metashape.\n"
                                                    if item[1] == 0 else "FAILED: see the message above.\n"))
                else:
                    self.log.insert("end", item)
                self.log.see("end")
        except queue.Empty:
            pass
        self.root.after(100, self.pump)


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
