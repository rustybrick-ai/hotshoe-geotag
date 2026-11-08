# hotshoe-geotag

Centimetre-level geotagging of photos from a Sony Alpha camera using a u-blox ZED-F9P GNSS receiver
and PPK (post-processed kinematic), ready for Agisoft Metashape.

The camera's hot shoe closes a contact at the instant of exposure. An Arduino timestamps that
pulse against the u-blox's 1-pulse-per-second (PPS) signal, giving each photo an exact GPS time
(~10 µs). The u-blox's raw satellite data is logged at 5 Hz. Afterwards, PPK against a base station
gives a centimetre trajectory, the position at each exposure is interpolated, and each image file
is paired with its exposure by capture time.

```
 Sony hot shoe ──D2──┐
                     Arduino Mega ──USB──┐                 (photo GPS times: arduino.txt)
 u-blox PPS ─────D3──┘   ▲ UART1 (pins 18/19)              │
                         │                                 ├──► laptop window or Raspberry Pi
 u-blox ZED-F9P ─────────┘                                 │     = one session folder
 (ArduSimple simpleRTK2B) ──────────USB────────────────────┘    (raw 5 Hz data: rover.ubx)

 session folder + base station files + photo folder ──► processing/ ──► metashape.csv
                                                                       (+ geotagged photo copies)
```

| Folder | What | Who uses it |
|---|---|---|
| [`processing/`](processing/) | PPK + photo matching → Metashape, with a simple window. **Start here if you process flights.** [Windows guide](processing/README.md) | whoever processes the photos |
| [`arduino/hotshoe_logger/`](arduino/hotshoe_logger/) | Arduino Mega sketch: hot shoe + PPS timing, u-blox setup | the rig |
| [`logger/`](logger/) | recording: `shutter_gui.py` (laptop: live view + recording), `log_shutter.py` (laptop, no window), `pi/` (Raspberry Pi field logger, systemd service) | the rig |
| [`docs/wiring.md`](docs/wiring.md) | all wiring | the rig |

## Recording a flight

1. Wire as in [docs/wiring.md](docs/wiring.md) and upload the sketch:
   `arduino-cli compile --upload -p <port> --fqbn arduino:avr:mega arduino/hotshoe_logger`
2. Plug the Arduino (USB-B) and the ArduSimple (USB-C, a data cable; a USB hub helped on our laptop) into the laptop.
3. `python3 logger/shutter_gui.py`: the window flashes at every photo and shows its GPS time and
   position. Its bottom lines must read **"satellite pulse OK"** and **"PPK raw data: RECORDING"**.
   Everything goes to `flights/NNN_<UTC date>/` (`arduino.txt`, `rover.ubx`).
4. Without a laptop: the Raspberry Pi runs `logger/pi/pi_logger.py` as a service and records the same
   session at boot, and also converts it to RINEX 3 (see the header of `pi_logger.py`).
5. Make sure the base station logs raw data covering the flight.

The u-blox is configured by the Arduino and the logger in RAM only, at every start; nothing is
saved to its flash.

## Processing

See **[processing/README.md](processing/README.md)**: one-time `python setup_tools.py`, then
`ppk_gui.py` (or `ppk.py`) → `metashape.csv`.

## Notes and limits

* Accuracy depends on the PPK: open sky at rover and base, base within ~10–20 km, and the base
  antenna position known accurately (e.g. 24 h PPP). Any error in the base position shifts every photo.
* Heights are ellipsoidal (WGS84). The vertical antenna-to-camera offset can be applied
  (`--antenna-above-camera`); a horizontal lever arm and camera attitude are not applied yet.
* Timing: the ArduSimple simpleRTK2B Budget does not expose the F9P's EXTINT input, so the Arduino
  measures the hot shoe pulse against PPS (~10 µs). A board with EXTINT (u-blox TIM-TM2) would remove
  the Arduino.
* Tools: RTKLIB by T. Takasu with rtklibexplorer's fork (BSD 2-clause; `processing/configs/f9p_ppk.conf`
  is rtklibexplorer's F9P configuration), ExifTool by Phil Harvey. Both are downloaded by
  `setup_tools.py`, not included here.
