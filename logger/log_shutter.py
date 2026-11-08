#!/usr/bin/env python3
"""Headless recording on the laptop, without the window: the same program the field Pi runs.
Records the Arduino's photo times (arduino.txt) and the u-blox raw data (rover.ubx) into
flights/NNN_<UTC>/, ready for ppk.py. Ctrl+C to stop. Only one program can use the Arduino at a
time, so don't run this and shutter_gui.py together (the window records the same session data).
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "pi"))
import pi_logger

if __name__ == "__main__":
    try:
        pi_logger.run()
    except KeyboardInterrupt:
        print("\nstopped")
