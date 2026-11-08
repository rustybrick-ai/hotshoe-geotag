# Wiring

Arduino Mega 2560 + ArduSimple simpleRTK2B Budget (u-blox ZED-F9P) + Sony hot shoe.
No resistors are needed with this board.

## ArduSimple simpleRTK2B → Arduino Mega

| ArduSimple pin | → Mega pin | What it does |
|---|---|---|
| **GND** | **GND** | common ground (essential) |
| **IOREF** | **IOREF** or **5V** | sets the level of TX1/RX1 to 5 V. Not power; without it TX1/RX1 stay silent |
| **TX1** | **19** (printed "RX1 19") | u-blox data → Arduino (positions, to label the seconds) |
| **RX1** | **18** (printed "TX1 18") | Arduino → u-blox configuration |
| **Timepulse** (special function pins) | **D3** | 1 pulse per second, exactly at each GPS second (3.3 V) |

Do **not** use Mega D0/D1: they are the Mega's USB link to the computer.

## Hot shoe → Arduino Mega

| Hot shoe | → Mega pin | What it does |
|---|---|---|
| centre pin | **D2** | shorted to ground at the instant of exposure (~9.5 ms on our Sony) |
| side rail | **GND** | ground |

D2 uses the Arduino's internal pull-up (idle high, low at exposure).

## Power and USB

| Cable | Notes |
|---|---|
| Mega USB-B → computer / Pi | power + photo times |
| ArduSimple USB-C (POWER+GPS port) → computer / Pi | power + raw satellite data for PPK; **use a data cable**. Error -71 in the Linux kernel log means a bad data connection; a USB hub fixed it for us |
| *(do not)* ArduSimple 5V_IN → Mega 5V | only when the ArduSimple has no USB; never both power sources at once |

Keep the ArduSimple's "5V=OUTPUT" switch (if fitted) off. The antenna needs open sky.

## Checks

| Indicator | Means |
|---|---|
| ArduSimple POWER LED on | powered |
| shutter_gui: "GPS: 3D, N satellites" | TX1/RX1/IOREF/GND good |
| ArduSimple GPS FIX LED blinks 1×/s, shutter_gui: "satellite pulse OK" | fix + Timepulse→D3 good |
| shutter_gui: "PPK raw data: RECORDING" | ArduSimple USB good |
| window flashes on a photo, then shows its GPS time | hot shoe good; whole chain working |

## Camera settings (Sony)

The hot shoe only pulses when the camera thinks a flash is in use: flash mode **Fill flash**, **mechanical
shutter** (the electronic shutter gives no sync pulse), and test at the shutter speed you will fly;
some bodies stop the sync signal above the flash sync speed.
