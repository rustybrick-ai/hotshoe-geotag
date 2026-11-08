// Hot shoe photo logger with GPS time from the u-blox PPS — Arduino Mega 2560
//
// Wiring (all grounds joined: hot shoe rail, Arduino GND, F9P GND):
//   hot shoe centre pin -> D2   (Arduino's internal pull-up; the shutter pulls it LOW at exposure)
//   F9P PPS / TIMEPULSE -> D3   (rising edge exactly at the start of each GPS second)
//   F9P UART1 TX        -> pin 19 (RX1)
//   F9P UART1 RX        <- pin 18 (TX1)
//   ArduSimple simpleRTK2B: use its TX1/RX1 header pins and tie its IOREF to the Mega's 5V
//   (IOREF sets the level of TX1/RX1; with nothing on it they stay silent). No resistors.
//   A bare 3.3 V F9P UART instead needs a 1k series / 2k to GND divider on the RX wire.
//   Do NOT use D0/D1: they are the USB serial port to the PC.
//
// How the GPS time of a photo is found:
//   Both the shutter edge (D2) and every PPS edge (D3) are timestamped by interrupts on the
//   same Arduino clock (micros). Each PPS is labelled with its GPS second from the NAV-PVT
//   message for that whole second (which always arrives just after that pulse). A photo taken
//   between the pulses for seconds S and S+1 happened at
//       S + (t_photo - t_pps_S) / (t_pps_S+1 - t_pps_S)
//   Dividing by the measured pulse spacing cancels the Arduino's own clock error (up to 0.5 %),
//   so the result is limited only by interrupt jitter (~10 us).
//
// The F9P is configured over UART1 at every boot, in RAM only (nothing saved to its flash):
// UBX out, NMEA off, NAV-PVT at 5 Hz, PPS: 1 pulse/s aligned to GPS seconds, only once locked.
//
// Serial (115200) to the PC, one line per event:
//   MARK,<n>,<t_fall_us>,<width_us>           photo n detected (immediately)
//   PHOTO,<n>,<gps_sec>,<frac_ns>,<period_us>  its GPS time-of-week = gps_sec + frac_ns/1e9
//                                              (gps_sec -1 = no GPS time: no PPS around it);
//                                              period_us = Arduino micros between the 2 pulses
//   PPS,<t_us>,<gps_sec>                       a labelled GPS second pulse
//   PVT,<itow_ms>,<fix>,<carr>,<sv>,<lat_e7>,<lon_e7>,<hmsl_mm>,<hacc_mm>   5 Hz position
//   ALIVE,<t_us>,<pin>,<count>,<f9p_ok>,<pps_ok>                          every 5 s
//   # ...                                      status / comments

#include "ubx.h"

const uint8_t PIN_SHOE = 2;                  // INT4 on the Mega
const uint8_t PIN_PPS = 3;                   // INT5 on the Mega
const uint32_t LOCKOUT_US = 20000;           // ignore contact bounce for 20 ms after an edge
const uint32_t PHOTO_TIMEOUT_US = 3000000;   // give up on GPS time for a photo after 3 s
// F9P UART1 baud search order (38400 = factory). Faster rates are not accurate on a 16 MHz AVR.
const uint32_t BAUDS[] = {38400, 115200, 9600};

// ---------- hot shoe interrupt ----------
struct Mark { uint32_t n, t_fall, width; };
const uint8_t QLEN = 32;                     // ring buffer, ISR -> loop
volatile Mark q[QLEN];
volatile uint8_t q_head = 0, q_tail = 0;
volatile uint32_t count = 0, overflows = 0;
volatile uint32_t last_fall = 0;
volatile bool low_active = false;
volatile uint8_t open_slot = 0;              // slot whose width is still being measured

void onShoe() {
  uint32_t now = micros();
  bool low = digitalRead(PIN_SHOE) == LOW;
  if (low) {
    if (count && now - last_fall < LOCKOUT_US) { low_active = true; return; }  // bounce of the same press
    uint8_t next = (q_head + 1) % QLEN;
    if (next == q_tail) { overflows++; return; }
    count++;
    last_fall = now;
    q[q_head].n = count;
    q[q_head].t_fall = now;
    q[q_head].width = 0;
    open_slot = q_head;
    low_active = true;
    q_head = next;
  } else {
    // Every rise ends the low state (so a glitch can never block the next photo); the last rise
    // within 100 ms of the fall sets the width, so contact bounce doesn't shorten it.
    if (count && now - last_fall < 100000UL) q[open_slot].width = now - last_fall;
    low_active = false;
  }
}

// ---------- PPS interrupt ----------
volatile uint32_t pps_t = 0;                 // micros() of the latest accepted PPS rising edge
volatile bool pps_new = false, pps_any = false;

void onPps() {
  uint32_t now = micros();
  if (pps_any && now - pps_t < 900000UL) return;   // glitch/ringing: real pulses are 1 s apart
  pps_t = now;
  pps_any = pps_new = true;
}

// Recent PPS edges, oldest first; sec = GPS second of week (-1 until labelled by NAV-PVT).
struct Pps { uint32_t t; int32_t sec; };
const uint8_t PPS_KEEP = 6;
Pps pps[PPS_KEEP];
uint8_t pps_n = 0;
uint32_t last_pps_ms = 0;

void addPps(uint32_t t) {
  if (pps_n == PPS_KEEP) { memmove(pps, pps + 1, sizeof(Pps) * (PPS_KEEP - 1)); pps_n--; }
  pps[pps_n++] = {t, -1};
  last_pps_ms = millis();
}

// ---------- photos waiting for the next PPS ----------
const uint8_t PEND = 40;                     // > fastest frame rate x 1.1 s
Mark pend[PEND];
uint8_t pend_n = 0;

// Try to give a pending photo its GPS time; returns true when it has been printed.
bool resolvePhoto(const Mark &m, uint32_t now) {
  for (uint8_t i = 0; i + 1 < pps_n; i++) {
    const Pps &a = pps[i], &b = pps[i + 1];
    if (a.sec < 0 || b.sec != (a.sec + 1) % 604800L) continue;   // consecutive seconds (week rollover)
    uint32_t span = b.t - a.t, into = m.t_fall - a.t;
    if (into >= span) continue;                         // not between these two (wrap-safe)
    uint32_t frac_ns = (uint64_t)into * 1000000000ULL / span;
    Serial.print(F("PHOTO,")); Serial.print(m.n);
    Serial.print(','); Serial.print(a.sec);
    Serial.print(','); Serial.print(frac_ns);
    Serial.print(','); Serial.println(span);
    return true;
  }
  if (now - m.t_fall > PHOTO_TIMEOUT_US) {              // no labelled PPS pair around it
    Serial.print(F("PHOTO,")); Serial.print(m.n); Serial.println(F(",-1,0,0"));
    return true;
  }
  return false;
}

void resolvePending() {
  uint32_t now = micros();
  uint8_t keep = 0;
  for (uint8_t i = 0; i < pend_n; i++)
    if (!resolvePhoto(pend[i], now)) pend[keep++] = pend[i];
  pend_n = keep;
}

// ---------- u-blox ----------
UbxParser ubx;
bool f9p_ok = false;                         // VALSET acknowledged
uint8_t baud_idx = 0;
uint32_t last_cfg_ms = 0, last_ubx_ms = 0;

void configureF9P() {
  ValSet v;
  v.add(0x10730001, 1, 1);        // CFG-UART1INPROT-UBX   = 1
  v.add(0x10740001, 1, 1);        // CFG-UART1OUTPROT-UBX  = 1
  v.add(0x10740002, 0, 1);        // CFG-UART1OUTPROT-NMEA = 0
  v.add(0x20910007, 1, 1);        // CFG-MSGOUT-UBX_NAV_PVT_UART1 = every epoch
  v.add(0x30210001, 200, 2);      // CFG-RATE-MEAS = 200 ms (5 Hz; the Pi logs raw data at this rate)
  v.add(0x10050007, 1, 1);        // CFG-TP-TP1_ENA = 1
  v.add(0x20050023, 0, 1);        // CFG-TP-PULSE_DEF = period
  v.add(0x20050030, 1, 1);        // CFG-TP-PULSE_LENGTH_DEF = length
  v.add(0x40050002, 1000000, 4);  // CFG-TP-PERIOD_TP1 = 1 s (before lock)
  v.add(0x40050004, 0, 4);        // CFG-TP-LEN_TP1 = 0: no pulse until the time is locked
  v.add(0x40050003, 1000000, 4);  // CFG-TP-PERIOD_LOCK_TP1 = 1 s
  v.add(0x40050005, 100000, 4);   // CFG-TP-LEN_LOCK_TP1 = 100 ms
  v.add(0x10050009, 1, 1);        // CFG-TP-USE_LOCKED_TP1 = 1
  v.add(0x1005000a, 1, 1);        // CFG-TP-ALIGN_TO_TOW_TP1 = 1 (pulse on whole seconds)
  v.add(0x1005000b, 1, 1);        // CFG-TP-POL_TP1 = 1 (rising edge at the second)
  v.add(0x2005000c, 1, 1);        // CFG-TP-TIMEGRID_TP1 = GPS
  Serial1.write(v.buf, v.finish());
}

void handleUbx() {
  last_ubx_ms = millis();
  if (ubx.cls == 0x05 && ubx.len >= 2 && ubx.u1(0) == 0x06 && ubx.u1(1) == 0x8A) {   // ACK/NAK for VALSET
    if (ubx.id == 0x01 && !f9p_ok) {
      f9p_ok = true;
      Serial.print(F("# F9P configured on UART1 at ")); Serial.print(BAUDS[baud_idx]); Serial.println(F(" baud"));
    } else if (ubx.id == 0x00) {
      Serial.println(F("# F9P rejected the configuration (NAK)"));
    }
  } else if (ubx.cls == 0x01 && ubx.id == 0x07 && ubx.len == 92) {                    // NAV-PVT
    uint32_t itow = ubx.u4(0);
    Serial.print(F("PVT,")); Serial.print(itow);
    Serial.print(','); Serial.print(ubx.u1(20));       // fixType
    Serial.print(','); Serial.print(ubx.u1(21) >> 6);  // carrSoln: 0 none, 1 RTK float, 2 RTK fixed
    Serial.print(','); Serial.print(ubx.u1(23));       // numSV
    Serial.print(','); Serial.print(ubx.i4(28));       // lat 1e-7 deg
    Serial.print(','); Serial.print(ubx.i4(24));       // lon 1e-7 deg
    Serial.print(','); Serial.print(ubx.i4(36));       // hMSL mm
    Serial.print(','); Serial.println(ubx.u4(40));     // hAcc mm
    // Label the PPS: the solution for a whole GPS second arrives shortly after that second's pulse.
    bool time_ok = (ubx.u1(11) & 0x06) == 0x06;        // validTime + fullyResolved
    if (time_ok && itow % 1000 == 0 && pps_n) {
      Pps &p = pps[pps_n - 1];
      uint32_t age = micros() - p.t;
      if (p.sec < 0 && age < 500000UL) {
        p.sec = itow / 1000;
        Serial.print(F("PPS,")); Serial.print(p.t); Serial.print(','); Serial.println(p.sec);
        resolvePending();
      }
    }
  }
}

// Wiring diagnostics: bytes seen on pin 19, and NMEA sentence starts ("$G?") = readable GPS text.
uint32_t rx_bytes = 0, nmea_seen = 0, nmea_baud = 0;

void serviceF9P() {
  static uint8_t p1 = 0, p2 = 0;
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    rx_bytes++;
    if (p2 == '$' && p1 == 'G' && c >= 'A' && c <= 'Z') { nmea_seen++; nmea_baud = BAUDS[baud_idx]; }
    p2 = p1; p1 = c;
    if (ubx.feed(c)) handleUbx();
  }
  uint32_t now = millis();
  if (f9p_ok && now - last_ubx_ms > 3000) {          // receiver went quiet (unplugged / reset)
    f9p_ok = false;
    Serial.println(F("# F9P stopped talking, reconfiguring"));
  }
  if (!f9p_ok && now - last_cfg_ms > 1000) {         // (re)configure; try the next baud each second
    last_cfg_ms = now;
    bool hearing_ubx = last_ubx_ms && now - last_ubx_ms < 1500;   // valid frames = baud already right
    if (!hearing_ubx) {
      if (now > 1500) baud_idx = (baud_idx + 1) % (sizeof BAUDS / sizeof BAUDS[0]);
      Serial1.end();
      Serial1.begin(BAUDS[baud_idx]);
      ubx.reset();
    }
    configureF9P();
  }
}

// ---------- main ----------
void setup() {
  pinMode(PIN_SHOE, INPUT_PULLUP);
  pinMode(PIN_PPS, INPUT);
  Serial.begin(115200);
  Serial.println(F("# hotshoe_logger v3: D2 = hot shoe, D3 = F9P PPS, F9P UART1 on pins 18/19"));
  Serial.println(F("# MARK,n,t_fall_us,width_us | PHOTO,n,gps_sec,frac_ns,period_us | PPS,t_us,gps_sec | PVT,itow_ms,fix,carr,sv,lat_e7,lon_e7,hmsl_mm,hacc_mm"));
  attachInterrupt(digitalPinToInterrupt(PIN_SHOE), onShoe, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), onPps, RISING);
}

void loop() {
  static uint32_t last_alive = 0;
  static uint32_t reported_overflows = 0;

  serviceF9P();

  noInterrupts();
  bool got_pps = pps_new;
  uint32_t t_pps = pps_t;
  pps_new = false;
  interrupts();
  if (got_pps) addPps(t_pps);
  if (pps_n && millis() - last_pps_ms > 2000) pps_n = 0;   // pulses stopped: old ones must never be reused

  // Report a mark once the bounce window is over and the pin is back high, or after 100 ms.
  while (q_tail != q_head) {
    noInterrupts();
    Mark m = { q[q_tail].n, q[q_tail].t_fall, q[q_tail].width };
    bool still_open = q_tail == open_slot && (low_active || micros() - m.t_fall < LOCKOUT_US);
    interrupts();
    if (still_open && micros() - m.t_fall < 100000UL) break;
    Serial.print(F("MARK,")); Serial.print(m.n);
    Serial.print(','); Serial.print(m.t_fall);
    Serial.print(','); Serial.println(m.width);
    if (pend_n < PEND) pend[pend_n++] = m;
    else { Serial.print(F("PHOTO,")); Serial.print(m.n); Serial.println(F(",-1,0,0")); }
    q_tail = (q_tail + 1) % QLEN;
    serviceF9P();
  }
  resolvePending();

  noInterrupts();                            // 32-bit values shared with the ISR: copy atomically
  uint32_t ovf = overflows, cnt = count;
  interrupts();
  if (ovf != reported_overflows) {
    reported_overflows = ovf;
    Serial.print(F("# OVERFLOW,")); Serial.println(ovf);
  }

  uint32_t now = micros();
  if (now - last_alive >= 5000000UL) {
    last_alive = now;
    bool pps_ok = last_pps_ms && millis() - last_pps_ms < 2000;
    Serial.print(F("ALIVE,")); Serial.print(now);
    Serial.print(','); Serial.print(digitalRead(PIN_SHOE));
    Serial.print(','); Serial.print(cnt);
    Serial.print(','); Serial.print(f9p_ok);
    Serial.print(','); Serial.println(pps_ok);
    if (!f9p_ok) {
      if (rx_bytes == 0)
        Serial.println(F("# no reply from F9P: NOTHING arrives on pin 19 (F9P TX1 -> 19? IOREF -> 5V? GND? F9P powered?)"));
      else if (nmea_seen) {
        Serial.print(F("# F9P talks (GPS text at ")); Serial.print(nmea_baud);
        Serial.println(F(" baud) but ignores the Arduino: check Mega pin 18 -> F9P RX1"));
      } else {
        Serial.print(F("# ")); Serial.print(rx_bytes);
        Serial.println(F(" unreadable bytes on pin 19 (noise or wrong baud; check GND and IOREF -> 5V)"));
      }
    } else if (!pps_ok) Serial.println(F("# no PPS on D3 yet (needs a GNSS fix; check the PPS wire)"));
    rx_bytes = nmea_seen = 0;
  }
}
