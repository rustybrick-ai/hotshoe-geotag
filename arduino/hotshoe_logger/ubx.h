// Minimal UBX frame parser and CFG-VALSET builder (no Arduino dependencies,
// so it can be unit-tested on a PC).
#pragma once
#include <stdint.h>
#include <string.h>

struct UbxParser {
  static const uint16_t MAX_PAYLOAD = 100;   // NAV-PVT (92) is the largest we read
  uint8_t cls = 0, id = 0;
  uint16_t len = 0;
  uint8_t payload[MAX_PAYLOAD];

  // Feed one byte; returns true when a complete frame with a valid checksum is in cls/id/len/payload.
  // Frames longer than MAX_PAYLOAD are dropped at once (so line noise with a huge length can't stall it).
  void reset() { state = 0; }
  bool feed(uint8_t b) {
    switch (state) {
      case 0: if (b == 0xB5) state = 1; return false;
      case 1: state = (b == 0x62) ? 2 : (b == 0xB5 ? 1 : 0); return false;
      case 2: cls = b; ca = cb = 0; add(b); state = 3; return false;
      case 3: id = b; add(b); state = 4; return false;
      case 4: len = b; add(b); state = 5; return false;
      case 5: len |= (uint16_t)b << 8; add(b); pos = 0;
              state = len > MAX_PAYLOAD ? 0 : (len ? 6 : 7);   // too long: not a frame we want, resync
              return false;
      case 6: add(b); payload[pos] = b; if (++pos >= len) state = 7; return false;
      case 7: state = (b == ca) ? 8 : 0; return false;
      case 8: state = 0; return b == cb;
    }
    state = 0;
    return false;
  }

  uint8_t  u1(uint16_t o) const { return payload[o]; }
  uint16_t u2(uint16_t o) const { return payload[o] | (uint16_t)payload[o + 1] << 8; }
  uint32_t u4(uint16_t o) const {
    return payload[o] | (uint32_t)payload[o + 1] << 8 | (uint32_t)payload[o + 2] << 16 | (uint32_t)payload[o + 3] << 24;
  }
  int32_t  i4(uint16_t o) const { return (int32_t)u4(o); }

 private:
  uint8_t state = 0, ca = 0, cb = 0;
  uint16_t pos = 0;
  void add(uint8_t b) { ca += b; cb += ca; }
};

// Builds a UBX-CFG-VALSET frame (RAM layer only: nothing is saved to the receiver's flash).
struct ValSet {
  uint8_t buf[128];
  uint16_t n = 0;
  ValSet() {
    const uint8_t head[] = {0xB5, 0x62, 0x06, 0x8A, 0, 0, 0x00, 0x01, 0x00, 0x00};  // version 0, layer RAM
    memcpy(buf, head, sizeof head);
    n = sizeof head;
  }
  // size = value bytes (1, 2 or 4), taken from bits 28..30 of the key in the u-blox config database.
  void add(uint32_t key, uint32_t value, uint8_t size) {
    for (uint8_t i = 0; i < 4; i++) buf[n++] = key >> (8 * i);
    for (uint8_t i = 0; i < size; i++) buf[n++] = value >> (8 * i);
  }
  // Fills in the length and checksum; returns the frame length.
  uint16_t finish() {
    uint16_t plen = n - 6;
    buf[4] = plen & 0xFF;
    buf[5] = plen >> 8;
    uint8_t a = 0, b = 0;
    for (uint16_t i = 2; i < n; i++) { a += buf[i]; b += a; }
    buf[n++] = a;
    buf[n++] = b;
    return n;
  }
};
