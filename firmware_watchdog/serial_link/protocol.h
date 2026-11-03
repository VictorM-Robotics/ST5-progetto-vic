#ifndef ST5_PROTOCOL_H
#define ST5_PROTOCOL_H
#include <stdint.h>

// Fixed 17-byte frames: A5 5A, version, command, sequence LE16, status,
// 8 payload bytes, CRC16-CCITT-FALSE LE16 over bytes 2..14.
namespace st5 {
const uint8_t FRAME_SIZE = 17;
const uint8_t VERSION = 2;
enum Status { OK = 0, DISARMED = 1, BAD_ARGUMENT = 2, UNKNOWN_COMMAND = 3,
              BAD_SEQUENCE = 4 };
inline uint16_t crc16(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    crc ^= uint16_t(*data++) << 8;
    for (uint8_t bit=0; bit<8; ++bit)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}
inline uint16_t read16(const uint8_t *p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
inline void write16(uint8_t *p, uint16_t n) { p[0]=n; p[1]=n >> 8; }
inline void write32(uint8_t *p, uint32_t n) {
  for (uint8_t i=0; i<4; ++i) p[i]=n >> (8*i);
}
inline void finish(uint8_t *frame) { write16(frame+15, crc16(frame+2, 13)); }
inline bool valid(const uint8_t *frame) {
  return frame[0]==0xA5 && frame[1]==0x5A && frame[2]==VERSION &&
         read16(frame+15)==crc16(frame+2, 13);
}
}
#endif
