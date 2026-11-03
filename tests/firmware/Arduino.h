#ifndef ST5_FAKE_ARDUINO_H
#define ST5_FAKE_ARDUINO_H
#include <stdint.h>
#include <stddef.h>
#include <deque>
#include <vector>
const int A6=60, HIGH=1, LOW=0, OUTPUT=1, INPUT=0, FALLING=2;
extern uint32_t fakeTime;
extern int fakePWM[70], fakeDigital[70];
inline uint32_t millis() { return fakeTime; }
inline void pinMode(int,int) {}
inline void analogWrite(int pin,int value) { fakePWM[pin]=value; }
inline void digitalWrite(int pin,int value) { fakeDigital[pin]=value; }
inline int digitalRead(int pin) { return fakeDigital[pin]; }
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int,void(*)(),int) {}
struct FakeSerial {
  std::deque<uint8_t> input;
  std::vector<uint8_t> output;
  int capacity=64;
  void begin(int) {}
  int available() { return input.size(); }
  int availableForWrite() { return capacity; }
  int read() { uint8_t b=input.front(); input.pop_front(); return b; }
  size_t write(const uint8_t *p,size_t n) {
    output.insert(output.end(),p,p+n); return n;
  }
};
extern FakeSerial Serial;
#endif
