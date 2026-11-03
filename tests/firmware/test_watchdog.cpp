// Executes the actual sketch against fake Arduino I/O. Not an AVR/hardware test.
#include "Arduino.h"
#include <cassert>
#include <cstring>
#include <iostream>
uint32_t fakeTime=0;
int fakePWM[70]={0},fakeDigital[70]={0};
FakeSerial Serial;
#include "../../firmware_watchdog/serial_link/serial_link.ino"

uint16_t sequence=0;
std::vector<uint8_t> frame(char command,int16_t pwm=0) {
  std::vector<uint8_t> f(17,0);
  f[0]=0xA5; f[1]=0x5A; f[2]=2; f[3]=command;
  st5::write16(f.data()+4,++sequence);
  if (command=='C') for (int i=0;i<4;++i) st5::write16(f.data()+7+2*i,uint16_t(pwm));
  st5::finish(f.data()); return f;
}
uint8_t send(const std::vector<uint8_t>& f) {
  Serial.output.clear();
  Serial.input.insert(Serial.input.end(),f.begin(),f.end()); loop();
  assert(Serial.output.size()==17); assert(st5::valid(Serial.output.data()));
  return Serial.output[6];
}
void reset(uint32_t time=0) {
  fakeTime=time; Serial=FakeSerial(); connected=false; armed=false; used=0;
  sequence=lastSequence=0; lastDriveMs=lastRampMs=frameStartedMs=0;
  for (int i=0;i<4;++i) counts[i]=0;
  setup();
}
void zero() { for (int i=0;i<4;++i) { assert(fakePWM[pwmPins[i]]==0); assert(targets[i]==0); } }
void start() {
  assert(send(frame('H'))==st5::OK); assert(!armed);
  assert(send(frame('R'))==st5::OK); assert(armed);
  assert(send(frame('C',60))==st5::OK);
  fakeTime+=20; loop(); assert(fakePWM[pwmPins[0]]==10);
}
int main(int argc, char **) {
  if (argc>1) {
    // Binary test transport: time LE32 + request[17]; length U8 + reply.
    reset();
    char input[21];
    while (std::cin.read(input,sizeof(input))) {
      fakeTime=0;
      for (uint8_t i=0;i<4;++i) fakeTime |= uint32_t(uint8_t(input[i])) << (8*i);
      Serial.output.clear();
      for (uint8_t i=4;i<21;++i) Serial.input.push_back(uint8_t(input[i]));
      loop();
      std::cout.put(char(Serial.output.size()));
      for (uint8_t b:Serial.output) std::cout.put(char(b));
      std::cout.flush();
    }
    return 0;
  }
  const uint8_t reference[]={'1','2','3','4','5','6','7','8','9'};
  assert(st5::crc16(reference,9)==0x29B1);
  reset(); zero();
  send(frame('H')); assert(send(frame('C',60))==st5::DISARMED); zero();
  reset(); start();
  fakeTime=349; loop(); assert(armed);
  fakeTime=350; loop(); assert(!armed); zero();
  assert(send(frame('C',60))==st5::DISARMED); zero();
  assert(send(frame('R'))==st5::OK); assert(send(frame('C',60))==st5::OK);
  fakeTime+=20; loop(); assert(fakePWM[pwmPins[0]]>0);
  assert(send(frame('C',0))==st5::OK); zero(); // immediate, no ramp wait

  reset(); start();
  for (int i=1;i<=4;++i) { fakeTime=i*100; send(frame('N')); }
  assert(!armed); zero(); // telemetry is not a motor heartbeat

  reset(); start();
  auto corrupt=frame('C',60); corrupt[8]^=1;
  Serial.input.insert(Serial.input.end(),corrupt.begin(),corrupt.end()); loop();
  assert(!armed && !connected); zero();

  reset(); start();
  auto partial=frame('C',60);
  Serial.input.insert(Serial.input.end(),partial.begin(),partial.begin()+8); loop();
  fakeTime+=40; loop(); assert(!armed && !connected); zero();

  reset(); start();
  assert(send(frame('C',101))==st5::BAD_ARGUMENT); assert(!armed); zero();

  reset(); start(); auto duplicate=frame('Q'); send(duplicate);
  assert(send(duplicate)==st5::BAD_SEQUENCE); assert(!armed); zero();

  reset(); start();
  auto emergency=frame('X'); st5::write16(emergency.data()+4,500); st5::finish(emergency.data());
  assert(send(emergency)==st5::OK); assert(!armed && !connected); zero();

  reset(); start(); Serial.capacity=0;
  auto query=frame('Q'); Serial.input.insert(Serial.input.end(),query.begin(),query.end());
  loop(); assert(!armed); zero(); // TX congestion does not block stop

  reset(0xFFFFFF00UL); start(); fakeTime+=330; loop(); assert(!armed); zero();

  reset(); auto hello=frame('H');
  for (uint8_t b:hello) { Serial.input.push_back(b); loop(); ++fakeTime; }
  assert(connected && !armed); assert(st5::valid(Serial.output.data()));

  counts[0]=0xFFFFFFFFUL; fakeDigital[Encoder1A]=HIGH; encoder0(); assert(counts[0]==0);
  counts[0]=0xFFFFFFFFUL; counts[1]=0x80000000UL;
  send(frame('N')); assert(Serial.output[7]==0xFF && Serial.output[14]==0x80);
  std::cout << "Firmware native scenarios passed (no AVR/hardware validation)\n";
}
