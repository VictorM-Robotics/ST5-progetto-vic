// Bluno MEGA / ATmega2560. USB Serial 115200. Protocol v2 ONLY.
// Communication watchdog; not an independent hardware emergency stop.
#include <Arduino.h>
#include <util/atomic.h>
#include "parameters.h"
#include "protocol.h"

#if !defined(__AVR_ATmega2560__) && !defined(ST5_NATIVE_TEST)
#error Select Arduino Mega or Mega 2560 / ATmega2560
#endif

const uint8_t pwmPins[4] = {vitMotor1,vitMotor2,vitMotor3,vitMotor4};
const uint8_t dirPins[4] = {dirMotor1,dirMotor2,dirMotor3,dirMotor4};
const uint8_t encoderA[4] = {Encoder1A,Encoder2A,Encoder3A,Encoder4A};
const uint8_t encoderB[4] = {Encoder1B,Encoder2B,Encoder3B,Encoder4B};
const uint32_t WATCHDOG_MS=350, FRAME_TIMEOUT_MS=40, RAMP_MS=20;
const int16_t PWM_LIMIT=100, RAMP_STEP=10;
volatile uint32_t counts[4] = {0,0,0,0}; // modulo 2^32, no signed overflow
int16_t outputs[4]={0,0,0,0}, targets[4]={0,0,0,0};
bool connected=false, armed=false;
uint16_t lastSequence=0;
uint32_t lastDriveMs=0, lastRampMs=0, frameStartedMs=0;
uint8_t rx[st5::FRAME_SIZE], used=0;

void writeMotor(uint8_t i, int16_t value) {
  // Same direction convention as supplied Motor.cpp (_Sns=0).
  analogWrite(pwmPins[i], 0);
  digitalWrite(dirPins[i], value>=0 ? HIGH : LOW);
  analogWrite(pwmPins[i], value<0 ? -value : value);
}
void stopOutputs() {
  for (uint8_t i=0; i<4; ++i) { targets[i]=outputs[i]=0; writeMotor(i,0); }
}
void trip() { stopOutputs(); armed=false; }
void checkWatchdog() {
  if (armed && uint32_t(millis()-lastDriveMs)>=WATCHDOG_MS) trip();
}
void encoder0() { counts[0] += digitalRead(encoderA[0])==HIGH ? 1UL : uint32_t(-1); }
void encoder1() { counts[1] += digitalRead(encoderA[1])==LOW  ? 1UL : uint32_t(-1); }
void encoder2() { counts[2] += digitalRead(encoderA[2])==HIGH ? 1UL : uint32_t(-1); }
void encoder3() { counts[3] += digitalRead(encoderA[3])==LOW  ? 1UL : uint32_t(-1); }

void processFrame() {
  uint8_t reply[st5::FRAME_SIZE]={0xA5,0x5A,st5::VERSION, uint8_t(rx[3]|0x80),0,0,0};
  const uint8_t command=rx[3];
  const uint16_t sequence=st5::read16(rx+4);
  st5::write16(reply+4,sequence);
  if (rx[6]!=0) { trip(); reply[6]=st5::BAD_ARGUMENT; }
  else if (command=='H') {
    // Every handshake disarms; it never starts motors or refreshes an old command.
    trip(); connected=true; lastSequence=sequence;
    reply[7]=st5::VERSION;
    st5::write16(reply+8, WATCHDOG_MS);
    st5::write16(reply+10, PWM_LIMIT);
  } else if (command=='X') {
    // A valid emergency stop is accepted even if the sequence is out of order.
    trip(); connected=false;
  } else if (!connected || sequence!=uint16_t(lastSequence+1)) {
    trip(); connected=false; reply[6]=st5::BAD_SEQUENCE;
  } else {
    lastSequence=sequence;
    if (command=='R') {
      stopOutputs(); armed=true; lastDriveMs=millis(); lastRampMs=millis();
    } else if (command=='C') {
      int16_t next[4]; bool zero=true, legal=true;
      for (uint8_t i=0; i<4; ++i) {
        uint16_t raw=st5::read16(rx+7+2*i);
        int32_t signedValue=raw<=32767 ? int32_t(raw) : int32_t(raw)-65536L;
        next[i]=int16_t(signedValue);
        if (signedValue < -PWM_LIMIT || signedValue > PWM_LIMIT) legal=false;
        if (next[i]!=0) zero=false;
      }
      if (!legal) { trip(); reply[6]=st5::BAD_ARGUMENT; }
      else if (zero) { stopOutputs(); if (armed) lastDriveMs=millis(); }
      else if (!armed) { reply[6]=st5::DISARMED; }
      else {
        for (uint8_t i=0; i<4; ++i) targets[i]=next[i];
        lastDriveMs=millis();
      }
    } else if (command=='N' || command=='n') {
      const uint8_t i=command=='N' ? 0 : 2;
      uint32_t a,b;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { a=counts[i]; b=counts[i+1]; }
      st5::write32(reply+7,a); st5::write32(reply+11,b);
    } else if (command=='Q') {
      reply[7]=armed ? 1 : 0;
    } else { trip(); reply[6]=st5::UNKNOWN_COMMAND; }
  }
  st5::finish(reply);
  // Never block on a full TX buffer. Drop reply, disarm; the host times out.
  if (Serial.availableForWrite()>=st5::FRAME_SIZE) Serial.write(reply,sizeof(reply));
  else trip();
}

void setup() {
  for (uint8_t i=0; i<4; ++i) {
    pinMode(pwmPins[i],OUTPUT); pinMode(dirPins[i],OUTPUT);
    pinMode(encoderA[i],INPUT); pinMode(encoderB[i],INPUT);
  }
  trip(); Serial.begin(SERIAL_BAUD);
  attachInterrupt(digitalPinToInterrupt(encoderB[0]),encoder0,FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderB[1]),encoder1,FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderB[2]),encoder2,FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderB[3]),encoder3,FALLING);
}

void loop() {
  checkWatchdog();
  if (used && uint32_t(millis()-frameStartedMs)>=FRAME_TIMEOUT_MS) {
    used=0; trip(); connected=false;
  }
  // Bounded work per iteration: even continuous RX cannot starve the watchdog.
  for (uint8_t budget=0; budget<st5::FRAME_SIZE && Serial.available(); ++budget) {
    uint8_t byte=uint8_t(Serial.read());
    if (!used) {
      if (byte==0xA5) { rx[used++]=byte; frameStartedMs=millis(); }
    } else if (used==1 && byte!=0x5A) {
      trip(); connected=false;
      used=(byte==0xA5) ? 1 : 0; frameStartedMs=millis();
    } else {
      rx[used++]=byte;
      if (used==st5::FRAME_SIZE) {
        checkWatchdog();
        if (st5::valid(rx)) processFrame();
        else { trip(); connected=false; }
        used=0;
      }
    }
  }
  checkWatchdog();
  const uint32_t now=millis();
  if (armed && uint32_t(now-lastRampMs)>=RAMP_MS) {
    lastRampMs=now;
    for (uint8_t i=0; i<4; ++i) {
      int16_t goal=targets[i];
      if ((outputs[i]>0 && goal<0) || (outputs[i]<0 && goal>0)) goal=0;
      int16_t difference=goal-outputs[i];
      if (difference>RAMP_STEP) difference=RAMP_STEP;
      if (difference < -RAMP_STEP) difference=-RAMP_STEP;
      outputs[i]+=difference; writeMotor(i,outputs[i]);
    }
  }
}
