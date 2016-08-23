/*
 * TMC4361 Motion control IC by Trinamic.
 *
 * Tom Magnier <tom@tmagnier.fr> 08/2016
 */

#include "Arduino.h"
#include "TMC4361.h"

TMC4361::TMC4361()
{

}

void TMC4361::begin(long clockFreq, int csPin)
{
  begin(clockFreq, csPin, -1, -1);
}

void TMC4361::begin(long clockFreq, int csPin, int intPin)
{
  begin(clockFreq, csPin, intPin, -1);
}

void TMC4361::begin(long clockFreq, int csPin, int intPin, int startPin)
{
  _clockFreq = clockFreq;
  _csPin = csPin;
  _intPin = intPin;
  _startPin = startPin;

  SPI.begin(); //Init SPI hardware
  _spiSettings = SPISettings(clockFreq/4, MSBFIRST, SPI_MODE3);
  //TODO SPI.usingInterrupt ? If SPI transactions are performed from the interrupt.

  digitalWrite(_csPin, HIGH);
  pinMode(_csPin, OUTPUT);

  if (_intPin > -1)
  {
    pinMode(_intPin, INPUT);
    //TODO attachInterrupt
  }

  if (_startPin > -1)
  {
    pinMode(_startPin, INPUT);
  }

  writeRegister(TMC4361_CLK_FREQ_REGISTER, clockFreq);

  //TODO configure output ; FREEZE register
}

void TMC4361::setRampMode(TMC4361::RampMode mode, TMC4361::RampType type)
{
  writeRegister(TMC4361_RAMP_MODE_REGISTER, mode | type);
}

long TMC4361::getCurrentPosition()
{
  return readRegister(TMC4361_X_ACTUAL_REGISTER);
}

void TMC4361::setCurrentPosition(long position)
{
  writeRegister(TMC4361_X_ACTUAL_REGISTER, position);
}

void TMC4361::setTargetPosition(long position)
{
  writeRegister(TMC4361_X_TARGET_REGISTER, position);
}

float TMC4361::getCurrentSpeed()
{
  return (float)readRegister(TMC4361_V_ACTUAL_REGISTER);
}

float TMC4361::getCurrentAcceleration()
{
  return (float)readRegister(TMC4361_A_ACTUAL_REGISTER);
}

void TMC4361::setMaxSpeed(float speed)
{
  writeRegister(TMC4361_V_MAX_REGISTER, floatToFixedPoint(speed, 8));
}

void TMC4361::setRampSpeeds(float startSpeed, float stopSpeed, float breakSpeed)
{
  writeRegister(TMC4361_V_START_REGISTER, floatToFixedPoint(abs(startSpeed), 8));
  writeRegister(TMC4361_V_STOP_REGISTER, floatToFixedPoint(abs(stopSpeed), 8));
  writeRegister(TMC4361_V_BREAK_REGISTER, floatToFixedPoint(abs(breakSpeed), 8));
}

void TMC4361::setAccelerations(float maxAccel, float maxDecel, float startAccel, float finalDecel)
{
  writeRegister(TMC4361_A_MAX_REGISTER, floatToFixedPoint(abs(maxAccel), 2) & 0xFFFFFF);
  writeRegister(TMC4361_D_MAX_REGISTER, floatToFixedPoint(abs(maxDecel), 2) & 0xFFFFFF);
  writeRegister(TMC4361_A_START_REGISTER, floatToFixedPoint(abs(startAccel), 2) & 0xFFFFFF);
  writeRegister(TMC4361_D_FINAL_REGISTER, floatToFixedPoint(abs(finalDecel), 2) & 0xFFFFFF);
}

void TMC4361::setBowValues(long bow1, long bow2, long bow3, long bow4)
{
  writeRegister(TMC4361_BOW_1_REGISTER, abs(bow1) & 0xFFFFFF);
  writeRegister(TMC4361_BOW_2_REGISTER, abs(bow2) & 0xFFFFFF);
  writeRegister(TMC4361_BOW_3_REGISTER, abs(bow3) & 0xFFFFFF);
  writeRegister(TMC4361_BOW_4_REGISTER, abs(bow4) & 0xFFFFFF);
}

void TMC4361::writeRegister(const byte address, const long data)
{
  spiTransfer(address | 0x80, data);
}

long TMC4361::readRegister(const byte address)
{
  spiTransfer(address & 0x7F, 0); //Dummy call to load the read address
  return spiTransfer(address & 0x7F, 0);
}

long TMC4361::spiTransfer(const byte address, const long data)
{
  long returnBuffer = 0;

  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);

  _spiStatus = SPI.transfer(address);
  //Send data MSB first
  for (int i = 3; i >= 0; i--)
    returnBuffer |= (SPI.transfer((data >> (i*8)) & 0xFF) << (i*8));

  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  return returnBuffer;
}

long TMC4361::floatToFixedPoint(float value, int decimalPlaces)
{
  value *= (float)(1 << decimalPlaces);
  return (long)((value > 0.0) ? (value + 0.5) : (value - 0.5));
}

float TMC4361::fixedPointToFloat(long value, int decimalPlaces)
{
  return (float)(value) / (float)(1 << decimalPlaces);
}
