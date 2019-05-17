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
  begin(clockFreq, csPin, -1, -1, -1);
}

void TMC4361::begin(long clockFreq, int csPin, int intPin)
{
  begin(clockFreq, csPin, intPin, -1, -1);
}

void TMC4361::begin(long clockFreq, int csPin, int intPin, int startPin)
{
  begin(clockFreq, csPin, intPin, startPin, -1);
}

void TMC4361::begin(long clockFreq, int csPin, int intPin, int startPin, int rstPin)
{
  _clockFreq = clockFreq;
  _csPin = csPin;
  _intPin = intPin;
  _startPin = startPin;
  _rstPin = rstPin;

  SPI.begin(); //Init SPI hardware
  _spiSettings = SPISettings(clockFreq/4, MSBFIRST, SPI_MODE3);
  SPI.usingInterrupt(intPin);

  digitalWrite(_csPin, HIGH);
  pinMode(_csPin, OUTPUT);

  if (_intPin > -1)
  {
    pinMode(_intPin, INPUT);
    //TODO attachInterrupt ?
  }

  if (_startPin > -1)
  {
    pinMode(_startPin, INPUT);
  }

  if (_rstPin > -1)
  {
    digitalWrite(_rstPin, HIGH);
    pinMode(_rstPin, OUTPUT);
  }

  reset();

  writeRegister(TMC4361_CLK_FREQ_REGISTER, clockFreq);
  setOutputTimings(_defaultStepLength, _defaultDirSetupTime);
  
  // Protect all events from automatic clearing when reading EVENTS register. 
  // This way individual events can be checked at the price of reading the register once per check.
  writeRegister(TMC4361_EVENT_CLEAR_CONF_REGISTER, 0xFFFFFFFF);
  clearEvents();

  //TODO init FREEZE register
}

void TMC4361::reset()
{
  if (_rstPin > -1)
  {
    digitalWrite(_rstPin, LOW);
    delay(2);
    digitalWrite(_rstPin, HIGH);
  }
  else
  {
    //Write magic value to the reset register
    writeRegister(TMC4361_RESET_CLK_GATING_REGISTER, 0x525354 << 8);
  }
}

bool TMC4361::checkFlag(TMC4361::FlagType flag)
{
  return readRegisterBit(TMC4361_STATUS_REGISTER, flag);
}

bool TMC4361::isTargetReached()
{
  return checkFlag(TARGET_REACHED_F);
}

void TMC4361::clearEvents()
{
  writeRegister(TMC4361_EVENTS_REGISTER, 0xFFFFFFFF);
}

bool TMC4361::checkEvent(EventType event)
{
  bool value = readRegisterBit(TMC4361_EVENTS_REGISTER, event);
  
  if (value)
    writeRegister(TMC4361_EVENTS_REGISTER, 1 << event);
    
  return value;
}

void TMC4361::setOutputsPolarity(bool stepInverted, bool dirInverted)
{
  long generalConfigReg = readRegister(TMC4361_GENERAL_CONFIG_REGISTER);

  bitWrite(generalConfigReg, 3, stepInverted);
  bitWrite(generalConfigReg, 5, dirInverted);

  writeRegister(TMC4361_GENERAL_CONFIG_REGISTER, generalConfigReg);
}

void TMC4361::setOutputTimings(int stepWidth, int dirSetupTime)
{
  long registerValue =
    ((stepWidth * _clockFreq / 1000000L - 1) & 0xFFFF) |
    (((dirSetupTime * _clockFreq / 1000000L) & 0xFFFF) << 16);

  writeRegister(TMC4361_STP_LENGTH_ADD, registerValue);
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

long TMC4361::getTargetPosition()
{
  return readRegister(TMC4361_X_TARGET_REGISTER);
}

void TMC4361::setTargetPosition(long position)
{
  writeRegister(TMC4361_X_TARGET_REGISTER, position);
}

void TMC4361::stop()
{
  setMaxSpeed(0.0);
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

void TMC4361::setRegisterBit(const byte address, const byte bit)
{
  uint32_t value = readRegister(address);
  bitSet(value, bit);
  writeRegister(address, value);
}

void TMC4361::clearRegisterBit(const byte address, const byte bit)
{
  uint32_t value = readRegister(address);
  bitClear(value, bit);
  writeRegister(address, value);
}

bool TMC4361::readRegisterBit(const byte address, const byte bit)
{
  return bitRead(readRegister(address), bit);
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
