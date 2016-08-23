/*
 * TMC4361 Motion control IC by Trinamic.
 *
 * Note that this library doesn't provide a clock to the TMC4361 as this is platform-dependent.
 *
 * Tom Magnier <tom@tmagnier.fr> 08/2016
 */

#ifndef TMC4361_H
#define TMC4361_H

#include "Arduino.h"
#include "SPI.h"

//registers for TMC4361 (from https://github.com/trinamic/T-Bone/blob/master/Software/ArduinoClient/constants.h)
#define TMC4361_GENERAL_CONFIG_REGISTER 0x0
#define TMC4361_REFERENCE_CONFIG_REGISTER 0x01
#define TMC4361_START_CONFIG_REGISTER 0x2
#define TMC4361_INPUT_FILTER_REGISTER 0x3
#define TMC4361_SPIOUT_CONF_REGISTER 0x04
#define TMC4361_ENCODER_INPUT_CONFIG_REGISTER 0x07
#define TMC4361_STEP_CONF_REGISTER 0x0A
#define TMC4361_EVENT_CLEAR_CONF_REGISTER 0x0c
#define TMC4361_INTERRUPT_CONFIG_REGISTER 0x0d
#define TMC4361_EVENTS_REGISTER 0x0e
#define TMC4361_STATUS_REGISTER 0x0f
#define TMC4361_START_OUT_ADD_REGISTER 0x11
#define TMC4361_GEAR_RATIO_REGISTER 0x12
#define TMC4361_START_DELAY_REGISTER 0x13
#define TMC4361_RAMP_MODE_REGISTER 0x20
#define TMC4361_X_ACTUAL_REGISTER 0x21
#define TMC4361_V_ACTUAL_REGISTER 0x22
#define TMC4361_A_ACTUAL_REGISTER 0x23
#define TMC4361_V_MAX_REGISTER 0x24
#define TMC4361_V_START_REGISTER 0x25
#define TMC4361_V_STOP_REGISTER 0x26
#define TMC4361_V_BREAK_REGISTER 0x27
#define TMC4361_A_MAX_REGISTER 0x28
#define TMC4361_D_MAX_REGISTER 0x29
#define TMC4361_A_START_REGISTER 0x2a
#define TMC4361_D_FINAL_REGISTER 0x2b
#define TMC4361_BOW_1_REGISTER 0x2d
#define TMC4361_BOW_2_REGISTER 0x2e
#define TMC4361_BOW_3_REGISTER 0x2f
#define TMC4361_BOW_4_REGISTER 0x30
#define TMC4361_CLK_FREQ_REGISTER 0x31
#define TMC4361_POSITION_COMPARE_REGISTER 0x32
#define TMC4361_VIRTUAL_STOP_LEFT_REGISTER 0x33
#define TMC4361_VIRTUAL_STOP_RIGHT_REGISTER 0x34
#define TMC4361_X_LATCH_REGISTER 0x36
#define TMC4361_X_TARGET_REGISTER 0x37
#define TMC4361_X_TARGET_PIPE_0_REGSISTER 0x38
#define TMC4361_SH_V_MAX_REGISTER 0x40
#define TMC4361_SH_A_MAX_REGISTER 0x41
#define TMC4361_SH_D_MAX_REGISTER 0x42
#define TMC4361_SH_VBREAK_REGISTER 0x45
#define TMC4361_SH_V_START_REGISTER 0x46
#define TMC4361_SH_V_STOP_REGISTER 0x47
#define TMC4361_SH_BOW_1_REGISTER 0x48
#define TMC4361_SH_BOW_2_REGISTER 0x49
#define TMC4361_SH_BOW_3_REGISTER 0x4A
#define TMC4361_SH_BOW_4_REGISTER 0x4B
#define TMC4361_SH_RAMP_MODE_REGISTER 0x4C
#define TMC4361_ENCODER_POSITION_REGISTER 0x50
#define TMC4361_ENCODER_INPUT_RESOLUTION_REGISTER 0x54
#define TMC4361_COVER_LOW_REGISTER 0x6c
#define TMC4361_COVER_HIGH_REGISTER 0x6d

class TMC4361
{
public:
  enum RampMode {
    VELOCITY_MODE = 0x00,
    POSITIONING_MODE = (0x01 << 2)
  };

  enum RampType {
    HOLD_RAMP = 0x00, //Follow max speed (rectangle shape)
    TRAPEZOIDAL_RAMP = 0x01,
    S_SHAPED_RAMP = 0x02
  };

  TMC4361();
  void begin(long clockFreq, int csPin);
  void begin(long clockFreq, int csPin, int intPin);
  void begin(long clockFreq, int csPin, int intPin, int startPin);

  //TODO events / status configuration

  //TODO output configuration

  /* Ramp generator commands */
  void setRampMode(RampMode mode, RampType type);

  /* Return the current internal position (in steps) */
  long getCurrentPosition();

  /* Set the current internal position (in steps) */
  void setCurrentPosition(long position);

  /* Set the target position
   * /!\ Set all other motion profile parameters before
   */
  void setTargetPosition(long position);

  /* Return the current speed (in steps / second) */
  float getCurrentSpeed();

  /* Return the current acceleration (in steps / second^2) */
  float getCurrentAcceleration();

  /* Set the max speed VMAX (steps/second)
   * /!\ Don't exceed clockFreq / 2 in velocity mode and clockFreq / 4 in positioning mode
   */
  void setMaxSpeed(float speed);

  /* Set the ramp start, stop and break speeds (in steps / second). See datasheet §6.4 for details */
  void setRampSpeeds(float startSpeed, float stopSpeed, float breakSpeed);

  /* Set the ramp accelerations (in steps / second^2). See datasheet §6.3.6 */
  void setAccelerations(float maxAccel, float maxDecel, float startAccel, float finalDecel);

  /* Set the bow values for S-shaped ramps (in steps / second^3). */
  void setBowValues(long bow1, long bow2, long bow3, long bow4);


private:
  long _clockFreq; //TMC4361 clock frequency (Hz)
  int _csPin; //Chip Select pin number
  int _startPin; //Start signal pin number
  int _intPin; //Interrupt line pin number

  byte _spiStatus; //Contents of the status bits SPI_STATUS updated on each SPI transaction

  SPISettings _spiSettings;

  //TODO are multi-register versions needed ?
  void writeRegister(const byte address, const long data);
  long readRegister(const byte address);
  long spiTransfer(const byte address, const long data);
  long floatToFixedPoint(float value, int decimalPlaces);
  float fixedPointToFloat(long value, int decimalPlaces);
};

#endif //TMC4361_H
