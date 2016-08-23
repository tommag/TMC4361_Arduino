/*
 * TMC4361 simple test sketch.
 * Initializes the IC then performs back and forth movements with "6-point" movements (initial and final speed, two different accelerations and decelerations).
 *
 * The speed as reported by the motion controller is printed on the serial port every 50ms.
 *
 * Tested on Teensy 3.1 / 3.2
 *
 * Tom Magnier <tom@tmagnier.fr> 08/2016
 */
#include "TMC4361.h"

#define TMC4361_CS_PIN      6
#define TMC4361_CLK_PIN     5
#define TMC4361_RST_PIN     7
#define TMC4361_TARGET_PIN  18
#define TMC4361_START_PIN   17
#define TMC4361_INT_PIN     15
#define TMC4361_SCK_PIN     14

#define TMC4361_CLK_FREQ    8000000L

TMC4361 tmc;

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting TMC4361 6-point test...");

  //Init TMC4361 clock
  pinMode(TMC4361_CLK_PIN, OUTPUT);
  analogWriteFrequency(TMC4361_CLK_PIN, TMC4361_CLK_FREQ);
  analogWrite(TMC4361_CLK_PIN, 128);

  //Init SPI SCK pin
  SPI.setSCK(TMC4361_SCK_PIN);

  tmc.begin(TMC4361_CLK_FREQ, TMC4361_CS_PIN, TMC4361_INT_PIN, TMC4361_START_PIN);

  tmc.setRampMode(TMC4361::POSITIONING_MODE, TMC4361::TRAPEZOIDAL_RAMP);
  tmc.setMaxSpeed(200);
  tmc.setAccelerations(25, 10, 50, 20);
  tmc.setRampSpeeds(20, 40, 150);
  tmc.setTargetPosition(1000);
}

void loop()
{
  Serial.println(tmc.getCurrentSpeed());

  long currentPos = tmc.getCurrentPosition();
  if (currentPos == 1000)
    tmc.setTargetPosition(-1000);
  else if (currentPos == -1000)
    tmc.setTargetPosition(1000);

  delay(50);
}
