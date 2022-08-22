/*
   COMPACT FACETING HEAD ANGLE CONTROL WIDTH DEPTH OF CUT INDICATOR

   Arduino Mega
   20x4 I2C Display
   Rotary Encoder (1000 steps, used to get 4000 ticks)
   Depth of Cut Indicator (by simple touch sensor)

   Stage 2:
   (A4988 Stepper motor controller)
   (4x4 Matrix keypad)

*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define USE_TIMER_1     false
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     true
// Warning: Do not move the above, below the following includes, see:
// https://github.com/khoih-prog/TimerInterrupt/discussions/21#discussioncomment-1477436
#include <TimerInterrupt.h>
#include <ISR_Timer.h>

// uncomment for debug mode
//#define DEBUG_MODE

// INPUT PORTS
#define IN_ROTENC_PIN_A           2   // 2-3 are the interrupt-enabled inputs
#define IN_ROTENC_PIN_B           3
#define IN_ROTENC_PIN_C           4   // Index input for calibration
#define IN_TOUCH_SENSOR           5   // Digital input if needle is touching angle limit


// OUTPUT PORTS
#define OUT_DEPTH_OF_CUT_LVL1     22   // close
#define OUT_DEPTH_OF_CUT_LVL2     23   // closer
#define OUT_DEPTH_OF_CUT_LVL3     24   // touch!
// I2C should be 20(SDA) and 21 (SCL)

// HALL sensor for RPMs, based on TimerInterrupt library and the following example:
// https://github.com/khoih-prog/TimerInterrupt/blob/master/examples/ISR_RPM_Measure/ISR_RPM_Measure.ino
#define HALL_SENSOR                18
#define KAVG                      100
#define TIMER_INTERVAL_MS           1
#define DEBOUNCING_INTERVAL_MS     80
volatile boolean hallState;
volatile unsigned int hallTick = 0;
volatile unsigned int previoustime = 0;
volatile bool activeState = false;
volatile unsigned long rotationTime = 0;
float RPM       = 0.00;
float avgRPM    = 0.00;
volatile int debounceCounter;


// Rotary Encoder
volatile unsigned int encPos;          // This is the current encoder count
unsigned int encCal = 4096;            // Encoder counts at calibration point
unsigned int enc0 = 0;                 // Counter at 0 degrees (table cut)
boolean Aset;                          // Whether we just had a positive going edge on A
boolean Bset;                          //   and on B. These keep track of where we are
boolean isCalibState;                  // This gets set when we have a new calibration
// https://www.quantumdev.com/how-to-calculate-pulses-per-degree-for-an-incremental-encoder/
unsigned int encoder_ppr = 2000;       // pulses per revolution for the encoder (CHANGEME)
float degPerStep = 360/encoder_ppr;    // The number of degrees per encoder step (1000 counts)

// Touch Sensor
// ring buffer for average calculation of touch sensor
# define RINGBUF_SIZE 80               // tied to the number of characters per display line
boolean touchBuf[] = { false, false, false, false, false, false, false, false, false, false,
                       false, false, false, false, false, false, false, false, false, false,
                       false, false, false, false, false, false, false, false, false, false,
                       false, false, false, false, false, false, false, false, false, false,
                       false, false, false, false, false, false, false, false, false, false,
                       false, false, false, false, false, false, false, false, false, false,
                       false, false, false, false, false, false, false, false, false, false,
                       false, false, false, false, false, false, false, false, false, false
                     };
int bufIndex = 0; // travelling round robin through the ring buffer
int ledsThresholds [] = { 1, RINGBUF_SIZE / 2, RINGBUF_SIZE - 2 };
int curTouchDegree = 0;
int lastTouchDegree = 0;  // remember for optimizing update calculation

// display content optimization
int lastDisplayTouchDegree = 0; // only redraw, if display resolution affected
int lastCalibState = false;
boolean screenDirty = true; // if false, display paint is omitted

// Set the LCD address  for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

/**
 * SETUP
 */
void setup() {

#ifdef DEBUG_MODE
  // for debugging
  Serial.begin(9600);
#endif

  // LCD init
  lcd.init();
  lcd.backlight();
  prefillDisplay();

  //noInterrupts();                                                           // Disable interrupts while attaching pins to ISRs

  // Hall sensor for RPM measurement
  pinMode(HALL_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), updateHallSensor, FALLING);

  ITimer5.init();
  Serial.println(BOARD_TYPE);
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));

  if (ITimer5.attachInterruptInterval(TIMER_INTERVAL_MS, updateHallSensor))
  {
    Serial.print(F("Starting  ITimer5 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer5. Select another freq. or timer"));

  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), detectRotation, FALLING);

  // Encoder
  pinMode(IN_ROTENC_PIN_A, INPUT_PULLUP);      // Define these as digital inputs
  pinMode(IN_ROTENC_PIN_B, INPUT_PULLUP);
  pinMode(IN_ROTENC_PIN_C, INPUT_PULLUP);
  digitalWrite (IN_ROTENC_PIN_C, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IN_ROTENC_PIN_A), doA, CHANGE);     // Interrupt 0: react to RISING/FALLING on pinA
  attachInterrupt(digitalPinToInterrupt(IN_ROTENC_PIN_B), doB, CHANGE);     // Interrupt 1: react to RISING/FALLING on pinB
  interrupts();                                                             // Enabled by default, but just in case

  // Touch Sensor
  pinMode ( IN_TOUCH_SENSOR, INPUT);
  digitalWrite ( IN_TOUCH_SENSOR, INPUT_PULLUP);
  pinMode ( OUT_DEPTH_OF_CUT_LVL1, OUTPUT);
  pinMode ( OUT_DEPTH_OF_CUT_LVL2, OUTPUT);
  pinMode ( OUT_DEPTH_OF_CUT_LVL3, OUTPUT);

}

/**
 * LOOP
 */
void loop() {

  // Touch Sensor indication update
  updateTouchDegree();
  updateTouchIndication();

  // check calibration active
  if (!digitalRead(IN_ROTENC_PIN_C) == HIGH)
    setCalibState(true);

  // Display
  updateDisplay();

  // Print RPMs on LCD
  lcd.setCursor(0, 1);
  lcd.print("RPM: ");
  lcd.print(RPM);

#ifdef DEBUG_MODE
  //Serial.write("A: "); Serial.print(Aset); Serial.write(" B: "); Serial.print(Bset);
  //Serial.write(" C: "); Serial.print (newCal);
  //Serial.write(" - ");
  Serial.print(encPos);
  Serial.write(" / ");
  Serial.print(getAngle());
  Serial.write(" -- ");
  Serial.println(curTouchDegree);
#endif
}

/**
 * Initial content of the LCD
 */
void prefillDisplay() {
  lcd.print("Angle: ");
  lcd.setCursor(0, 3); lcd.print("--------------------");
}

/**
 * Update display
 * Note: Aim is to update only these characters on the display that really
 *       changed, as this operations is a very expensive one
 */
void updateDisplay() {
  if (screenDirty) {
    // update display
    lcd.setCursor(7, 0);
    float angle = getAngle();
    if (angle < 100)
      lcd.print(angle < 10 ? "  " : " ");
    lcd.print(getAngle()); lcd.print(char(223));

    if(lastCalibState != isCalibState) {
      lastCalibState = isCalibState;
      lcd.setCursor(0, 0); lcd.print(isCalibState ? "    CALIBRATION     ":"Angle:               ");
    }

#ifdef DEBUG_MODE
    lcd.setCursor(0, 2); lcd.print("DBG Steps: "); lcd.print(encPos);
#endif

    // adapt bar according to lastTouchDegree to minimize display changes
    int curDisplayTouchDegree = curTouchDegree / (RINGBUF_SIZE / 20);
    //Serial.print(curTouchDegree);
    if (curDisplayTouchDegree != lastDisplayTouchDegree) {
      int diff = curDisplayTouchDegree - lastDisplayTouchDegree;
      lcd.setCursor(min(lastDisplayTouchDegree, curDisplayTouchDegree), 3);
      for (int i = 0; i < abs(diff); i++)
        lcd.print(diff > 0 ? char(255) : char(45));
      lastDisplayTouchDegree = curDisplayTouchDegree;
    }

    screenDirty = false;
  }
}

/**
 * Control 3 LEDs with different degrees of closeness to target degree
 */
void updateTouchIndication() {
  digitalWrite(OUT_DEPTH_OF_CUT_LVL1, curTouchDegree > ledsThresholds[0]);
  digitalWrite(OUT_DEPTH_OF_CUT_LVL2, curTouchDegree > ledsThresholds[1]);
  digitalWrite(OUT_DEPTH_OF_CUT_LVL3, curTouchDegree > ledsThresholds[2]);
}

/**
   Return how close the head is touching the target angle. Therefore a ring buffer with RINGBUG_SIZE entries
   is round-robin wise filled with the binary indication if the head touches the angle limit, or not.
   With a spinning lap it will always be the case that the signal will be jumping between touch and no touch.
   By counting the touch results in the ring buffer the degree how close we are to a constant "touch" is a
   value between 0 and RINGBUG_SIZE, where RINGBUG_SIZE means that the last RINGBUG_SIZE samples indicated a touch.
*/
void updateTouchDegree() {
  boolean isTouched = !digitalRead(IN_TOUCH_SENSOR);

  boolean oldValue = touchBuf[bufIndex];

  touchBuf[bufIndex] = isTouched;
  bufIndex ++;
  bufIndex %= RINGBUF_SIZE;
//Serial.print(curTouchDegree); Serial.write("\n");
  // incremental calculation of new state
  lastTouchDegree = curTouchDegree; // remember for display dirty indication
  curTouchDegree += (oldValue == isTouched ? 0 : (oldValue == true ? -1 : +1));

  if (lastTouchDegree != curTouchDegree)
    screenDirty = true;
}

void detectRotation(void)
{
  activeState = true;
}

void updateHallSensor() {
    if ( activeState )
  {
    // Reset to prepare for next round of interrupt
    activeState = false;

    if (debounceCounter >= DEBOUNCING_INTERVAL_MS / TIMER_INTERVAL_MS )
    {

      //min time between pulses has passed
      RPM = (float) ( 60000.0f / ( rotationTime * TIMER_INTERVAL_MS ) );

      avgRPM = ( 2 * avgRPM + RPM) / 3,


#ifdef DEBUG_MODE
      Serial.print("RPM = "); Serial.print(avgRPM);
      Serial.print(", rotationTime ms = "); Serial.println(rotationTime * TIMER_INTERVAL_MS);
#endif

      rotationTime = 0;
      debounceCounter = 0;
    }
    else
      debounceCounter++;
  }
  else
  {
    debounceCounter++;
  }

  if (rotationTime >= 5000)
  {
    // If idle, set RPM to 0, don't increase rotationTime
    RPM = 0;

#ifdef DEBUG_MODE
    Serial.print("RPM = "); Serial.print(RPM); Serial.print(", rotationTime = "); Serial.println(rotationTime);
#endif

    rotationTime = 0;
  }
  else
  {
    rotationTime++;
  }
}
/**
   Return the angle according to the current step position
*/
float getAngle() {

  return (encPos-enc0) * degPerStep;
}

void setCalibState(boolean c) {
  if(isCalibState != c) {
    screenDirty = true;
    isCalibState = c;
    if(isCalibState)
      encPos = encCal;
  }
}


/**
 * Interrupt call for A
 */
void doA() {     // CHANGE transition on pinA - either upgoing or downgoing

  if (!digitalRead(IN_ROTENC_PIN_C) == HIGH) {  // Check for index pulse
    Aset = false;                   // Reset these to have the same starting condition
    Bset = false;
    setCalibState(true);              // A new calibration has happened - Set flag
    encPos = encCal;                  // Reset encoder counts to reference value
#ifdef DEBUG_MODE
  Serial.write("CALIBRATE - A");
#endif
  } else {                              // No Index. Change on A - check which type and react
    setCalibState(false);
    if (digitalRead(IN_ROTENC_PIN_A) == HIGH) {  // UP going transition on A
      Aset = true;                    // We just had an upgoing transition
      encPos += (Bset ? 1 : -1);
    } else {                            // DOWN going transition on A
      Aset = false;                   // We just had a falling transition
      encPos += (Bset ? -1 : 1);
    }
  }
  screenDirty = true;
}

/**
 * Interrupt call for B
 */
void doB() {     // CHANGE transition on pinB

  if (!digitalRead(IN_ROTENC_PIN_C) == HIGH) {  // Check for index bit
    Aset = false;                   // Reset these to the same starting condition
    Bset = false;
    setCalibState(true);                  // Set flag
    encPos = encCal;                  // Reset
#ifdef DEBUG_MODE
  Serial.write("CALIBRATE - B");
#endif
  } else {                              // Change on B - check which type and react
    setCalibState(false);
    if (digitalRead(IN_ROTENC_PIN_B) == HIGH) {  // UP going transition
      Bset = true;                    // We just had an upgoing transition
      encPos += (Aset ? -1 : 1);
    } else {                            // DOWN going transition
      Bset = false;                   // We just had a falling transition
      encPos += (Aset ? 1 : -1);
    }
  }
  screenDirty = true;
}
