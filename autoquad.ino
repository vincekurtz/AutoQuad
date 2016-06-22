/*
 *   autoquad.ino
 *
 * Code for the AutoQuad project, an autonomous obstacle avoidance system for
 * a custom built quadcopter using arduino and sonar.
 *
 *
 * This code does three main things:
 *       - Passes input directly from the reciever to the controller ('manual mode')
 *       - Gets readings from rangefinders and uses them to help the craft avoid
 *         obstacles ('autonomous mode')
 *       - Provides a method for switching between modes.
 *
 * Isaiah Breckbill and Vince Kurtz and Peter Wise
 *
 */

// ---------------------------- Setup -------------------------------------

// =============== Libraries =============================
#include <digitalWriteFast.h>  // Simple library to read and write fast. Pretty self-explanitory.
#include <PID_v1.h>         // PID stuff for altitude hold.
#include <Servo.h>          // Servo library to write outputs to the controller.

// =======================================================

// ========= Input-Output Global Variables ===============
  // These variables are used in determining what commands are being passed 
  // to the kk2 controller. aPulse, ePulse, tPulse, and rPulse are especially
  // important. They store the pulse lengths that control roll, pitch, thrust,
  // and yaw. The final_x variables are used to pass pulses to the controller.
  // The xPulse variables are not used, because there is a risk of them getting
  // overwritten by ISRs before any changes can be made to them.

// Input pins
int aIn = 6;  // Aileron (roll)
int eIn = 7;  // Elevator (pitch)
int tIn = 8;  // Throttle
int rIn = 9;  // Rudder (yaw)
int mIn = 4;  // Mode switcher on channel 5

// Output pins
int aOut = 10;
int eOut = 11;
int tOut = 12;
int rOut = 13; 

// Variables to use in calculating pulses (xStart, xEnd) and storing pulse lengths (xPulse). 
// Must be volatile to be used in interrupts. unsigned because they will always be positive.
volatile unsigned int aStart, aEnd, aPulse;
volatile unsigned int eStart, eEnd, ePulse;
volatile unsigned int tStart, tEnd, tPulse;
volatile unsigned int rStart, rEnd, rPulse;
volatile unsigned int mStart, mEnd, mPulse;  // for the mode switching channel

// These variables are used to pass pulse lengths to the controller. They can be based
// on the xPulse variables for manual mode, or independantly calculated for autonomous
// mode.
int final_a, final_e, final_t, final_r;
// =======================================================


// ============ Rangefinding Global Variables ============

// The trigger pin. 
int trig = 22;

// 8 response pins for 8 rangefinders
int resp1 = 23;
int resp2 = 25;
int resp3 = 27;
int resp4 = 29;
int resp5 = 31;
int resp6 = 33;
int resp7 = 35;
int resp8 = 37;

// Pulse measurement variables for all 8 rangefinders
volatile unsigned int pStart1, pEnd1, range1;
volatile unsigned int pStart2, pEnd2, range2;
volatile unsigned int pStart3, pEnd3, range3;
volatile unsigned int pStart4, pEnd4, range4;
volatile unsigned int pStart5, pEnd5, range5;
volatile unsigned int pStart6, pEnd6, range6;
volatile unsigned int pStart7, pEnd7, range7;
volatile unsigned int pStart8, pEnd8, range8;
// =======================================================


// =============== Mode Variables ========================
int auto_mode = 0;   // Initiate things in manual mode.
// =======================================================


// =============== Object Initialization =================
// Create servo objects for all 4 channels
Servo a_servo;
Servo e_servo;
Servo t_servo;
Servo r_servo;

// PID for holding altitude -----------------
// PID constants
float kP = 0.8;
float kI = 5;
float kD = 1;

double offset, pid_out;
double TARGET_ALT = 90;

// PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID altHoldPID(&offset, &pid_out, 0, kP, kI, kD, DIRECT);

// PID for maintaining distance from obstacles ---
// PID constants for the horizontal direction
float hKp = 1;
float hKi = 0;
float hKd = 8;

double CRIT_DIST = 100;   // Critical distance we want to keep the copter away from stuff

double f_corr, b_corr, l_corr, r_corr;   // front PID output is (x)_corr, a correction factor.
double f_range, b_range, l_range, r_range;  // Need an intermediate variable instead of range2 because of types.
                                            // rangeN variables are volatile ints, but these need to be doubles.

PID frontPID(&f_range, &f_corr, &CRIT_DIST, hKp, hKi, hKd, DIRECT);
PID backPID(&b_range, &b_corr, &CRIT_DIST, hKp, hKi, hKd, DIRECT);
PID leftPID(&l_range, &l_corr, &CRIT_DIST, hKp, hKi, hKd, DIRECT);
PID rightPID(&r_range, &r_corr, &CRIT_DIST, hKp, hKi, hKd, DIRECT);

// =======================================================


// ============== Standard Setup =========================
void setup() {
  // Start a serial stream so we can debug without a display shield.
  Serial.begin(9600);

  // Set up ISRs to take range readings.
  attachInterrupt(resp1, resp1_ISR, CHANGE);
  attachInterrupt(resp2, resp2_ISR, CHANGE);
  attachInterrupt(resp3, resp3_ISR, CHANGE);
  attachInterrupt(resp4, resp4_ISR, CHANGE);
  attachInterrupt(resp5, resp5_ISR, CHANGE);
  attachInterrupt(resp6, resp6_ISR, CHANGE);
  attachInterrupt(resp7, resp7_ISR, CHANGE);
  attachInterrupt(resp8, resp8_ISR, CHANGE);

  // Set up ISRs to read incoming control signals from the reciever.
  attachInterrupt(aIn, aileronISR, CHANGE);
  attachInterrupt(eIn, elevatorISR, CHANGE);
  attachInterrupt(tIn, throttleISR, CHANGE);
  attachInterrupt(rIn, rudderISR, CHANGE);
  attachInterrupt(mIn, modeISR, CHANGE);

  // Attach each output channel to the correct pin.
  a_servo.attach(aOut);
  e_servo.attach(eOut);
  t_servo.attach(tOut);
  r_servo.attach(rOut);

  // Create limits so PID loop gives results in the desired range.
  // This is more strict than the board can acually handle (~1200 to ~1800)
  // The default throttle is 1430
  altHoldPID.SetOutputLimits(-20,80);

  // Output limits for the horizontal PIDs
  frontPID.SetOutputLimits(-100,100);
  backPID.SetOutputLimits(-100,100);
  leftPID.SetOutputLimits(-100,100);
  rightPID.SetOutputLimits(-100,100);

  // Go through the PID calculations every 1 ms. We want this to be fast enough that 
  // it returns a different value in every loop.
  altHoldPID.SetSampleTime(1);
  frontPID.SetSampleTime(1);
  backPID.SetSampleTime(1);
  leftPID.SetSampleTime(1);
  rightPID.SetSampleTime(1);

  // Start everything up with the PID off so it doesn't run while it's sitting on the ground.
  altHoldPID.SetMode(MANUAL);
  // Start up the directional PIDS
  frontPID.SetMode(AUTOMATIC);
  backPID.SetMode(AUTOMATIC);
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);

  // LEDs for testing
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(21, OUTPUT);

  // Set the trigger pin HIGH. This will enable continuous measurements from
  // the rangefinders.
  digitalWrite(trig, HIGH);

  // Wait a bit to ensure everything is initialized
  delay(200);
}
// =======================================================


// ---------------------------- Main Code -------------------------------------

// ============== Standard Loop ==========================
  // The code that is run repeatedly. Keep in mind that this is not
  // the only thing going on: we also have a bunch of interrupt service
  // routines taking measurements in the "background".
void loop() {
  // Check what mode we're in.
  auto_mode = isAutomode();

  // For manual mode (and debugging in the development phase),
  // let's simply output the same pulses as we're getting from the reciever.
  final_a = aPulse;
  final_e = ePulse;
  final_t = tPulse;
  final_r = rPulse;

  // If it's autonomous mode, do something to modify final_x variables.
  if (auto_mode) {
    if (range1 > 40) {   // when the copter is above 40 cm from the ground
      // Don't bother trying to control direction while in ground effect, because
      // rangefinders will pick up all sorts of things.
      control_roll();
      control_pitch();
      control_yaw();
    }

    maintain_altitude();

    altHoldPID.SetMode(AUTOMATIC);  // Only calculate PID stuff when in auto mode

    // Light up a green LED to show we're in autonomous mode
    digitalWrite(21, HIGH);

  } else {
    altHoldPID.SetMode(MANUAL);  // don't calculate pid_out
    digitalWrite(21, LOW);  // turn the indicator off
  }

  // Light up an LED if the bottom ranger has a desirable range
  if ( range1 > TARGET_ALT - 10 && range1 < TARGET_ALT + 10 ) {
    digitalWriteFast(15, HIGH);
  } else {
    digitalWriteFast(15, LOW);
  }

  // Light up an LED if the side rangers find something within the critical distance
  lightLED(range2, 16);
  lightLED(range3, 17);
  lightLED(range4, 18);
  lightLED(range5, 19);

  Serial.print(final_a);
  Serial.print(" | ");
  Serial.print(final_e);
  Serial.print(" | ");
  Serial.print(final_t);
  Serial.print(" | ");
  Serial.println(final_r);  // correction value for the front

  // Output the xPulse variables to the controller, causing the quadcopter to 
  // behave in one way or another.
  writePulses(final_a, final_e, final_t, final_r);

}
// =======================================================


// ================ Autonomous Mode Functions ============
  // These functions modify the global variables final_a, final_e,
  // final_t, and final_r according to rangefinder readings.

void control_roll() {
  // LEFT PID CONTROL
  if (range5 < CRIT_DIST) {
    l_range = range5;    // l_range is input to PID loop
    leftPID.Compute();   // calculate a correction (l_corr)
  } else {
    l_corr = 0;   // no correction outside the critical distance
  }
  // RIGHT PID CONTROL
  if (range3 < CRIT_DIST) {
    r_range = range3;
    rightPID.Compute();  // updates the value of r_corr
  } else {
    r_corr = 0; 
  }
  final_a = final_a + l_corr - r_corr;
  //final_a = correct(final_a, range3, -1);
  //final_a = correct(final_a, range5, 1);
}

// Tilt the copter forward and back in response to objects in the field of
// view of Rangefinders 2 and 4.
void control_pitch() {
  // FRONT PID CONTROL
  if (range2 < CRIT_DIST) {
    f_range = range2;     // f_range is the input variable for the PID loop
    frontPID.Compute();
  } else {
    f_corr = 0;    // No correction factor for distances big enough.
  }
  // BACK PID CONTROL
  if (range4 < CRIT_DIST) {
    b_range = range4;    // input to PID loop
    backPID.Compute();
  } else {
    // Outside the danger zone, so don't apply a correction
    b_corr = 0;
  }
  final_e = final_e + f_corr - b_corr;
}

// Keep flying at a constant altitude
void maintain_altitude() {
  offset = range1 - TARGET_ALT;   // offset is the input to the PID loop. Center
                                  // this around zero.

  if ( tPulse < 1200 ) {   // Override the correction for low throttle values, in the
    return;                // hopes that this will prevent getting the throttle locked high.
  }

  // The PID loop takes into account range1 and the desired altitude, and updates pid_out
  //PID altHoldPID(&offset, &pid_out, 0, 3, 5, 1, DIRECT);
  altHoldPID.Compute();  
  final_t = pid_out + 1430;   
}

void control_yaw() {
  // The pulse width when perfectly centered of channel 4 (rudder/yaw) is 1502 us
  return;
}

// Apply a generic correction (tilt forward or back) to a generic channel (final_x)
// based on readings from any rangefinder. If direction is -1, the correction is 
// subtracted from the pulse. Otherwise if it is +1, the correction is added to
// the pulse.
int correct(int channel, int range, int direction) {
  float k=2;    // A constant to determine the strength of a response to an obstacle.
  int correction = (CRIT_DIST - range) * k;
  if (range < CRIT_DIST) { 
    // In the danger zone. Apply the correction.
    channel = channel + direction * correction;
    return channel;
  } else {
    // If there is nothing within range, simply return the same channel pulse
    // length.
    return channel;
  }
}
// =======================================================


// ================= RangeFinder ISRs ====================
  // Interupt Service Routines to time response pulses from each
  // of 8 rangefinders. 
void resp1_ISR() {
  if ( digitalReadFast(resp1) == HIGH ) {
    // This is the beginning of a pulse. Start counting...
    pStart1 = micros();
  } else {
    // This must be the end of a pulse. Stop the count and calculate range.
    pEnd1 = micros();
    range1 = ( pEnd1 - pStart1 ) / 58;  // Convert the length of the pulse in us to cm.
  }
}
void resp2_ISR() {
  if ( digitalReadFast(resp2) == HIGH ) {
    pStart2 = micros();
  } else {
    pEnd2 = micros();
    range2 = ( pEnd2 - pStart2 ) / 58;  
  }
}
void resp3_ISR() {
  if ( digitalReadFast(resp3) == HIGH ) {
    pStart3 = micros();
  } else {
    pEnd3 = micros();
    range3 = ( pEnd3 - pStart3 ) / 58; 
  }
}
void resp4_ISR() {
  if ( digitalReadFast(resp4) == HIGH ) {
    pStart4 = micros();
  } else {
    pEnd4 = micros();
    range4 = ( pEnd4 - pStart4 ) / 58;  
  }
}
void resp5_ISR() {
  if ( digitalReadFast(resp5) == HIGH ) {
    pStart5 = micros();
  } else {
    pEnd5 = micros();
    range5 = ( pEnd5 - pStart5 ) / 58; 
  }
}
void resp6_ISR() {
  if ( digitalReadFast(resp6) == HIGH ) {
    pStart6 = micros();
  } else {
    pEnd6 = micros();
    range6 = ( pEnd6 - pStart6 ) / 58;
  }
}
void resp7_ISR() {
  if ( digitalReadFast(resp7) == HIGH ) {
    pStart7 = micros();
  } else {
    pEnd7 = micros();
    range7 = ( pEnd7 - pStart7 ) / 58;  
  }
}
void resp8_ISR() {
  if ( digitalReadFast(resp8) == HIGH ) {
    pStart8 = micros();
  } else {
    pEnd8 = micros();
    range8 = ( pEnd8 - pStart8 ) / 58; 
  }
}
// ======================================================


// ================== Pulse Timing ISRs ==================
  // Interrupt Service Routines to time pulses on each channel.
  // These modify the global xPulse variables to store the length in 
  // microseconds of 'high' pulses from the reciever.

// Aileron ----
void aileronISR() {
  if ( digitalReadFast(aIn) == HIGH ) {
    // Got begining of pulse. Start the clock!
    aStart = micros();
  } else {
    // Got end of pulse. Calculate time.
    aEnd = micros();
    aPulse = aEnd - aStart;
  }
}
// Elevator ----
void elevatorISR() {
  if ( digitalReadFast(eIn) == HIGH ) {
    // Got begining of pulse. Start the clock!
    eStart = micros();
  } else {
    // Got end of pulse. Calculate time.
    eEnd = micros();
    ePulse = eEnd - eStart;
  }
}
// Throttle ----
void throttleISR() {
  if ( digitalReadFast(tIn) == HIGH ) {
    // Got begining of pulse. Start the clock!
    tStart = micros();
  } else {
    // Got end of pulse. Calculate time.
    tEnd = micros();
    tPulse = tEnd - tStart;
  }
}
// Rudder ----
void rudderISR() {
  if ( digitalReadFast(rIn) == HIGH ) {
    // Got begining of pulse. Start the clock!
    rStart = micros();
  } else {
    // Got end of pulse. Calculate time.
    rEnd = micros();
    rPulse = rEnd - rStart;
  }
}
// Mode (for auto/manual mode switching). This is on channel 5.
void modeISR() {
  if ( digitalReadFast(mIn) == HIGH ) {
    // Got begining of pulse. Start the clock!
    mStart = micros();
  } else {
    // Got end of pulse. Calculate time.
    mEnd = micros();
    mPulse = mEnd - mStart;
  }
}

// =======================================================


// ================= Other Functions =====================

// Check the signal we're getting on channel 5, and output an integer (1 or 0)
// that corresponds to that mode. Mode 0 is manual mode. Mode 1 is autonomous mode.
int isAutomode() {
  if (mPulse > 1200) {
    // The switch is probably on, so we're in autonomous mode.
    return 1;
  } else {
    // When pulses are shorter, the switch is off, so we're in manual mode.
    return 0;
  }
}

// Write pulses to the controller. Useing the servo library and PWM pins
// allows these pins to continuously output pulses, so we don't have to worry
// so much about the timing of the main loop.
void writePulses(int a_pulse, int e_pulse, int t_pulse, int r_pulse) {

  a_servo.writeMicroseconds(a_pulse);
  e_servo.writeMicroseconds(e_pulse);
  t_servo.writeMicroseconds(t_pulse);
  r_servo.writeMicroseconds(r_pulse);

}

// Light an LED on 'pin' when rangecm (a distance in cm) is below the critical value.
void lightLED(int rangecm, int pin) {
  if (rangecm < CRIT_DIST) {
    digitalWrite(pin, HIGH);
  } else {
    digitalWrite(pin, LOW);
  }
}

// A function that actually delays the correct number of microseconds. delayMicroseconds()
// introduces an unacceptable amount of wierd noise, at least on the Due.
void altDelayMicros(unsigned int waitUs) {
  // get the starting time
  unsigned int startUs = micros();
  unsigned int curTime = startUs;
  while ( curTime-startUs < waitUs ) {
    // just do nothing until the correct time is reached
    curTime = micros();
  }
}
// =======================================================
