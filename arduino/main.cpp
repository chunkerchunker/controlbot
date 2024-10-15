// This program drives the test robot around randomly while collect training
// data for an ML model onto an SD card.  The final training data file is in
// commas-separated-values (CSV) format, where each line is like this:
//
//   left wheel rotation, right wheel rotation, change in heading
//
// The program has four logical components together with a main program that
// uses those components.  The four components are:
//
//   1.  Code to access the compass, which tracks the robot's heading.
//   2.  Code that uses motor encoders to track wheel rotation.
//   3.  Code to write training data gathered from the compass and
//       motor encoders to the SD card.
//   4.  An "autodrive" system to drive the robot around randomly.
//
// Below, there is one block of code for each of these four components.
// The main program appears after these four code blocks.
//
// Each component has an associated setup function: setupCompass(),
// setupEncoders(), etc.  All of these setup functions are called during
// the Arudino's main setup() procedure when the robot starts up.  Beyond
// that commonality, the four components are rather different.

////////////////////////////////////////////////////////////////////////////////
//
//  Compass
//
// The test robot has a QMC5883L compass that can approximately determine the
// robot's orientation by measuring the earth's magnetic field.  We'll use a
// pre-written library to simplify interaction with the compass:
//
//   https://github.com/mprograms/QMC5883LCompass

// Access the pre-written compass library.
#include <QMC5883LCompass.h>

// Create a compass object, through which we can access the compass.
QMC5883LCompass compass;

// Prepare the compass for use.  The main job here is to supply the compass
// object with a description of the local magnetic field, so that it can
// more accurately determine its orientation.  These numbers were
// generated with a separate calibration script.
void setupCompass() {
  compass.init();
  compass.setCalibrationOffsets(-125.00, -223.00, -661.00);
  compass.setCalibrationScales(1.01, 0.93, 1.06);
}

// Return the compass heading in radians.  There are 2 pi radians in a circle,
// as opposed to 360 degrees.  Degrees are nice for humans because 360 is
// readily divisible, but radians are cleaner for many mathematical uses.
float compassHeading() {
  // Import fresh data from the compass onto the Arduino.
  compass.read();

  // Unlike a regular compass, which returns a simple direction (like "a bit
  // north of northwest" or whatever), the QMC5883L measures magnetic field
  // strength in three perpendicular directions, called X, Y, and Z.  We can
  // compute the robot's heading from the field strength in the X and Y
  // directions.  We do not need the Z (up-and-down) field strength.
  //
  // The atan2f function converts these two field strengths into a direction
  // in radians.  In general atan2f(y, x) is the orientation of a line
  // from the origin to the point (x, y) on a coordinate plane.
  return atan2f(compass.getY(), compass.getX());
}

////////////////////////////////////////////////////////////////////////////////
//
//  Motor Encoders
//
// The two motors on the test robot have "quadrature encoders", sensors that
// tell is roughly how far the motors (and wheels connected directly to
// them!) have turned.
//
// Each encoder constantly emits two signals, called A and B.  Each signal
// is either "low" or "high" at any given instant.  As the motor turns
// forward, each signal pulses: low, high, low, high, low, etc.  However,
// the two signals are shifted relative to one another:
//
//         +-----+     +-----+     +-----+     +---
// A:      |     |     |     |     |     |     |
//       --+     +-----+     +-----+     +-----+
//
//       -----+     +-----+     +-----+     +-----+
// B:         |     |     |     |     |     |     |
//            +-----+     +-----+     +-----+     +
//
// Notice that whenever the A signal goes from low to high (a "rising edge"),
// the B signal is high.  In contrast, suppose we run the motor in reverse.
// To see how signals A and B behave in that situation, run your eyes backward
// across the chart above, from right to left.  Notice that whenever A rises,
// the B signal is now low (rather than high).
//
// In summary, when the A signal rises:
//   - If B is low, the motor moved forward 1 step.
//   - If B is high, the motor moved backward 1 step.
//
// So we can track a motor's position over time by adding or subtracting 1
// step whenever signal A rises, depending on the value of signal B.  That
// is what the code below accomplishes.
//
// We're using the Arduino's "interrupt" mechanism.  This causes a function
// to be called whenever signal A rises.  Specifically, either
// interruptLeft() or interruptRight() is called, depending on which
// motor's A signal rose.  These functions update the motor's estimated
// position based on the B signal, as described above.

// These are the Arduino pins wired to the A and B encoder signals from
// the left and right motors.
const int LEFT_ENCODER_A = 3;
const int LEFT_ENCODER_B = 5;
const int RIGHT_ENCODER_A = 2;
const int RIGHT_ENCODER_B = 4;

// These are estimated positions of the left and right motors.  They are
// "volatile", because their value could change any time due to an interrupt.
// This forces this compiler to treat these variables with extra care.
volatile int volatile_left_steps = 0;
volatile int volatile_right_steps = 0;

// The two functions below update motor positions when an A signal rises
// based on the value of the B signal.
void interruptLeft() {
  volatile_left_steps += 2 * digitalRead(LEFT_ENCODER_B) - 1;
}

void interruptRight() {
  volatile_right_steps -= 2 * digitalRead(RIGHT_ENCODER_B) - 1;
}

// Tell the Arduino to call one of the functions above when the A signal
// for one of the motors rises.
void setupEncoders() {
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), interruptRight, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), interruptLeft, RISING);
}

////////////////////////////////////////////////////////////////////////////////
//
//  SD Card
//
// As the test robot drives around, we'll write data to an SD card.  Later,
// we'll move this card from the test robot to a laptop and use that data
// to train an ML model.  We'll interact with the compass through
// pre-written libraries.

#include <SPI.h>
#include <SD.h>

// This object represents the file that we're writing to the SD card.
File file;

void setupSD() {
  const int CHIP_SELECT_PIN = 10;
  SD.begin(CHIP_SELECT_PIN);

  // Remove the old data file, called "tele.csv" as a shorthand for
  // "telemery.csv", because long file names are not allowed.
  SD.remove("tele.csv");

  // Associate the "file" object with the file we're writing on the SD card.
  file = SD.open("tele.csv", FILE_WRITE);
}

// Write one line to the tele.csv file on the SD card.  This line has the
// form:
//
//   left wheel steps, right wheel steps, heading change in radians
//
// An example line might be:  8,12,0.071
void writeSD(int left_change, int right_change, float heading_change) {
  // Set aside 32 bytes of memory to contain the text of the line.
  char buffer[32];

  // Sprintf stands for "string print formatted".  This is an ancient,
  // cryptic, and useful function to create a character string containing
  // variable values.
  sprintf(buffer, "%d,%d,%f\n", left_change, right_change, heading_change);

  // Write the character string to the SD card, following the instructions
  // for the SD card library.
  file.write((byte *) &buffer, strlen(buffer));
  file.flush();
}

////////////////////////////////////////////////////////////////////////////////
//
//  Autodrive
//
// This code drives the robot around varying the speeds of the wheels
// according to a random plan.  Calling autodrive() updates the wheel
// speed to match the plan, so this should be done often.
//
// The speed of a wheel is set with "pulse width modulation".  The idea is
// that, rather than providing full power to the corresponding motor, we
// rapidly pulse the power ON an OFF.  If the power is ON most of the time,
// the motor runs fast.  If the power is OFF a lot of the time, the motor
// runs slow.  If the power is OFF most of the time, then the motor might
// small, heat up, melt down, and reduce KLS to smoking slag.
//
// The Arduino (and FRC robots!) have built-in PWM.  All we have to do
// to specify a motor speed is to set the pulse-width to the value between
// 128 and 255.  (We'll avoid lower values; see note above about slag.)
//
// At every moment, we'll maintain target motor speeds for some time in the
// past and target motor speeds for another time in the future.  We'll set
// the motor speed in the present by interpolating between these past and
// future targets.  As time marches on, the future target becomes the past
// target and we generate new, future target motor speeds at random.

const int LEFT_DRIVE_PIN = 6;
const int RIGHT_DRIVE_PIN = 9;

long start_time;
int start_left;
int start_right;

long end_time;
int end_left;
int end_right;

void setupAutodrive() {
  start_time = millis();
  start_left = 128;
  start_right = 128;

  end_time = millis() + 100;
  end_left = start_left;
  end_right = start_right;
}

void autodrive() {
  // Get the current time in milliseconds.
  long time = millis();

  // If the current time is past our future motor speed targets, then
  // make the future targets into past targets and generate new future
  // targets.
  if (time > end_time) {
    start_time = end_time;
    start_left = end_left;
    start_right = end_right;

    end_time = time + random() * 1000 + 1000;
    end_left = random() * 128 + 128;
    end_right = random() * 128 + 128;
  }

  // Current motor speeds are interpolated from past and future targets.
  int left_pwm = map(time, start_time, end_time, start_left, end_left);
  int right_pwm = map(time, start_time, end_time, start_right, end_right);

  // Ensure there's no mistake:  motor speeds must stay between 128 and 255.
  constrain(left_pwm, 128, 255);
  constrain(right_pwm, 128, 255);

  // Tell the Arduino to pulse the pins controlling the motors as the
  // desired speeds.
  analogWrite(LEFT_DRIVE_PIN, left_pwm);
  analogWrite(RIGHT_DRIVE_PIN, right_pwm);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Main Program
//

void setup() {
  // Enable printing text on a laptop screen.
  Serial.begin(9600);

  // Drive system
  pinMode(LEFT_DRIVE_PIN, OUTPUT);
  pinMode(RIGHT_DRIVE_PIN, OUTPUT);

  setupCompass();
  setupSD();
  setupEncoders();
  setupAutodrive();
}

float previous_heading = 0;
long previous_left_steps = 0;
long previous_right_steps = 0;

void loop() {
  // Compute the change in compass heading since the last time through
  // this loop.
  float heading = compassHeading();
  float heading_change = heading - previous_heading;
  previous_heading = heading;

  // If the robot appears to have turned more than 180 degrees in one
  // direction, assume it turned less than 180 in the opposite direction.
  if (heading_change > PI) {
    heading_change -= 2 * PI;
  }

  if (heading_change < - PI) {
    heading_change += 2 * PI;
  }

  // Compute the change in left and right wheel steps since the last time
  // through this loop.  Turn off interrupts while reading the current
  // step counts so they can't change while we're reading them.
  noInterrupts();
  long left_steps = volatile_left_steps;
  long right_steps = volatile_right_steps;
  interrupts();

  long left_change = left_steps - previous_left_steps;
  long right_change = right_steps - previous_right_steps;

  previous_left_steps = left_steps;
  previous_right_steps = right_steps;

  // Store the changes in wheel position and changes in compass heading to
  // the SD card in CSV (comma separated values) format.
  writeSD(left_change, right_change, heading_change);

  // Update the robot's random driving pattern.
  autodrive();

  // Wait a while (1/20th second) before collecting more data.
  delay(50);
}
