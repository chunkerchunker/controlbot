#include <QMC5883LCompass.h>
#include <SD.h>
#include <SPI.h>

typedef unsigned long time_t;

const int SD_CS_PIN = 10;  // Connected to SD card

// Control wheel motors
const int LEFT_FORWARD = 6;
const int LEFT_BACKWARD = 7;
const int RIGHT_FORWARD = 9;
const int RIGHT_BACKWARD = 8;

// Encoder outputs.  Pins 2 and 3 generate interrupts.
const int MOTOR_ENC_LEFT_A = 3;
const int MOTOR_ENC_LEFT_B = 5;
const int MOTOR_ENC_RIGHT_A = 2;
const int MOTOR_ENC_RIGHT_B = 4;

// Compass object
QMC5883LCompass compass;

// These record wheel travel distances
volatile long right_pos = 0;
volatile long left_pos = 0;

void interruptLeft() {
  left_pos += 2 * digitalRead(MOTOR_ENC_LEFT_B) - 1;
}

void interruptRight() {
  right_pos -= 2 * digitalRead(MOTOR_ENC_RIGHT_B) - 1;
}

// The robot picks random target speeds for the left and right wheels
// and a random time to achieve those speed.  Speeds are adjusted
// linearly to reach those target speeds by the target time.
int prev_left_speed;
int prev_right_speed;
time_t prev_time;

int next_left_speed;
int next_right_speed;
time_t next_time;

// Telemetry file on the SD card
File sdfile;

const char* telemetry_file_name = "tele.bin";

// Set left and right motor speeds to random values for a random time
void setDriveGoal() {
  next_left_speed = random(128, 256);
  next_right_speed = random(128, 256);
  next_time = prev_time + random(500, 3000);
}

// Copies of this struct are written to the SD card
struct {
  time_t time;     // 4 bytes
  float heading;   // 4 bytes
  long left_pos;   // 4 bytes
  long right_pos;  // 4 bytes
  int zval;        // 2 bytes
} buffer;

void setup() {
  Serial.begin(9600);

  // Drive system
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);

  // Odometry
  pinMode(MOTOR_ENC_RIGHT_A, INPUT);
  pinMode(MOTOR_ENC_RIGHT_B, INPUT);
  pinMode(MOTOR_ENC_LEFT_A, INPUT);
  pinMode(MOTOR_ENC_LEFT_B, INPUT);

  // Trigger interrupts when encoders detect wheel movement
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENC_RIGHT_A), interruptRight,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENC_LEFT_A), interruptLeft,
                  RISING);

  // Set up compass
  compass.init();
  compass.setCalibrationOffsets(-125.00, -223.00, -661.00);
  compass.setCalibrationScales(1.01, 0.93, 1.06);

  // Set up SD card.  Overwrite data.  Do not append.
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD.begin() fail");
    while (1)
      ;
  }
  SD.remove(telemetry_file_name);
  sdfile = SD.open(telemetry_file_name, FILE_WRITE);
  if (!sdfile) {
    Serial.println("SD.open() fail");
    while (1)
      ;
  }

  // Initialize autonomous driving
  prev_left_speed = 128;
  prev_right_speed = 128;
  prev_time = millis();
  setDriveGoal();

  // The first datapoint begins 50 milliseconds from now.
  buffer.time = millis();
}

void loop() {
  // Start the loop on 50 millisecond intervals
  unsigned long now;
  do {
    now = millis();
  } while (now < buffer.time + 50);
  buffer.time = now;

  // We'll do five compass readings.  The compass should update at
  // 200Hz or every 5 milliseconds.  We'll separate the compass
  // readings by 6 milliseconds so they're independent.
  compass.read();
  int x1 = compass.getX(), y1 = compass.getY();

  delay(6);

  compass.read();
  int x2 = compass.getX(), y2 = compass.getY();

  delay(6);

  // Grab wheel data.  Disable interrupts so the variables
  // can't change while we're reading them.
  noInterrupts();
  buffer.left_pos = left_pos;
  buffer.right_pos = right_pos;
  interrupts();

  compass.read();
  int x3 = compass.getX(), y3 = compass.getY();
  // Record the vertical component of the compass to
  // help detect when the robot was picked up.
  buffer.zval = compass.getZ();

  delay(6);
  compass.read();
  int x4 = compass.getX(), y4 = compass.getY();
  delay(6);
  compass.read();
  int x5 = compass.getX(), y5 = compass.getY();

  // Combine the compass readings and get a heading.
  float x = float(x1 + x2 + x3 + x4 + x5) / 5;
  float y = float(y1 + y2 + y3 + y4 + y5) / 5;
  buffer.heading = atan2f(y, x);

  // Write to the SD card.  Flushing takes time, but better
  // to flush often and keep updates regular.
  sdfile.write((byte*)&buffer, sizeof(buffer));
  sdfile.flush();

  // If we've completed a stage of the drive plan, make a new one.
  if (now > next_time) {
    prev_left_speed = next_left_speed;
    prev_right_speed = next_right_speed;
    prev_time = next_time;
    setDriveGoal();
  }

  // s = fraction of the way through this drive stage
  float s = float(now - prev_time) / (next_time - prev_time);
  int left_speed = s * next_left_speed + (1 - s) * prev_left_speed;
  int right_speed = s * next_right_speed + (1 - s) * prev_right_speed;

  left_speed = min(255, max(128, left_speed));
  right_speed = min(255, max(128, right_speed));

  analogWrite(LEFT_FORWARD, left_speed);
  analogWrite(RIGHT_FORWARD, right_speed);
}
