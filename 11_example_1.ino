#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// Configurable parameters for sonar
#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define _DIST_MIN 180.0
#define _DIST_MAX 360.0

#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)

#define _EMA_ALPHA 0.3

// Original target distances
#define _TARGET_LOW  250.0
#define _TARGET_HIGH 290.0

// Measured servo duty values (keep as measured)
#define _DUTY_MIN 0
#define _DUTY_NEU 1500
#define _DUTY_MAX 3000

// Global variables
float dist_ema = _DIST_MIN;
float dist_prev = _DIST_MIN;
unsigned long last_sampling_time = 0;

Servo myservo;

void setup() {
  // Initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED, HIGH);  // LED OFF (active-low)

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  // Initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_filtered;

  // Wait until next sampling time
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // Measure distance
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // Range filter: accept only 180~360 mm
  bool valid = (dist_raw >= _DIST_MIN &&
                dist_raw <= _DIST_MAX);

  if (valid) {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, LOW);   // LED ON
  } else {
    dist_filtered = dist_prev;
    digitalWrite(PIN_LED, HIGH);  // LED OFF
  }

  // EMA filter
  dist_ema = _EMA_ALPHA * dist_filtered
           + (1.0 - _EMA_ALPHA) * dist_ema;

  // Convert distance to angle
  // 180 mm -> 0 degrees, 270 mm -> 90 degrees,
  // 360 mm -> 180 degrees
  float angle = (dist_ema - _DIST_MIN)
              * 180.0 / (_DIST_MAX - _DIST_MIN);

  angle = constrain(angle, 0.0, 180.0);

  // Convert angle to servo pulse width
  int duty = _DUTY_MIN
           + (int)(angle / 180.0
           * (_DUTY_MAX - _DUTY_MIN));

  myservo.writeMicroseconds(duty);

  // Serial Plotter output
  Serial.print("Min:");
  Serial.print(_DIST_MIN);
  Serial.print(",dist:");
  Serial.print(dist_raw);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",Servo:");
  Serial.print(angle);
  Serial.print(",Max:");
  Serial.println(_DIST_MAX);

  // Update sampling time
  last_sampling_time += INTERVAL;
}

// Get distance reading from USS (mm)
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
