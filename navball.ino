#include <Arduino.h>
#include <math.h>

#define MOT_A1_PIN 6 // swap the order here if motor turn in reverse
#define MOT_A2_PIN 5
#define MOT_B1_PIN 10
#define MOT_B2_PIN 11
#define MOT_C1_PIN 3
#define MOT_C2_PIN 9

struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float heading, pitch, roll;
};

Quaternion navballCurrentAttitude;

float degreesToRadians(float degrees) {
    return degrees * (PI / 180.0);
}

// WARNING, USE RADIAN AND NOTE DEGREE !!!
Quaternion eulerToQuaternion(float heading, float pitch, float roll) {
    Quaternion q;

    float cy = cos(heading * 0.5);
    float sy = sin(heading * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

EulerAngles quaternionToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // Heading (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.heading = atan2(siny_cosp, cosy_cosp);

    // Convert radians to degrees
    angles.heading = angles.heading * 180.0 / M_PI;
    angles.pitch = angles.pitch * 180.0 / M_PI;
    angles.roll = angles.roll * 180.0 / M_PI;

    return angles;
}

void normalize(Quaternion& q) {
    float n = q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
    q.w /= n;
    q.x /= n;
    q.y /= n;
    q.z /= n;
}

Quaternion rotation_between_quaternions(const Quaternion& q1, const Quaternion& q2) {
    // Calculate the rotation quaternion between two quaternions q1 and q2
    Quaternion ret;
    ret.w = 1 + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    ret.x = q1.y * q2.z - q1.z * q2.y;
    ret.y = q1.z * q2.x - q1.x * q2.z;
    ret.z = q1.x * q2.y - q1.y * q2.x;
    normalize(ret);
    return ret;
}

// Function to project a quaternion as the sum of three static quaternions
// and return the coefficients for each quaternion
void projectQuaternion(Quaternion q, float &coeffA, float &coeffB, float &coeffC) {
    // Define the three static quaternions
    Quaternion A = {0.46193975, 0.1913417, 0.33141354456, 0.80010311354}; // roue A
    Quaternion B = {0.46193975, 0.1913417, -0.33141354456, -0.80010311354}; // roue B
    Quaternion C = {0.9238795, 0.3826834, 0, 0}; // roue C

    // Compute the projection coefficients
    float dotA = q.w * A.w + q.x * A.x + q.y * A.y + q.z * A.z;
    float dotB = q.w * B.w + q.x * B.x + q.y * B.y + q.z * B.z;
    float dotC = q.w * C.w + q.x * C.x + q.y * C.y + q.z * C.z;

    // Assign coefficients
    coeffA = dotA;
    coeffB = dotB;
    coeffC = dotC;
}



void set_motor_pwm(int pwm, char motor)
{
  switch(motor) {
    case 'A':
      if (pwm < 0) {  // reverse speeds
        analogWrite(MOT_A1_PIN, -pwm);
        digitalWrite(MOT_A2_PIN, LOW);
      } else { // stop or forward
        digitalWrite(MOT_A1_PIN, LOW);
        analogWrite(MOT_A2_PIN, pwm);
      }
      break;
    case 'B':
      if (pwm < 0) {  // reverse speeds
        analogWrite(MOT_B1_PIN, -pwm);
        digitalWrite(MOT_B2_PIN, LOW);
      } else { // stop or forward
        digitalWrite(MOT_B1_PIN, LOW);
        analogWrite(MOT_B2_PIN, pwm);
      }
      break;
    case 'C':
      if (pwm < 0) {  // reverse speeds
        analogWrite(MOT_C1_PIN, -pwm);
        digitalWrite(MOT_C2_PIN, LOW);
      } else { // stop or forward
        digitalWrite(MOT_C1_PIN, LOW);
        analogWrite(MOT_C2_PIN, pwm);
      }
      break;
  }
}

void stop_all_motor()
{
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);
  digitalWrite(MOT_C1_PIN, LOW);
  digitalWrite(MOT_C2_PIN, LOW);
}

void setup() {
  Serial.begin(9600);

  // Set all the motor control inputs to OUTPUT
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);
  pinMode(MOT_C1_PIN, OUTPUT);
  pinMode(MOT_C2_PIN, OUTPUT);
  // Turn off motors - Initial state
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);
  digitalWrite(MOT_C1_PIN, LOW);
  digitalWrite(MOT_C2_PIN, LOW);
}

void loop() {
  // we get the current attitude
  EulerAngles atitudeDEGREE;
  atitudeDEGREE.heading = 100;
  atitudeDEGREE.pitch = 0;
  atitudeDEGREE.roll = 0;
  // we convert it from degree to radian
  EulerAngles atitudeRADIAN;
  atitudeRADIAN.heading = degreesToRadians(atitudeDEGREE.heading);
  atitudeRADIAN.pitch = degreesToRadians(atitudeDEGREE.pitch);
  atitudeRADIAN.roll = degreesToRadians(atitudeDEGREE.roll);
  // we make it a quaternion
  Quaternion q = eulerToQuaternion(atitudeRADIAN.heading, atitudeRADIAN.pitch, atitudeRADIAN.roll);
  // we compute the difference with the current navball attitude
  Quaternion toRotate = rotation_between_quaternions(q, navballCurrentAttitude);
  
  float coeffA, coeffB, coeffC;
  projectQuaternion(toRotate, coeffA, coeffB, coeffC);
  set_motor_pwm(coeffA, 'A');
  set_motor_pwm(coeffB, 'B');
  set_motor_pwm(coeffC, 'C');
  // wait a set amount of time and stop.
  delay(2000);
  stop_all_motor();
  navballCurrentAttitude = q;
}
