#include <Arduino.h>


// GLOBALS
#define RUN 1
#define PPR 4560.0 
#define cm_PER_ROT 6.28
#define MAX_PWM 50  // THE MAX SPEED THE ENCODER CAN ~ACCURATELY READ AT
#define test_motor 0

int MOTORS[3][6] = {
  {7, 4, 48, 49, 0, 0}, // MOTOR 1, PHASE_PIN, EN_PIN, ENC-A, ENC-B, DIRECTION, NUM_PULSES
  {6, 3, 50, 51, 0, 0},
  {5, 2, 52, 53, 0, 0},
};


// CUSTOM FUNCTIONS
void run_motor(int motor_pin, int dir, int pwm_val) {
  // MOVES MOTOR IN DIRECTION
  analogWrite(MOTORS[motor_pin][1], pwm_val);
  digitalWrite(MOTORS[motor_pin][0], dir);

  MOTORS[motor_pin][4] = dir;
}

void record_pulses() {
  // RECORDS THE NUMBER OF PULSES FROM ENCODERS TO EACH MOTOR IN MOTORS
  // PULSES CAN THEN BE USED TO DETERMINE THE NUMBER OF ROTATIONS, DISTANCE, AND ANGLE
  static int last_encs[3] = {3, 3, 3};

  int encs[3] = {
    digitalRead(MOTORS[0][2]) + digitalRead(MOTORS[0][3]),
    digitalRead(MOTORS[1][2]) + digitalRead(MOTORS[1][3]),
    digitalRead(MOTORS[2][2]) + digitalRead(MOTORS[2][3])
    };
  
  for (int i=0; i<3; i++) {
    int last_enc = last_encs[i];
    int enc = encs[i];

    if (enc != last_enc) {
      MOTORS[i][5] += 1 - (2 * MOTORS[i][4]); // left rotation decrements, right increments pulses
      last_encs[i] = enc;
    };
  }

  Serial.println();
}

float get_motor_rotations(int motor_pin) {
  // RETURNS THE NUMBER OF REVOLUTIONS FROM THE ORIGIN 
  return (MOTORS[motor_pin][5] /  PPR); // (NUM_PULSES / PPR)
}

float get_motor_angle(int motor_pin) {
  // RETURNS THE ROTATION ANGLE FROM THE ORIGIN
  return get_motor_rotations(motor_pin) * 360;  // NUM_ROTATIONS * 360
}

float get_motor_dist(int motor_pin){
  // RETURNS THE DISTANCE (cm) FROM THE ORIGIN THE "MOTOR" HAS TRAVELED (IE THE ROD IN OUR CASE)
  return get_motor_rotations(motor_pin) * cm_PER_ROT; // NUM_ROTATIONS * ~CIRCUMFERENCE
}


// VIRTUAL FUNCTIONS
void setup() {
  Serial.begin(2000000); // MUST CHANGE IN .ini FILE TO VIEW IN SERIAL MONITOR
  if (RUN) {
    run_motor(test_motor, 0, MAX_PWM);
  }
}


void loop() {
  if(RUN){

    record_pulses();

    if (MOTORS[test_motor][5] >= PPR){
      analogWrite(MOTORS[test_motor][1], 0);
      exit(0);
    }
    Serial.print(get_motor_angle(test_motor));
    Serial.print("º | cm = ");
    Serial.println(get_motor_dist(test_motor));
  }
}