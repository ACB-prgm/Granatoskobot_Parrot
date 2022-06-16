#include <Arduino.h>


int MOTORS[3][6] = {
  {7, 4, 48, 49, 0, 0}, // MOTOR 1, PHASE_PIN, EN_PIN, ENC-A, ENC-B, DIRECTION, NUM_PULSES  | ANGLEº = (NUM_PULSES /  PPR) * 360º
  {6, 3, 50, 51, 0, 0},
  {5, 2, 52, 53, 0, 0},
};
const int test_motor = 0;
const int PPR = 4560;


void run_motor(int motor_pin, int dir, int pwm_val) {
  analogWrite(MOTORS[motor_pin][1], pwm_val);
  digitalWrite(MOTORS[motor_pin][0], dir);

  MOTORS[motor_pin][4] = dir;
}


void record_pulses() {
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
      MOTORS[i][5] += 1;
      last_encs[i] = enc;
    };
    Serial.print(MOTORS[i][5]);
    Serial.print(" | ");
  }

  Serial.println();
}


void setup() {
  Serial.begin(250000); // MUST CHANGE IN .ini FILE TO VIEW IN SERIAL MONITOR
  run_motor(0, 0, 40);
}


int RUN = 1;

void loop() {
  if(RUN){
    record_pulses();

    if (MOTORS[test_motor][5] >= PPR){
      analogWrite(MOTORS[test_motor][1], 0);
    }
  }
}