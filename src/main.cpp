#include <Arduino.h>
#include <math.h>


// GLOBALS ——————————————————————————————————————————————————————————————————————————————————————————————————
#define RUN 1
#define PPR 3576.0 
#define mm_PER_ROT 62.8
#define MAX_PWM 200  // THE MAX SPEED THE ENCODER CAN ~ACCURATELY READ AT (CAN GO FROM 0-255)
#define test_motor 0
#define d 57.0 // mm constant representing the distance between the rods and virtual backbone
#define kmax 500 // mm max curvature of virtual backbone

// s1,s2,s3 are the length of each rod i (i =1,2,3)
// S is the length of the virtual backbone
// k is the arclength curvature of the virtual backbone path ()= 1 / radius of curvature)
// phi direction of curvature
const String ENC_READS[4] = {"11", "01", "00", "10"};
float HEAD_COORDS[3] = {0, 0, 0}; // x, y, z
float RODS_DESIRED[6] = {0, 0, 0, 0, 0, 0}; // 0:s1, 1:s2, 2:s3, 3:S, 4:k, 5:phi
float RODS[6] = {0, 0, 0, 0, 0, 0}; // 0:s1, 1:s2, 2:s3, 3:S, 4:k, 5:phi
float MOTORS[3][6] = { // 0:MOTOR #, 1:PHASE_PIN, 2:EN_PIN, 3:ENC-A, ENC-B, 4:DIRECTION, 5:NUM_PULSES
  {9, 6, 2, 3, 0.5, 0}, 
  {8, 5, 18, 19, 0.5, 0},
  {7, 4, 20, 21, 0.5, 0},
};


// Interrupt Service Routines ———————————————————————————————————————————————————————————————————————————————
void rec_enc_00(){
  MOTORS[0][5] += (2 * MOTORS[0][4]) - 1;;
};

void rec_enc_01(){
  MOTORS[1][5] += (2 * MOTORS[1][4]) - 1;;
};

void rec_enc_02(){
  MOTORS[2][5] += (2 * MOTORS[2][4]) - 1;;
};


// CUSTOM FUNCTIONS —————————————————————————————————————————————————————————————————————————————————————————
void run_motor(int motor_pin, int dir, int pwm_val) {
  // MOVES MOTOR IN DIRECTION
  digitalWrite(MOTORS[motor_pin][0], dir);
  analogWrite(MOTORS[motor_pin][1], pwm_val);

  MOTORS[motor_pin][4] = dir;
}

float get_motor_rotations(int motor_pin) {
  // RETURNS THE NUMBER OF REVOLUTIONS FROM THE ORIGIN 
  // Serial.println(MOTORS[motor_pin][5]);
  if (MOTORS[motor_pin][5] == 0.0) {
    return 0.0; // PREVENT DIVISION BY ZERO ERROR
  }
  return (MOTORS[motor_pin][5] /  PPR); // (NUM_PULSES / PPR)
}

float get_motor_angle(int motor_pin) {
  // RETURNS THE ROTATION ANGLE FROM THE ORIGIN
  return get_motor_rotations(motor_pin) * 360;  // NUM_ROTATIONS * 360
}

float get_motor_dist(int motor_pin){
  // RETURNS THE DISTANCE (cm) FROM THE ORIGIN THE "MOTOR" HAS TRAVELED (IE THE ROD IN OUR CASE)
  return get_motor_rotations(motor_pin) * mm_PER_ROT; // NUM_ROTATIONS * ~CIRCUMFERENCE
}

void inv_kin() {
    RODS_DESIRED[0] = (1 + d * RODS_DESIRED[4] * cos(RODS_DESIRED[5])) * RODS_DESIRED[3];
    RODS_DESIRED[1] = (1 - RODS_DESIRED[4] * d * sin(M_PI/6 - RODS_DESIRED[5])) * RODS_DESIRED[3];
    RODS_DESIRED[2] = (1 - RODS_DESIRED[4] * d * sin(M_PI/6 + RODS_DESIRED[5])) * RODS_DESIRED[3];
}

void set_desired_from_input(float Sin, float Kfbin, float Klrin){
    RODS_DESIRED[3] = Sin / 1000;
    RODS_DESIRED[4] = min(sqrt(pow(Kfbin, 2) + pow(Klrin, 2)), kmax);
    RODS_DESIRED[5] = atan2(Klrin, Kfbin);

    inv_kin();
}

void find_head_coords(){
    float val = 1 - cos(RODS[4] * RODS[3]);

    HEAD_COORDS[0] = (val * cos(RODS[5])) / RODS[4];
    HEAD_COORDS[1] = (val * sin(RODS[5])) / RODS[4];
    HEAD_COORDS[2] = (sin(RODS[4] * RODS[3])) / RODS[4];
  }

void set_head_coords(float x, float y, float z){
  float val = 1 - cos(RODS[4] * RODS[3]);
  
  x = (val * cos(RODS[5]));
  y = (val * sin(RODS[5]));
  z = (sin(RODS[4] * RODS[3]));
}

bool is_about_equal(float sourceNum, float targetNum, float limit){
  return (sourceNum > targetNum - limit && sourceNum < targetNum + limit);
}


// VIRTUAL FUNCTIONS ————————————————————————————————————————————————————————————————————————————————————————
void setup() {
  Serial.begin(2000000); // MUST CHANGE IN .ini FILE TO VIEW IN SERIAL MONITOR

  attachInterrupt(digitalPinToInterrupt(MOTORS[0][2]), rec_enc_00, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTORS[0][3]), rec_enc_00, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTORS[1][2]), rec_enc_01, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTORS[1][3]), rec_enc_01, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTORS[2][2]), rec_enc_02, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTORS[2][3]), rec_enc_02, CHANGE);

  set_desired_from_input(2000, 1, 1);
  Serial.println(RODS_DESIRED[0]);
  Serial.println(RODS_DESIRED[1]);
  Serial.println(RODS_DESIRED[2]);
}

void loop() {
  if (millis() > 10000){
      for (int i=0; i<3; i++) {
        Serial.println("###### TIMEOUT ######");
        run_motor(i, 0, 0);
      }
      exit(0);
    }
  
  if(RUN){
    for (int i=0; i<3; i++) {
      float rod_length = get_motor_dist(i);
      RODS[i] = rod_length;

      // Serial.print(i);
      // Serial.print(": ");
      // Serial.print(rod_length);
      // Serial.print(" / ");
      // Serial.print(RODS_DESIRED[i]);
      // Serial.print(" ?== ");
      // Serial.println(is_about_equal(rod_length, RODS_DESIRED[i], 0.1));
      
      if (is_about_equal(rod_length, RODS_DESIRED[i], 0.1)){
        run_motor(i, MOTORS[i][4], 0); // STOP
      }
      else if (rod_length < RODS_DESIRED[i]) {
        run_motor(i, 1, MAX_PWM); // EXTEND
      }
      else if (rod_length > RODS_DESIRED[i]) {
        run_motor(i, 0, MAX_PWM); // RETRACT
      }
    };
    // Serial.print(get_motor_angle(test_motor));
    // Serial.print("º | cm = ");
    // Serial.println(get_motor_dist(test_motor));
  }
}


// GRAVEYARD ————————————————————————————————————————————————————————————————————————————————————————

// void record_pulses() {
//   // RECORDS THE NUMBER OF PULSES FROM ENCODERS TO EACH MOTOR IN MOTORS
//   // PULSES CAN THEN BE USED TO DETERMINE THE NUMBER OF ROTATIONS, DISTANCE, AND ANGLE
//   static int last_encs[3] = {3, 3, 3};

//   int encs[3] = {
//     digitalRead(MOTORS[0][2]) + digitalRead(MOTORS[0][3]),
//     digitalRead(MOTORS[1][2]) + digitalRead(MOTORS[1][3]),
//     digitalRead(MOTORS[2][2]) + digitalRead(MOTORS[2][3])
//     };
    
//   for (int i=0; i<3; i++) {
//     int last_enc = last_encs[i];
//     int enc = encs[i];

//     if (enc != last_enc) {
//       MOTORS[i][5] += (2 * MOTORS[i][4]) - 1; // left rotation decrements, right increments pulses
//       last_encs[i] = enc;
//     };
//   }
//   Serial.print(digitalRead(MOTORS[test_motor][2]));
//   Serial.println(digitalRead(MOTORS[test_motor][3]));
// }