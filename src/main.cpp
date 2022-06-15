#include <Arduino.h>


const int MOTORS[3][4] = {
  {7, 4, 48, 49}, // MOTOR 1 | PHASE | ENABLE | ENC-A | ENC-B
  {6, 3, 50, 51}, // MOTOR 2 | PHASE | ENABLE | ENC-A | ENC-B
  {5, 2, 52, 53}, // MOTOR 3 | PHASE | ENABLE | ENC-A | ENC-B
};


void run_motor(int motor_pin, int dir, int pwm_val) {
  analogWrite(MOTORS[motor_pin][1], pwm_val);
  digitalWrite(MOTORS[motor_pin][0], dir);

}


void setup() {
  Serial.begin(250000); // MUST CHANGE IN .ini FILE TO VIEW IN SERIAL MONITOR
}

int count = 0;
void loop() {
  if(0){
    static int motor = 0;
    static bool hell = false;
    run_motor(motor, 0, 40);

    int enc[2] = {digitalRead(MOTORS[motor][2]), digitalRead(MOTORS[motor][3])};
    if (count < 201 && (enc[0] + enc[1] == 2 || hell)){
      hell = true;
      count += 1;
      Serial.println(enc[0] + enc[1]);
    }
  }
}


// int interpret_enc(int motor) {
//   return MOTORS[motor][]
// }