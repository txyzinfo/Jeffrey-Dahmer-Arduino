//       ____.       _____  _____                       ________         .__                          
//      |    | _____/ ____\/ ____\______   ____ ___.__. \______ \ _____  |  |__   _____   ___________ 
//      |    |/ __ \   __\\   __\\_  __ \_/ __ <   |  |  |    |  \\__  \ |  |  \ /     \_/ __ \_  __ \
//  /\__|    \  ___/|  |   |  |   |  | \/\  ___/\___  |  |    `   \/ __ \|   Y  \  Y Y  \  ___/|  | \/
//  \________|\___  >__|   |__|   |__|    \___  > ____| /_______  (____  /___|  /__|_|  /\___  >__|   
//                \/                          \/\/              \/     \/     \/      \/     \/       
//  v 0.9
//  code tested on Teensy 3.1 and it will work the same on 3.2

#include <Servo.h> 
#include <PulsePosition.h> //https://www.pjrc.com/teensy/td_libs_PulsePosition.html

#define RC_receiver 9  //OrangeRx R615X DSM2/DSMX Compatible 6Ch 2.4GHz Receiver W/CPPM

#define ENC_B 1
#define ENC_A 2
#define h_bridge_PWM 3 //I'm using MC33926, Pololu has simmilar to my custom made breakout. 
#define h_bridge_1 4
#define h_bridge_2 5
#define h_bridge_FB A1

#define sharpSensor 14 //Pololu Carrier with Sharp GP2Y0D805Z0F Digital Distance Sensor

#define LXservoPin 22  //Left rubber ball axis servo
#define LYservoPin 23  //Left servo closest to the battery
#define RXservoPin 20  //Right rubber ball axis servo
#define RYservoPin 21  //Right servo closest to the battery
#define flipperServoPin 6
#define clawServoPin 10

Servo LXservo;
Servo LYservo;
Servo RXservo;
Servo RYservo;
Servo flipperServo;
Servo clawServo;

int X_max_travel = 1400; //max travel from home position
int Y_max_travel = 1400;
int flipper_max_travel = 750;
int claw_max_travel = 485;

int LX_0 = 1560, LX_0f = LX_0;
int LY_0 = 1620, LY_0f = LY_0;
int RX_0 = 1500, RX_0f = RX_0;
int RY_0 = 1370, RY_0f = RY_0;

int LX_pos, LY_pos, RX_pos, RY_pos, flipper_pos, claw_pos = 0;

int LX_HOME = 2250;
int LX_MAX = LX_HOME;
int LX_MIN = LX_HOME - X_max_travel;

int RX_HOME = 810;
int RX_MIN = RX_HOME;
int RX_MAX = RX_HOME + X_max_travel;

int LY_HOME = 752;
int LY_MIN = LY_HOME;
int LY_MAX = LY_HOME + Y_max_travel;

int RY_HOME = 2235;
int RY_MAX = RY_HOME;
int RY_MIN = RY_HOME - Y_max_travel;

int flipper_HOME = 750;
int flipper_MIN = flipper_HOME;
int flipper_MAX = flipper_HOME + flipper_max_travel;

int claw_HOME = 1060;
int claw_MIN = claw_HOME;
int claw_MAX = claw_HOME + claw_max_travel;

int L_V_center = 1477;
int L_V_max = 1900;
int L_H_center = 1543;
int L_H_max = 1900;
int R_V_center = 1498;
int R_V_max = 1900;
int R_H_center = 1498;
int R_H_max = 1900;

PulsePositionInput myIn;

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT); // LED
  //pinMode(ENC_B, INPUT); no need if using encoder library
  //pinMode(ENC_A, INPUT);
  pinMode(h_bridge_2, OUTPUT); 
  pinMode(h_bridge_1, OUTPUT);
  pinMode(h_bridge_PWM, OUTPUT);
  pinMode(h_bridge_FB, INPUT);
  pinMode(sharpSensor, INPUT);

  //testing H-bridge first time
  digitalWrite(h_bridge_PWM, LOW);
  digitalWrite(h_bridge_2, HIGH);
  digitalWrite(h_bridge_1, LOW);
 
  myIn.begin(RC_receiver); // PPM Stream input pin (receiver connected to this pin)
}

int mode = 1;
bool AUTO = false;
bool POWER_ON = false;

int R_H, R_V, L_V, L_H, GEAR, GYRO;
int LX, LY, RX, RY;
int w;

void loop() {
  
  if (myIn.available()==6) {
        R_H  = myIn.read(1); // Right Horizontal stick
        R_V  = myIn.read(2); // Right Vertical stick
        L_V  = myIn.read(3); // Left Vertical stick
        L_H  = myIn.read(4); // Left Horizontal stick
        GEAR = myIn.read(5); // F-MODE/GEAR switch
        GYRO = myIn.read(6); // FLAP/GYRO switch (how to setup CH6 on Orange https://www.youtube.com/watch?v=gUUHXaK4PKg)
  }
  
  if (GEAR > 1750) AUTO = true; else AUTO = false; //GEAR=1070 if receiver is not connected to transmiter, so it's in Manual mode by default/reciver is OFF/not_connected

  if (GYRO > 1750) { //GYRO=1497 if receiver is not connected to transmiter
        mode = 0;
  } else if (GYRO < 1250) {
        mode = 2;
  } else {
        mode = 1;
  }

  //POWER ON: left & right sticks up/forward
  if (!POWER_ON){
        if (L_V>1900 && R_V>1900) {
            POWER_ON = true;
            digitalWrite(13, HIGH);
            //awakening(); // move all servos from home position to work/center position
            startFromCenter(); // attach servos and go to center/home position
        } else {
            digitalWrite(13, LOW);
        }
  }


  if (POWER_ON) {
        if (AUTO) {   //Auto mode
            switch (mode) {
                case 0: walkAlgorithmA(); break;
                case 1: walkAlgorithmB(); break;
                case 2: walkAlgorithmC(); break;
            }        
        } else {    //Manual mode
            switch (mode) {
                case 0: manualModeA(0, 0, 300, 300); break;
                case 1: manualModeB(200, 0, 300, 300); break;
                case 2: manualModeC(200, 200, 200, 200); break;
            }
        }
  } else {
        Serial.print(L_V); Serial.print("\t");
        Serial.print(L_H); Serial.print("\t");
        Serial.print(R_V); Serial.print("\t");
        Serial.print(R_H); Serial.print("\t");
        Serial.print(GYRO); Serial.print("\t");
        Serial.print(GEAR); Serial.print("\n");
  }

}


void LXmove (int q) {
      w = LX_0f - q;
      if (w >= LX_MIN && w <= LX_MAX) { LX_pos = q;   LXservo.writeMicroseconds(w); }
}

void RXmove (int q) {
      w = RX_0f + q;
      if (w >= RX_MIN && w <= RX_MAX) { RX_pos = q;   RXservo.writeMicroseconds(w); }
}

void LYmove (int q) {
      w = LY_0f + q;
      if (w >= LY_MIN && w <= LY_MAX) { LY_pos = q;   LYservo.writeMicroseconds(w); }
}

void RYmove (int q) {
      w = RY_0f - q;
      if (w >= RY_MIN && w <= RY_MAX) { RY_pos = q;   RYservo.writeMicroseconds(w); }
}

void flipperMove (int q) {
      int w = flipper_HOME + q;
      if (w >= flipper_MIN && w <= flipper_MAX) { flipper_pos = q;    flipperServo.writeMicroseconds(w); }
}

void clawMove (int q) {
      int w = claw_HOME + q;
      if (w >= claw_MIN && w <= claw_MAX) { claw_pos = q;    clawServo.writeMicroseconds(w); }
}

//// Each axis on sticks control each servo on robot arms
void manualModeA (int x_center_offset, int y_center_offset, int x_max, int y_max) {
              LX_0f = LX_0 - x_center_offset;
              RX_0f = RX_0 + x_center_offset;
              LY_0f = LY_0 + y_center_offset;
              RY_0f = RY_0 - y_center_offset;
              LX = map(L_V, L_V_center, 1900, 0, 0 - x_max);
              RX = map(R_V, R_V_center, 1900, 0, 0 - x_max);
              LY = map(L_H, L_H_center, 1900, 0, 0 + y_max);
              RY = map(R_H, R_H_center, 1900, 0, 0 - y_max);
              LXmove(LX); 
              RXmove(RX);
              LYmove(LY);
              RYmove(RY);
}

//// Left stick control claw rotation in OPEN LOOP, and servo on gripper
//// Right stick control arms synchronously.
void manualModeB (int x_center_offset, int y_center_offset, int x_max, int y_max)  {
              LX_0f = LX_0 - x_center_offset;
              RX_0f = RX_0 + x_center_offset;
              LY_0f = LY_0 + y_center_offset;
              RY_0f = RY_0 - y_center_offset;
              LX = map(L_V, L_V_center, 1900, 0, 0 - x_max);
              RX = map(R_V, R_V_center, 1900, 0, 0 - x_max);
              LY = map(L_H, L_H_center, 1900, 0, 0 + y_max);
              RY = map(R_H, R_H_center, 1900, 0, 0 - y_max);
              LXmove(RX); 
              RXmove(RX);
              LYmove(RY);
              RYmove(RY);
}

//// Left stick control claw rotation in CLOSED LOOP, and servo on gripper
//// Right stick control arms synchronously.
void manualModeC (int x_center_offset, int y_center_offset, int x_max, int y_max)  {
              LX_0f = LX_0 - x_center_offset;
              RX_0f = RX_0 + x_center_offset;
              LY_0f = LY_0 + y_center_offset;
              RY_0f = RY_0 - y_center_offset;
              LX = map(L_V, L_V_center, 1900, 0, 0 - x_max);
              RX = map(R_V, R_V_center, 1900, 0, 0 - x_max);
              LY = map(L_H, L_H_center, 1900, 0, 0 + y_max);
              RY = map(R_H, R_H_center, 1900, 0, 0 - y_max);
              LXmove(RX); 
              RXmove(RX);
              LYmove(RY);
              RYmove(RY);
}

void walkAlgorithmA () {}

void walkAlgorithmB () {}

void walkAlgorithmC () {}

void showOffA () {}

void showOffB () {}

void showOffC () {}

void startFromCenter () {
      LXservo.attach(LXservoPin);      LXmove(0);
      RXservo.attach(RXservoPin);      RXmove(0);
      LYservo.attach(LYservoPin);      LYmove(0);
      RYservo.attach(RYservoPin);      RYmove(0);

      flipperServo.attach(flipperServoPin);       flipperMove(0);
      clawServo.attach(clawServoPin);             clawMove(0);
}

void awakening () {
    //temp deleted
}

