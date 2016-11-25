//       ____.       _____  _____                       ________         .__                          
//      |    | _____/ ____\/ ____\______   ____ ___.__. \______ \ _____  |  |__   _____   ___________ 
//      |    |/ __ \   __\\   __\\_  __ \_/ __ <   |  |  |    |  \\__  \ |  |  \ /     \_/ __ \_  __ \
//  /\__|    \  ___/|  |   |  |   |  | \/\  ___/\___  |  |    `   \/ __ \|   Y  \  Y Y  \  ___/|  | \/
//  \________|\___  >__|   |__|   |__|    \___  > ____| /_______  (____  /___|  /__|_|  /\___  >__|   
//                \/                          \/\/              \/     \/     \/      \/     \/       
//  v 0.8

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

int X_max_travel = 1200;
int Y_max_travel = 1200;
int flipper_max_travel = 750;
int claw_max_travel = 485;

int X_center = 908;
int Y_center = 1100;


int LX_HOME = 2250;
int LX_MAX = LX_HOME;
int LX_MIN = LX_HOME - X_max_travel;
int LX_position = 0;

int RX_HOME = 810;
int RX_MIN = RX_HOME;
int RX_MAX = RX_HOME + X_max_travel;
int RX_position = 0;

int LY_HOME = 752;
int LY_MIN = LY_HOME;
int LY_MAX = LY_HOME + Y_max_travel;
int LY_position = 0;

int RY_HOME = 2250;
int RY_MAX = RY_HOME;
int RY_MIN = RY_HOME - Y_max_travel; ///here bug? RY_MIN = 1050
int RY_position = 0;

int flipper_HOME = 750;
int flipper_MIN = flipper_HOME;
int flipper_MAX = flipper_HOME + flipper_max_travel;
int flipper_position = 0;

int claw_HOME = 1060;
int claw_MIN = claw_HOME;
int claw_MAX = claw_HOME + claw_max_travel;
int claw_position = 0;

int L_V_center = 1500;
int L_H_center = 1503;
int R_V_center = 1498;
int R_H_center = 1498;


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

void loop() {
  myIn.available();
  
  R_H  = myIn.read(1); // Right Horizontal stick
  R_V  = myIn.read(2); // Right Vertical stick
  L_V  = myIn.read(3); // Left Vertical stick
  L_H  = myIn.read(4); // Left Horizontal stick
  GEAR = myIn.read(5); // F-MODE/GEAR switch
  GYRO = myIn.read(6); // FLAP/GYRO switch (how to setup CH6 on Orange https://www.youtube.com/watch?v=gUUHXaK4PKg)

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
            awakening(); // move all servos from home position to work position            
        } else {
            digitalWrite(13, LOW);
        }
  }


  if (POWER_ON) {
        if (AUTO) {   //Auto mode
            switch (mode) {
                case 0: walkAlgorithmA(); break;
                case 1: walkAlgorithmB(); break;
                case 2: walkAlgorithmB(); break;
            }        
        } else {    //Manual mode
            switch (mode) {
                case 0: manualModeA(); break;
                case 1: manualModeB(); break;
                case 2: manualModeC(); break;
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

void LXservoMove (int q) {
     int w = LX_HOME - q;
     if (w >= LX_MIN && w <= LX_MAX) {
        LX_position = q;
        LXservo.writeMicroseconds(w);
     }
}

void RXservoMove (int q) {
     int w = RX_HOME + q;
     if (w >= RX_MIN && w <= RX_MAX) {
        RX_position = q;
        RXservo.writeMicroseconds(w);
     }
}

void LYservoMove (int q) {
     int w = LY_HOME + q;
     if (w >= LY_MIN && w <= LY_MAX) {
        LY_position = q;
//        Serial.print("LY_w: "); Serial.println(w);
        LYservo.writeMicroseconds(w);
        
     }
}

void RYservoMove (int q) {
     int w = RY_HOME - q;
     if (w >= RY_MIN && w <= RY_MAX) {
        RY_position = q;
//        Serial.print("RY_w: "); Serial.println(w); 
        RYservo.writeMicroseconds(w);
     }
}

void flipperMove (int q) {
      int w = flipper_HOME + q;
      if (w >= flipper_MIN && w <= flipper_MAX) { 
          flipper_position = q;
          flipperServo.writeMicroseconds(w);
      }
}


void clawMove (int q) {
      int w = claw_HOME + q;
      if (w >= claw_MIN && w <= claw_MAX) {
          claw_position = q;
          clawServo.writeMicroseconds(w);
      }
      
}

//// Each axis on sticks control each servo on robot arms
void manualModeA () { 
              LX = map(L_V, L_V_center, 1900, X_center, X_center+400);
              LY = map(L_H, L_H_center, 1900, Y_center, Y_center+400);
              RX = map(R_V, R_V_center, 1900, X_center, X_center+400);
              RY = map(R_H, R_H_center, 1900, Y_center, Y_center+400);
            //Serial.print("  LX: "); Serial.print(LX);
            //Serial.print("  LY: "); Serial.print(LY);
            //Serial.print("  RX: "); Serial.print(RX);
            //Serial.print("  RY: "); Serial.println(RY);
              LXservo.writeMicroseconds(LX);
              LYservo.writeMicroseconds(LY);
              RXservo.writeMicroseconds(RX);
              RYservo.writeMicroseconds(RY);
    
}

//// Left stick control claw rotation in OPEN LOOP, and servo on gripper
//// Right stick control arms synchronously.
void manualModeB () {}

//// Left stick control claw rotation in CLOSED LOOP, and servo on gripper
//// Right stick control arms synchronously.
void manualModeC () {}

void walkAlgorithmA () {}

void walkAlgorithmB () {}

void walkAlgorithmC () {}

void showOffA () {}

void showOffB () {}

void showOffC () {}

void awakening () {
  
      LXservo.attach(LXservoPin);
      LXservo.writeMicroseconds(LX_HOME);
      LYservo.attach(LYservoPin);
      LYservo.writeMicroseconds(LY_HOME);
      RXservo.attach(RXservoPin);
      RXservo.writeMicroseconds(RX_HOME);
      RYservo.attach(RYservoPin);
      RYservo.writeMicroseconds(RY_HOME);
 
      flipperServo.attach(flipperServoPin); 
      flipperServo.writeMicroseconds(flipper_HOME); // 500min, 2400max 
      
      clawServo.attach(clawServoPin);
      clawServo.writeMicroseconds(claw_HOME);

      delay(100);
      clawServo.detach();


      for(int i=0; i<= 300; i++) {
          LXservoMove(i);
          delay(6);
      }
      delay(2000);
      for(int i=0; i<= 300; i++) {
          RXservoMove(i);
          LXservoMove(300-i);
          delay(2);
      }
      delay(1000);
      for(int i=0; i<= 300; i++) {
          LXservoMove(i);
          delay(4);
      }
      
      delay(500);
      
      clawServo.attach(clawServoPin);
      clawMove(claw_max_travel);
      flipperMove(flipper_max_travel);  
      
      LXservoMove(LX_position+390); 
      RXservoMove(RX_position+390);
      LYservoMove(LY_position + 1200);
      RYservoMove(RY_position + 1200);

      delay(2000);
      
      while (claw_position>0) {
             claw_position--;
             clawMove(claw_position);
             delay(1);
      }
      clawServo.detach();
      delay(500);

      while (flipper_position>0) {
             flipper_position--;
             flipperMove(flipper_position);
             delay(1);
      }

      delay(500);
      
      while (LY_position>900){
             LY_position--;
             RY_position--;
             LYservoMove(LY_position);
             RYservoMove(RY_position);
             delay(1);
       }

}

