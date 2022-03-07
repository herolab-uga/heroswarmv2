#include <Wire.h>
#include <Swarmbot.h>

SwarmBot steve; 
imu IMU;

int timer = 0;
bool SerialFlag = false;
bool driveMode = true;
float targetX, targetY, linearVelocity, angularVelocity;
float inputArray[3];
float scaleFactor = 0;

void leftInterrupt(){
  steve.updateLeftEncoder();
}
void rightInterrupt(){
  steve.updateRightEncoder();
}

void sendEvent(){
  float odometryData[5];
  odometryData[0] = steve.getX();
  odometryData[1] = steve.getY();
  odometryData[2] = steve.getHeading();
  odometryData[3] = steve.getLinearVel();
  odometryData[4] = steve.getAngularVel();
  Wire.write((byte*)odometryData, sizeof(odometryData));
}
void setup() {

  Wire.begin(0x8);
  Wire.onRequest(sendEvent);

  Serial1.begin(9600);

  attachInterrupt(steve.getLeftEncoderA(), leftInterrupt, CHANGE);
  attachInterrupt(steve.getLeftEncoderB(), leftInterrupt, CHANGE);

  attachInterrupt(steve.getRightEncoderA(), rightInterrupt, CHANGE);
  attachInterrupt(steve.getRightEncoderB(), rightInterrupt, CHANGE);
  steve.initializePorts();
  delay(500);
}

void loop() {

  steve.updateOdometery();
  
  if (Serial1.available()){
    int mode = Serial1.parseInt();
    if (mode == 0) {
      linearVelocity = Serial1.parseFloat();
      angularVelocity = Serial1.parseFloat();
      while (Serial1.available()) {
        Serial1.read();
      }
      steve.setVelocity(linearVelocity, angularVelocity);
      steve.printOdom(driveMode, linearVelocity, angularVelocity);
    } else if (mode == 1) {
        steve.moveToPoint(targetX, targetY);
        while (Serial1.available()) {
        Serial1.read();
      }
    }
  }
  delay(50);
}