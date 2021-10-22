#include <Arduino.h> 
#include <Swarmbot.h>
#include <Wire.h>
// #include <String.h>
SwarmBot steve; 
imu IMU;

int timer = 0;
bool serialFlag = false;
bool driveMode = true;
float targetX, targetY, linearVelocity, angularVelocity;
float inputArray[3];
float scaleFactor = 0;
//float odometryData[5];

void leftInterrupt(){
  steve.updateLeftEncoder();
}
void rightInterrupt(){
  steve.updateRightEncoder();
}

union BytesToFloat {
    byte valueBuffer[12];
    float valueReading[3];    
} converter;

void printInfo(){
    for(uint8_t index = 0; index<3; index++){
        Serial.print("The number is: ");
        float data = converter.valueReading[index];
        Serial.println(data);
        inputArray[index] = data;
    }

    serialFlag = false;
}
void receiveEvent(int byteCount){
    for(uint8_t index = 0; index<byteCount; index++){
        converter.valueBuffer[index] = Wire.read();
    }
    serialFlag = true;
}

void sendEvent(){
  float odometryData[5];
  odometryData[0] = steve.getX();
  odometryData[1] = steve.getY();
  odometryData[2] = steve.getHeading();
  odometryData[3] = steve.getLinearVel();
  odometryData[4] = steve.getAngularVel();
  Serial.print("Get X: ");
  Serial.println(odometryData[0]);
  Wire.write((byte*)odometryData, sizeof(odometryData));
  //Serial.println(steve.getX(),8);
}

void interpretData(float array[3]){
  // //Case 1: recallibrateOdom Pos
  // if(int(array[0]) == 0){
  //   float outputArray[3];
  //   for(uint8_t index = 1; index<4; index++){
  //     outputArray[index-1] = array[index]; 
  //   }
  //   steve.callibrateOdometery(outputArray);
  //   Serial.println("Case1: Set");
  // }

  // //Case 2: Set Target Pos
  // if (int(array[0]) == 1){
  //   driveMode = false;
  //   targetX = array[1];
  //   targetY = array[2];
  //   Serial.println("Case2: Set");
  // }

  //Case 3: ROS Twist
  //if( int(array[0]) == 2){
    // driveMode = true;
    linearVelocity = array[0];
    angularVelocity = array[2];
    scaleFactor = (sqrt(pow(linearVelocity, 2) + pow(angularVelocity,2)) - steve.getMaxSpeed())/steve.getMaxSpeed(); 
    steve.setPIDSetpoint(linearVelocity * scaleFactor,angularVelocity * scaleFactor);
    Serial.println("Case3: Set");
  //}
  //Case 4: PID TUNING
  // if(int(array[0]) == 3){

    // steve.tunePID(array[0], array[1], array[2]);
    // linearVelocity = array[3];
    // Serial.println("Case4: Set");
    // Serial.println(linearVelocity);
  // }
}
void setup() {
  //IMU.setupIMU();

  Wire.begin(0x8);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(sendEvent);

  attachInterrupt(steve.getLeftEncoderA(), leftInterrupt, CHANGE);
  attachInterrupt(steve.getLeftEncoderB(), leftInterrupt, CHANGE);

  attachInterrupt(steve.getRightEncoderA(), rightInterrupt, CHANGE);
  attachInterrupt(steve.getRightEncoderB(), rightInterrupt, CHANGE);
  steve.initializePorts();
  Serial.begin(115200);
  
  Serial.print("ready");
  delay(1000);
}

void loop() {

  steve.updateOdometery();
  

  if(serialFlag){
    printInfo();
    serialFlag = false;
    Serial.println("Recieved: ");
    interpretData(inputArray);
  } 

  if(driveMode == false){
    steve.moveToPoint(targetX, targetY);
  //  steve.printOdom(driveMode, targetX, targetY);
  }
  else if(driveMode == true){
    steve.setVelocity(linearVelocity, angularVelocity);
    steve.printOdom(driveMode, linearVelocity, angularVelocity);
  }

  delay(50);
}