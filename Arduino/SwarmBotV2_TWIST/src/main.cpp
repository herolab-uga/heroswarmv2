
#include <Arduino.h> 
#include <Swarmbot.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
SwarmBot steve; 
// imu IMU;

int timer = 0;
bool serialFlag = false;
bool driveMode = true;
float targetX, targetY, linearVelocity, angularVelocity;
float inputArray[4];
float scaleFactor = 0;
//float data[5];
float measuredvbat  = 0;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);

void leftInterrupt(){
  steve.updateLeftEncoder();
}
void rightInterrupt(){
  steve.updateRightEncoder();
}

union BytesToFloat {
    byte valueBuffer[16];
    float valueReading[4];    
} converter;

void printInfo(){
    for(uint8_t index = 0; index<4; index++){
        // Serial.print("The number is: ");
        float data = converter.valueReading[index];
        inputArray[index] = data;
    }

    serialFlag = false;
}
void receiveEvent(int byteCount){
    for(uint8_t index = 0; index<byteCount; index++){
        converter.valueBuffer[index] = Wire.read();
        Serial.println(converter.valueBuffer[index]);
    }
    serialFlag = true;
}

void sendEvent(){
  float data[6];
  data[0] = steve.getX();
  data[1] = steve.getY();
  data[2] = steve.getHeading();
  data[3] = steve.getLinearVel();
  data[4] = steve.getAngularVel();
  data[5] = measuredvbat;
  // Serial.print("Get X: ");
  // Serial.println(data[0]);
  Wire.write((byte*)data, sizeof(data));
  //Serial.println(steve.getX(),8);
}

void interpretData(float array[4]){
  switch((int) array[0]){
    case 0:
      linearVelocity = array[1];
      angularVelocity = array[3];
      steve.setPIDSetpoint(linearVelocity,angularVelocity);
      break;
    case 1:
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(array[1], array[2], array[3]));
      pixels.show();
    default:
      break;
  }
}
void setup() {
  //IMU.setupIMU();

  measuredvbat = analogRead(A6);
  measuredvbat *= 2; // we divided by 2, so multiply back
  measuredvbat *= 3.6; // Multiply by 3.6V, our reference voltage
  measuredvbat /= 1024; // convert to voltage


  Wire.begin(0x8);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(sendEvent);

  attachInterrupt(steve.getLeftEncoderA(), leftInterrupt, CHANGE);
  attachInterrupt(steve.getLeftEncoderB(), leftInterrupt, CHANGE);

  attachInterrupt(steve.getRightEncoderA(), rightInterrupt, CHANGE);
  attachInterrupt(steve.getRightEncoderB(), rightInterrupt, CHANGE);
  steve.initializePorts();
  Serial.begin(115200);
  
  // Serial.print("ready");

  pinMode(A6,INPUT);

  pixels.begin();
  pixels.clear();
  pixels.setBrightness(255);
  pixels.show();

  delay(1000);
}

void loop() {
  measuredvbat = analogRead(A6);
  measuredvbat *= 2; // we divided by 2, so multiply back
  measuredvbat *= 3.6; // Multiply by 3.6V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  steve.updateOdometery();
  

  if(serialFlag){
    printInfo();
    serialFlag = false;
    // Serial.println("Recieved: ");
    interpretData(inputArray);
  } 

  if(driveMode == false){
    steve.moveToPoint(targetX, targetY);
  //  steve.printOdom(driveMode, targetX, targetY);
  }
  else if(driveMode == true){
    steve.setVelocity(linearVelocity, angularVelocity);
    // steve.printOdom(driveMode, linearVelocity, angularVelocity);
  }

  delay(50);
  }