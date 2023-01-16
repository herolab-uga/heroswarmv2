#include <Arduino.h>
#include <Swarmbot.h>
#include <Wire.h>
#include <NRF52TimerInterrupt.hpp>
#include <nrf.h>
#include "PDM.h"
SwarmBot steve;

int timer = 0;
bool serialFlag = false;
bool driveMode = true;
float targetX = 0, targetY = 0, linearVelocity = 0, angularVelocity = 0;
float inputArray[4];
float scaleFactor = 0;
float measuredvbat = 0;
float max_val = 0;
unsigned long lastTime = 0;
byte msg[256];
int msgLen = 0;
char recvChar;

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];

// number of samples read
volatile int samplesRead = 0;

void stop()
{
  NRF_TIMER2->TASKS_START = 0; // Stop TIMER2
  NRF_TIMER2->TASKS_STOP = 1;  // Stop TIMER2

  steve.setPIDSetpoint(0, 0);
  steve.setVelocity(0, 0);
  steve.setLeftMotorSpeed(0);
  steve.setRightMotorSpeed(0);
}

union BytesToFloat
{
  byte valueBuffer[16];
  float valueReading[4];
} converter;

void leftInterrupt(void);
void rightInterrupt(void);
void convertData(void);
void revcieveEvent(int);
void sendEvent();
void interpretData();

void leftInterrupt()
{
  steve.updateLeftEncoder();
}
void rightInterrupt()
{
  steve.updateRightEncoder();
}

void convertData()
{
  for (uint8_t index = 0; index < 4; index++)
  {
    // Serial.print("The number is: ");
    float data = converter.valueReading[index];
    inputArray[index] = data;
    Serial.println(data);
  }
}

void receiveEvent(int byteCount)
{
  for (uint8_t index = 0; index < byteCount; index++)
  {
    converter.valueBuffer[index] = Wire.read();
  }
  convertData();
  interpretData();
}

void sendEvent()
{
  if (samplesRead)
  {
    int max_val_temp = 0;
    // print samples to the serial monitor or plotter
    for (int i = 0; i < samplesRead; i++)
    {
      if (max_val_temp < sampleBuffer[i])
      {
        max_val_temp = sampleBuffer[i];
      }
    }
    // clear the read count
    max_val = max_val_temp;
    samplesRead = 0;
  }

  measuredvbat = analogRead(A6);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.6;  // Multiply by 3.6V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  float data[7];
  data[0] = steve.getX();
  data[1] = steve.getY();
  data[2] = steve.getHeading();
  data[3] = steve.getLinearVel();
  data[4] = steve.getAngularVel();
  data[5] = measuredvbat;
  data[6] = max_val;
  Wire.write((byte *)data, sizeof(data));
  samplesRead = 0;
}

void init_timer(void)
{
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;          // Set the timer in Counter Mode
  NRF_TIMER2->TASKS_CLEAR = 1;                       // clear the task first to be usable for later
  NRF_TIMER2->PRESCALER = 1;                         // Set prescaler. Higher number gives slower timer. Prescaler = 0 gives 16MHz timer
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit; // Set counter to 16 bit resolution
  NRF_TIMER2->CC[0] = 5000;                          // Set value for TIMER2 compare register 0 15384

  // Enable interrupt on Timer 2, both for CC[0] and CC[1] compare match events
  NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
  NVIC_EnableIRQ(TIMER2_IRQn);
}

extern "C"
{
  void TIMER2_IRQHandler(void)
  {
    NRF_TIMER2->EVENTS_COMPARE[0] = 0; // Clear compare register 0 event
    if (millis() - lastTime > 10000)
    {
      stop();
    }
    else
    {
      steve.updateOdometery();
      // Serial.println("Running");
    }
  }
}

void onPDMdata()
{
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

void setup()
{
  Serial.begin(9600);
  Serial1.begin(115200);

  // IMU.setupIMU();

  measuredvbat = analogRead(A6);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.6;  // Multiply by 3.6V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  Wire.begin(0x8);
  Wire.onRequest(sendEvent);

  // interrupting twice on 01 -> 10 and 11 -> 00?
  attachInterrupt(steve.getLeftEncoderA(), leftInterrupt, CHANGE);
  attachInterrupt(steve.getLeftEncoderB(), leftInterrupt, CHANGE);

  attachInterrupt(steve.getRightEncoderA(), rightInterrupt, CHANGE);
  attachInterrupt(steve.getRightEncoderB(), rightInterrupt, CHANGE);

  steve.initializePorts();

  pinMode(A6, INPUT);

  steve.setColor(0, 0, 0);
  init_timer();
  PDM.onReceive(onPDMdata);

  if (!PDM.begin(1, 16000))
  {
    Serial.println("Failed to start PDM!");
    while (1)
      yield();
  }
  PDM.setGain(.75);
  steve.setPIDSetpoint(0, 0);
  steve.setVelocity(0, 0);
  // steve.updateOdometery();
}

void loop()
{
  // Serial.println("Waiting for command");
  if (Serial1.available())
  {
    // Serial.println("Command received");
    msgLen = Serial1.readBytesUntil('\n', msg, 255);
    int mode;
    memcpy(&mode, msg + 2, sizeof(int));
    // Serial.print("Mode: ");
    // Serial.println(mode);
    if (mode == 0)
    {

      memcpy(&linearVelocity, msg + 6, sizeof(float));
      memcpy(&angularVelocity, msg + 14, sizeof(float));
      // Serial.print("Linear Velocity: ");
      // Serial.println(linearVelocity);
      // Serial.print("Angular Velocity: ");
      // Serial.println(angularVelocity);
      Serial1.flush();
      Serial1.println("ack");

      if (abs(linearVelocity) < .001 && abs(angularVelocity) < .001)
      {
        stop();
      }
      else
      {
        steve.setPIDSetpoint(linearVelocity, angularVelocity);
        steve.setVelocity(linearVelocity, angularVelocity);
        lastTime = millis();
        NRF_TIMER2->TASKS_STOP = 0;  // Start TIMER2
        NRF_TIMER2->TASKS_START = 1; // Start TIMER2
      }
    }
    else if (mode == 1)
    {
      float r;
      memcpy(&r, msg + 2, sizeof(float));

      float g;
      memcpy(&g, msg + 6, sizeof(float));

      float b;
      memcpy(&b, msg + 10, sizeof(float));

      steve.setColor((int)r, (int)g, (int)b);
      Serial1.println("ack");
    }
    else if (mode == 2)
    {
      Serial1.println("ack");
    }
  }
}