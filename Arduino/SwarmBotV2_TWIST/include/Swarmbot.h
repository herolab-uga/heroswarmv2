
#ifndef SWARMBOT_H
#define SWARMBOT_H
    #include <Arduino.h>
    #include <Adafruit_NeoPixel.h>
    
    class SwarmBot{
        //Motor Driver and Encoder Definitions 
        byte leftMotorIN1;
        byte leftMotorIN2;
        byte leftEncoderA;
        byte leftEncoderB;

        byte rightMotorIN1;
        byte rightMotorIN2;
        byte rightEncoderA;
        byte rightEncoderB;

        //Physical Constants
        int encoderCPP;
        int gearRatio;
        float wheelDiameter;
        float wheelBase;
        double toDegrees;
        double toRadians;
        float ticksPerMeter;

        //Odometery Constants
        volatile int leftLastEncoded;
        volatile int leftEncoderValue;
        double leftDeltaTicks;
        double leftLastEncoderValue;
        float leftOmegaDeg;
        float leftOmegaRad;
        float leftRPM;
        float leftVelocity;
        float leftDegrees;
        float leftDeltaDegrees;
        float leftDeltaRevolutions;

        volatile int rightLastEncoded;
        volatile int rightEncoderValue;
        double rightDeltaTicks;
        double rightLastEncoderValue;
        float rightOmegaDeg;
        float rightOmegaRad;
        float rightRPM;
        float rightVelocity;
        float rightDegrees;
        float rightDeltaDegrees;
        float rightDeltaRevolutions;

        double lastTime;
        double maxSpeed;

        float X;
        float Y;
        float lastX;
        float lastY;
        float velocityX;
        float velocityY;

        float targetVel;
        float targetAngularVel;

        float thetaRadOdom;
        float thetaDegOdom;
        float thetaRadIMU;
        float thetaDegIMU;

        float linearVelocity;
        float angularVelocityOdom;
        float angularVelocityIMU;
        float vTheta;

        float aKp;
        float aKi;
        float aKd;
        float angleIntegral;
        float angleDerivative;
        float angleLastError;

        float lKp;
        float lKi;
        float lKd;
        float linearIntegral;
        float linearDerivative;
        float linearLastError;


        float aVKp;
        float aVKi;
        float aVKd;
        float aVFF;
        float angleVelIntegral;
        float angleVelDerivative;
        float angleVelLastError;


        float lmKp;
        float lmKi;
        float lmKd;
        float lmFF;
        float lmFB;
        float leftMotorIntegral;
        float leftMotorDerivative;
        float leftMotorLastError;
        float leftMotorLastSpeed;

        float rmKp;
        float rmKi;
        float rmKd;
        float rmFF;
        float rmFB;
        float rightMotorIntegral;
        float rightMotorDerivative;
        float rightMotorLastError;
        float rightMotorLastSpeed;
        Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);

        public:

            //SwarmBot Constructor
            SwarmBot(
                     byte leftIN1 = 13, 
                     byte leftIN2 = 12, 
                     byte leftA = A1, 
                     byte leftB = A0, 
                     byte rightIN1 = 10,
                     byte rightIN2 = 9, 
                     byte rightA = A3, 
                     byte rightB = A2, 
                     int CPP = 28, 
                     int ratio = 100, 
                     float wheelDiameter = 0.0325, 
                     float wheelBase = 0.083
                    );

            void initializePorts();
            byte getLeftEncoderA();
            byte getLeftEncoderB();
            byte getRightEncoderA();
            byte getRightEncoderB();
            byte getLeftIn1();
            byte getLeftIn2();
            byte getRightIn1();
            byte getRightIn2();

            void setPIDSetpoint(float,float);
            
            float getX();
            void setX(float);
            float getY();
            void setY(float);
            float getHeading();
            void setHeading(float);
            float getLinearVel();
            float getAngularVel();
            float getMaxSpeed();
            float getRightFeedForward();
            float getLeftFeedForward();

            void tunePID(float p, float i, float d);
            //Base Motor Control
            void setLeftMotorSpeed(float speed);
            void setRightMotorSpeed(float speed);
                                                         
            //Odometery Functions
            void updateLeftEncoder();
            void updateRightEncoder();
            void updateWheels();
            float angleWrap(float angle, bool isRad = true);
            float angleWrap2(float angle, bool isRad = true);
            void updateOdometery();
            void callibrateOdometery(float inputArray[3]);

            //Chassis Movement
            float linearVelocityPID(float velocity);
            float angularVelocityPID(float omega);
            void setVelocity(float velocity, float omega);
            void updateVel(float velocity, float omega);
            float distancePID(float distance);
            float anglePID(float theta);
            void setMotorSpeed(float leftMotorSpeed, float rightMotorSpeed);
            float setLinVel(float velocity);
            float setAngVel(float omega);

            //Sensors
            //Output
            void printOdom(bool driveMode, float a, float b);
            void obstacleAvoidance();
            void setColor(int,int,int);
    };

#endif
