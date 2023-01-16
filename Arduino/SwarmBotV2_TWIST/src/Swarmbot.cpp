
#include <Swarmbot.h>

SwarmBot::SwarmBot(
                    byte leftIN1, 
                    byte leftIN2, 
                    byte leftA, 
                    byte leftB, 
                    byte rightIN1, 
                    byte rightIN2, 
                    byte rightA, 
                    byte rightB,  
                    int CPP, 
                    int ratio, 
                    float wheelDiameterM, 
                    float wheelBaseM
                )
{
    //Motor Driver and Encoder Definitions
    leftMotorIN1 = leftIN1;
    leftMotorIN2 = leftIN2;
    leftEncoderA = leftA;
    leftEncoderB = leftB;

    rightMotorIN1 = rightIN1;
    rightMotorIN2 = rightIN2;
    rightEncoderA = rightA;
    rightEncoderB = rightB;

    //Physical Constants
    encoderCPP = CPP;
    gearRatio = ratio;
    wheelDiameter = wheelDiameterM;
    wheelBase = wheelBaseM;
    toDegrees = (180 / PI);
    toRadians = (PI / 180);
    ticksPerMeter = (wheelDiameter * PI) / (encoderCPP * gearRatio);

    //Odometery Constants
    leftLastEncoded = 0;
    leftEncoderValue = 0;
    leftDeltaTicks = 0;
    leftLastEncoderValue = 0;
    leftOmegaDeg = 0;
    leftOmegaRad = 0;
    leftRPM = 0;
    leftDegrees = 0;
    leftDeltaDegrees = 0;
    leftDeltaRevolutions = 0;

    rightLastEncoded = 0;
    rightEncoderValue = 0;
    rightDeltaTicks = 0;
    rightLastEncoderValue = 0;
    rightOmegaDeg = 0;
    rightOmegaRad = 0;
    rightRPM = 0;
    rightDegrees = 0;
    rightDeltaDegrees = 0;
    rightDeltaRevolutions = 0;

    lastTime = 0;

    X = 0;
    Y = 0;
    lastX = 0;
    lastY = 0;
    velocityX = 0;
    velocityY = 0;
    thetaRadOdom = 0;
    thetaDegOdom = 0;
    thetaRadIMU = 0;
    thetaDegIMU = 0;

    linearVelocity = 0;
    angularVelocityOdom = 0;
    angularVelocityIMU = 0;

    //Linear Velocity PID Constants
    lKp = 2;
    lKi = 0;
    lKd = 0;
    linearIntegral = 0;
    linearDerivative = 0;
    linearLastError = 0;

    //Angular Velocity PID Constants
    aVKp = 5.0;   //0.06
    aVKi = 0.0; //0.001
    aVKd = 0.0;
    angleVelIntegral = 0;
    angleVelDerivative = 0;
    angleVelLastError = 0;

    //LeftMotor Velocity PID Constants
    lmFF = 165; //165 - everyone else
    lmFB = 195;
    leftMotorLastError = 0;
    leftMotorLastSpeed = 0;

    //RightMotor
    rmFF = 160; //155 - everyone else
    rmFB = 195;
    rightMotorLastError = 0;
    rightMotorLastSpeed = 0;

    // imu IMU;
    //  IMU.setupIMU();
}

byte SwarmBot::getLeftEncoderA()
{
    return leftEncoderA;
}
byte SwarmBot::getLeftEncoderB()
{
    return leftEncoderB;
}
byte SwarmBot::getRightEncoderA()
{
    return rightEncoderA;
}
byte SwarmBot::getRightEncoderB()
{
    return rightEncoderB;
}

byte SwarmBot::getLeftIn1()
{
    return leftMotorIN1;
}
byte SwarmBot::getLeftIn2()
{
    return leftMotorIN2;
}
byte SwarmBot::getRightIn1()
{
    return rightMotorIN1;
}
byte SwarmBot::getRightIn2()
{
    return rightMotorIN2;
}

float SwarmBot::getX()
{
    return X;
}

void SwarmBot::setX(float x)
{
    this->X = x;
}

float SwarmBot::getY()
{
    return Y;
}

void SwarmBot::setY(float y)
{
    this->Y = y;
}

float SwarmBot::getHeading()
{
    return thetaDegOdom;
}

void SwarmBot::setHeading(float heading)
{
    this->thetaDegOdom = heading;
}

float SwarmBot::getLinearVel()
{
    return linearVelocity;
}

float SwarmBot::getAngularVel()
{
    return angularVelocityOdom;
}

float SwarmBot::getMaxSpeed()
{
    return maxSpeed;
}

float SwarmBot::getRightFeedForward()
{
    return this->rmFF;
}

float SwarmBot::getLeftFeedForward()
{
    return this->lmFF;
}

void SwarmBot::setPIDSetpoint(float linear, float angular)
{
    rightMotorLastSpeed = ((linear + angular * wheelBase * .5) / 0.29) * (100);
    leftMotorLastSpeed = ((linear - angular * wheelBase * .5) / 0.29) * (100);
}

void SwarmBot::tunePID(float p, float i, float d)
{
    lKp = p;
    lKi = i;
    lKd = d;
    Serial.println(lKp);
    Serial.println(lKi);
    Serial.println(lKd);
}

void SwarmBot::initializePorts()
{

    pinMode(leftEncoderA, INPUT_PULLUP);
    pinMode(leftEncoderB, INPUT_PULLUP);
    digitalWrite(leftEncoderA, HIGH);
    digitalWrite(leftEncoderB, HIGH);

    pinMode(rightEncoderA, INPUT_PULLUP);
    pinMode(rightEncoderB, INPUT_PULLUP);
    digitalWrite(rightEncoderA, HIGH);
    digitalWrite(rightEncoderB, HIGH);

    this->pixels.begin();
    this->pixels.clear();
    this->pixels.setBrightness(255);
    this->pixels.show();
}

void SwarmBot::updateLeftEncoder()
{
    int MSB = digitalRead(leftEncoderA); //MSB = most significant bit
    int LSB = digitalRead(leftEncoderB); //LSB = least significant bit

    int encoded = (MSB << 1) | LSB;             //converting the 2 pin value to single number
    int sum = (leftLastEncoded << 2) | encoded; //adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        leftEncoderValue--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        leftEncoderValue++;

    leftLastEncoded = encoded; //store this value for next time
}

void SwarmBot::updateRightEncoder()
{
    int MSB = digitalRead(rightEncoderA); //MSB = most significant bit
    int LSB = digitalRead(rightEncoderB); //LSB = least significant bit

    int encoded = (MSB << 1) | LSB;              //converting the 2 pin value to single number
    int sum = (rightLastEncoded << 2) | encoded; //adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        rightEncoderValue--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        rightEncoderValue++;

    rightLastEncoded = encoded; //store this value for next time
}

void SwarmBot::updateWheels()
{ 
    //Left Motor
    leftDeltaTicks = leftEncoderValue - leftLastEncoderValue;
    leftDeltaRevolutions = leftDeltaTicks / (encoderCPP * gearRatio);
    leftDeltaDegrees = leftDeltaRevolutions * 360;

    //Right Motor
    rightDeltaTicks = rightEncoderValue - rightLastEncoderValue;
    rightDeltaRevolutions = rightDeltaTicks / (encoderCPP * gearRatio);
    rightDeltaDegrees = rightDeltaRevolutions * 360;

    //Archive Values
    leftLastEncoderValue = leftEncoderValue;
    rightLastEncoderValue = rightEncoderValue;
}

float SwarmBot::angleWrap(float angle, bool isRad)
{
    double conversionFactor;
    if (isRad == true)
        conversionFactor = PI;
    else
        conversionFactor = 180.0;

    if (angle > (2.0 * conversionFactor))
        angle -= (2.0 * conversionFactor);
    if (angle < 0.0)
        angle += (2.0 * conversionFactor);
    return angle;
}
float SwarmBot::angleWrap2(float angle, bool isRad)
{
    double conversionFactor;
    if (isRad == true)
        conversionFactor = PI;
    else
        conversionFactor = 180.0;

    if (angle < ((-1) * conversionFactor))
        angle += (2.0 * conversionFactor);
    if (angle > conversionFactor)
        angle -= (2.0 * conversionFactor);
    return angle;
}
void SwarmBot::updateOdometery()
{   
    updateWheels();
    //  IMU.updateIMU();
    // float heading = IMU.getHeading(true);
    ////Position
    //Convert Encoder Ticks to Linear Distance
    double dLeft = leftDeltaTicks * ticksPerMeter;
    double dRight = rightDeltaTicks * ticksPerMeter;
    double dCenter = (dLeft + dRight) / 2;
    double dPhi = (dRight - dLeft) / wheelBase;

    //Position Output
    thetaRadOdom += dPhi;
    X += dCenter * cos(thetaRadOdom);
    Y += dCenter * sin(thetaRadOdom);
    ////Velocity
    long currentTime = millis();
    if (currentTime < lastTime)
    {
        currentTime = lastTime;
    }

    //Time Conversions
    double deltaTime = (currentTime - lastTime);
    double deltaTimeSeconds = deltaTime / 1000;
    double deltaTimeMinutes = deltaTime / 60000;

    leftRPM = leftDeltaRevolutions / deltaTimeMinutes;
    rightRPM = rightDeltaRevolutions / deltaTimeMinutes;

    double vLeft = dLeft / deltaTimeSeconds;
    double vRight = dRight / deltaTimeSeconds;
    //double vCenter = (vRight + vLeft)/2;
    int direction;
    if ((vLeft + vRight) > 0)
    {
        direction = 1;
    }
    else if ((vLeft + vRight) < 0)
    {
        direction = -1;
    }
    else
    {
        direction = 0;
    }
    //Velocity Outputs
    // velocityX = vCenter * cos(thetaRadOdom);
    // velocityY = vCenter * sin(thetaRadOdom);
    float dX = X - lastX;
    float dY = Y - lastY;
    lastX = X;
    lastY = Y;
    velocityX = dX / deltaTimeSeconds;
    velocityY = dY / deltaTimeSeconds;

    linearVelocity = sqrt(pow(velocityX, 2) + pow(velocityY, 2)) * direction;
    angularVelocityOdom = (dPhi / deltaTimeSeconds);

    //Theta To Degrees
    thetaRadOdom = angleWrap(thetaRadOdom, true);
    thetaDegOdom = thetaRadOdom * toDegrees;

    lastTime = currentTime;
    this->updateVel(targetVel,targetAngularVel);
}
void SwarmBot::printOdom(bool mode, float a, float b)
{

    // float heading = IMU.getHeading(false);
    float angle = angleWrap2(thetaDegOdom, false);
    if (mode == false)
    {
        // Serial.print("X: ");
        Serial.print(X, 4);
        // Serial.print(" Y: ");
        Serial.print(Y, 4);
        // Serial.print(" targetX: ");
        Serial.print(a, 4);
        // Serial.print(" targetY: ");
        Serial.print(b, 4);
        // Serial.print(" Theta: ");
        Serial.println(angle);
    }
    else
    {
        // Serial.print("linVel: ");
        Serial.print(linearVelocity, 4);
        // Serial.print(" anglarVel: ");
        Serial.print(angularVelocityOdom);
        // Serial.print(" targetLin: ");
        Serial.print(a);
        // Serial.print(" targetAng: ");
        Serial.println(b);
    }
    // Serial.println("");
    // delay(20);
}

void SwarmBot::setLeftMotorSpeed(float speed)
{
    // Serial.print("Output Left: ");
    float outputLeft = abs(speed) + lmFF;
    // Serial.println(outputLeft);
    if (speed > 0)
    {
        analogWrite(leftMotorIN1, 0);
        analogWrite(leftMotorIN2, outputLeft);
    }
    else if (speed < 0)
    {
        analogWrite(leftMotorIN1, outputLeft);
        analogWrite(leftMotorIN2, 0);
    }
    else
    {
        analogWrite(leftMotorIN1, 0);
        analogWrite(leftMotorIN2, 0);
    }
}
void SwarmBot::setRightMotorSpeed(float speed)
{
    float outputRight = abs(speed) + rmFF;
    // Serial.print("Output Right: ");
    // Serial.println(outputRight);
    if (speed > 0)
    {
        analogWrite(rightMotorIN1, 0);
        analogWrite(rightMotorIN2, outputRight);
    }
    else if (speed < 0)
    {
        analogWrite(rightMotorIN1, outputRight);
        analogWrite(rightMotorIN2, 0);
    }
    else
    {
        analogWrite(rightMotorIN1, 0);
        analogWrite(rightMotorIN2, 0);
    }
}
void SwarmBot::setMotorSpeed(float leftMotorSpeed, float rightMotorSpeed)
{
    setLeftMotorSpeed(leftMotorSpeed);
    setRightMotorSpeed(rightMotorSpeed);
}

float SwarmBot::angularVelocityPID(float omega)
{
    float error = omega - angularVelocityOdom;
    // Serial.print("Angular Error: ");
    // Serial.println(error);
    angleVelIntegral += error;
    angleVelDerivative = error - angleVelLastError;
    angleVelLastError = error;

    return ((aVKp * error) + (aVKi * angleVelIntegral) + (aVKd * angleVelDerivative));
}
float SwarmBot::linearVelocityPID(float vel)
{
    float error = vel - linearVelocity;
    // Serial.print("Linear Error: ");
    // Serial.println(error);
    linearIntegral += error;
    linearDerivative = error - linearLastError;
    linearLastError = error;

    return ((lKp * error) + (lKi * linearIntegral) + (lKd * linearDerivative));
}
void SwarmBot::setVelocity(float velocity, float omega)
{ 
    targetVel = velocity;
    targetAngularVel = omega;
    updateVel(velocity,omega);
}

void SwarmBot::updateVel(float velocity, float omega){

    float linearOutput = linearVelocityPID(velocity);
    float angularOutput = angularVelocityPID(omega);
    float leftMotorOut = leftMotorLastSpeed + linearOutput - angularOutput;
    float rightMotorOut = rightMotorLastSpeed + linearOutput + angularOutput;

    leftMotorLastSpeed = leftMotorOut;
    rightMotorLastSpeed = rightMotorOut;
    setMotorSpeed(leftMotorOut, rightMotorOut);
}

void SwarmBot::callibrateOdometery(float inputArray[3])
{
    X = inputArray[0];
    Y = inputArray[1];
    thetaRadOdom = toRadians * inputArray[2];
}

float SwarmBot::setLinVel(float velocity)
{
    if (velocity != 0)
    {
        float motorOut = (((velocity - 0.0045) / 0.0011));
        return motorOut;
    }
    else
    {
        return 0;
    }
}
float SwarmBot::setAngVel(float omega)
{
    if (omega != 0)
    {
        float motorOut = (((omega - 1.2363) / .2882));
        return motorOut;
    }
    else
    {
        return 0;
    }
}

void SwarmBot::setColor(int R, int G, int B){
    this->pixels.clear();
    this->pixels.setPixelColor(0, this->pixels.Color(R,G,B));
    this->pixels.show();
}