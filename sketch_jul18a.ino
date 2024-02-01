#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define leftMotorPWMPin   3
#define leftMotorDirPin   5
#define leftMotorDirPin2  4
#define rightMotorPWMPin  10
#define rightMotorDirPin  12
#define rightMotorDirPin2  11

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

double setpoint = 175;
//double oscilacionTime = 0.2;

double Kp = 25; //Set this first 25
double Kd = 1.8; //Set this secound 0.8/1.8/2/2.2
double Ki = 100; //Finally set this  100
/*
double Kp = 30*0.6;//25/12.6=0.6*Kp
double Kd = oscilacionTime*Kp/8;//0.8/time*Kp/8
double Ki = 2*Kp/oscilacionTime;//100/50.4 = 2*Kp/time
*/
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;

void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(44);
    mpu.setYGyroOffset(46);
    mpu.setZGyroOffset(8);
    mpu.setZAccelOffset(1192);

    if (devStatus == 0)
    {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(leftMotorDirPin, OUTPUT);
    pinMode(leftMotorDirPin2, OUTPUT);
    pinMode(rightMotorDirPin, OUTPUT);
    pinMode(rightMotorDirPin2, OUTPUT);
    pinMode(leftMotorPWMPin, OUTPUT);
    pinMode(rightMotorPWMPin, OUTPUT);

    analogWrite(leftMotorDirPin, LOW);
    analogWrite(leftMotorDirPin2, LOW);
    analogWrite(rightMotorDirPin, LOW);
    analogWrite(rightMotorDirPin2, LOW);
    analogWrite(leftMotorPWMPin, LOW);
    analogWrite(rightMotorPWMPin, LOW);
}

void loop() {
    if (!dmpReady) return;

    if (mpuInterrupt) {
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024)
        {
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));
        }
        else if (mpuIntStatus & 0x02)
        {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            input = ypr[1] * 180 / M_PI + 180;
        }
    }

    controlLoop(); // Call the non-blocking control loop function
}

void controlLoop() {
    pid.Compute();
    Serial.print(input); Serial.print(" =>"); Serial.println(output);

    if (input > 145 && input < 195) {
        if (output > 0)
            Forward();
        else if (output < 0)
            Reverse();
    }
    else
        Stop();
}

void Forward() {
    analogWrite(leftMotorPWMPin, output);
    analogWrite(rightMotorPWMPin, output * 0.9);
    analogWrite(leftMotorDirPin, output);
    analogWrite(leftMotorDirPin2, 0);
    analogWrite(rightMotorDirPin, output * 0.9);
    analogWrite(rightMotorDirPin2, 0);
    Serial.print("F");
}

void Reverse() {
    analogWrite(leftMotorPWMPin, output * -1);
    analogWrite(rightMotorPWMPin, (output * -1) * 0.7);
    analogWrite(leftMotorDirPin, 0);
    analogWrite(leftMotorDirPin2, output * -1);
    analogWrite(rightMotorDirPin, 0);
    analogWrite(rightMotorDirPin2, (output * -1) * 0.7);
    Serial.print("R");
}

void Stop() {
    analogWrite(leftMotorPWMPin, 0);
    analogWrite(rightMotorPWMPin, 0);
    analogWrite(leftMotorDirPin, 0);
    analogWrite(leftMotorDirPin2, 0);
    analogWrite(rightMotorDirPin, 0);
    analogWrite(rightMotorDirPin2, 0);
    Serial.print("S");
}
