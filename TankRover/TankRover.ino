/*******************************************************************************
*  File Name     :  TankRover.ino
*  Version       :  V 0.0
*  Author        :  Taylor Jenkins
*  Description   :  IR Controlled/Autonomous Tank Robot project, using a single
*                   ultrasonic ping sensor. Based on Makeblock Starter Robot 
*                   Kit. Robot uses two motors, connected to connectors M1 and
*                   M2. IR receiver module connects on Port 6. The Ultrasonic 
*                   sensor connects on port 3.
*                   The Makeblock IR Controller/Transmitter is used for robot
*                   control. The four arrow buttons are used for directional 
*                   control (forward, reverse, left turn, right turn). The 
*                   number buttons control the speed, with 1 being slowest, and
*                   2 being the fastest. The 'A' button switches control of the
*                   robot between IR and Autonomous.
*******************************************************************************/
#include <Arduino.h>
#include <Makeblock.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Define constants
#define UP_ARROW       IR_BUTTON_PLUS
#define DOWN_ARROW     IR_BUTTON_MINUS
#define RIGHT_ARROW    IR_BUTTON_NEXT
#define LEFT_ARROW     IR_BUTTON_PREVIOUS
#define IR_BUTTON_MODE IR_BUTTON_TEST
#define MIN_SPEED 30
#define SPEED_FACTOR 25
#define IR_MODE 0
#define AUTO_MODE 1
#define BUTTON_SENSITIVITY 100
#define ULTRASONIC_SENSITIVITY 50

// Define expressions
#define DELTA(a,b) ((a > b) ? (a - b) : (b - a))

// Connect Motors
MeDCMotor LeftMotor(M1);
MeDCMotor RightMotor(M2);

// Connect Infrared Receiver Module
MeInfraredReceiver IRReceiverDecode(PORT_6);

// Connect Ultrasonic Sensor Module
//Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield.
MeUltrasonicSensor UltraSensor(PORT_3); 


uint8_t driveSpeed = 180; // speed is unsigned integer in range 0 to 255 inclusive
uint8_t mode = IR_MODE;   // initialize control mode
boolean left, right;
long modeDebounceTime = 0;
long ultrasonicDebounceTime = 0;
int distance = 0;

void setup()
{
    IRReceiverDecode.begin();
    Serial.begin(9600);
}

void loop()
{
    if(IRReceiverDecode.available() || IRReceiverDecode.buttonState()) 
    {
        switch(IRReceiverDecode.read()) 
        {
            // Direction control
            case UP_ARROW:
                Forward();
                break;
            case DOWN_ARROW:
                Reverse();
                break;
            case RIGHT_ARROW:
                RotateRight();
                break;
            case LEFT_ARROW:
                RotateLeft();
                break;
                
            // Speed control
            // Since response time it more important a higher speeds, start with high numbers
            case IR_BUTTON_9:
                SetSpeed(SPEED_FACTOR * 9 + MIN_SPEED);
                break;
            case IR_BUTTON_8:
                SetSpeed(SPEED_FACTOR * 8 + MIN_SPEED);
                break;
            case IR_BUTTON_7:
                SetSpeed(SPEED_FACTOR * 7 + MIN_SPEED);
                break;
            case IR_BUTTON_6:
                SetSpeed(SPEED_FACTOR * 6 + MIN_SPEED);
                break;
            case IR_BUTTON_5:
                SetSpeed(SPEED_FACTOR * 5 + MIN_SPEED);
                break;
            case IR_BUTTON_4:
                SetSpeed(SPEED_FACTOR * 4 + MIN_SPEED);
                break;
            case IR_BUTTON_3:
                SetSpeed(SPEED_FACTOR * 3 + MIN_SPEED);
                break;
            case IR_BUTTON_2:
                SetSpeed(SPEED_FACTOR * 2 + MIN_SPEED);
                break;
            case IR_BUTTON_1:
                SetSpeed(SPEED_FACTOR * 1 + MIN_SPEED);
                break;
                
            // Change mode
            case IR_BUTTON_MODE:
                if(DELTA(millis(), modeDebounceTime) > BUTTON_SENSITIVITY)
                {
                    if(++mode > AUTO_MODE) mode = IR_MODE;
                    modeDebounceTime = millis();
                }
                break;
            default:
                break;
        }
    }
    else
    {
        if(mode == IR_MODE) Stop();
        else if(DELTA(millis(), ultrasonicDebounceTime) > ULTRASONIC_SENSITIVITY)
        {
            ultrasonicDebounceTime = millis();
            AutoRun();
        }
    }
        
}

void SetSpeed(uint8_t s)
{
    driveSpeed = s;
}

void Forward()
{
    LeftMotor.run(driveSpeed);
    RightMotor.run(driveSpeed);
}

void Reverse()
{
    LeftMotor.run(-driveSpeed);
    RightMotor.run(-driveSpeed);
}

void RotateRight()
{
    LeftMotor.run(driveSpeed);
    RightMotor.run(-driveSpeed);
}

void RotateLeft()
{
    LeftMotor.run(-driveSpeed);
    RightMotor.run(driveSpeed);
}

void Stop()
{
    LeftMotor.stop();
    RightMotor.stop();
}

void AutoRun()
{
    int minRange = 6;
    int maxRange = 15;
    distance = UltraSensor.distanceInch();
    Serial.println(distance);
    if(distance > maxRange) 
    {
        Forward();
    }
    else if(distance < minRange) 
    {
        Reverse();
    }
    else
    {
        RotateLeft();
    }
}

