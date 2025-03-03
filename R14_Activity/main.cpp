/**
 * @file main.cpp
 * @brief Code for Exploration 3
 * @date 03/03/2025
 * @author Luca Sivilotti
 */

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHBattery.h>
#include <FEHRCS.h>
#include <math.h>

#define IDENTIFIER "1130D6KKR"

// Can change the following if one motor is slower than another.
#define LEFT_MOTOR_CORRECTION_FACTOR 1
#define RIGHT_MOTOR_CORRECTION_FACTOR 1

#define BOT_WIDTH 7
#define PI 3.141592
#define DEGREES 360
#define UNIT_COUNTS 40.489

#define ON 0
#define OFF 1

#define POWER 25

FEHMotor leftMotor(FEHMotor::Motor0, 9.0);
FEHMotor rightMotor(FEHMotor::Motor1, 9.0);
DigitalEncoder encoderL(FEHIO::P0_1);
DigitalEncoder encoderR(FEHIO::P0_0);
DigitalInputPin distanceSensor(FEHIO::P3_0); // Distance sensor should be plugged into Port 0 in Bank 3


/**
 * @brief Calculates actual motor speed based on remaining battery power.
 *
 * @param percent desired motor percent
 * @return actual motor percent
 */
float motorSpeed(float percent)
{
    return (11.5 / Battery.Voltage()) * percent;
}

/**
 * @brief Drives in reverse unitl sensor can see a wall.
 */
void driveUntilSensorDetected()
{
    leftMotor.SetPercent(-1 * LEFT_MOTOR_CORRECTION_FACTOR * motorSpeed(POWER));
    rightMotor.SetPercent(-1 * RIGHT_MOTOR_CORRECTION_FACTOR * motorSpeed(POWER));

    while (distanceSensor.Value() == OFF)
        ;

    leftMotor.Stop();
    rightMotor.Stop();
}

/**
 * @brief Turns bot the desired degrees.
 *
 * @param degree amount to turn the robot
 * @warning degree > 0 means left, degree < 0 means right
 */
void turn(float degree)
{
    int counts = 2 * UNIT_COUNTS * (BOT_WIDTH * PI) * degree / DEGREES;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    if (degree > 0)
    {
        rightMotor.SetPercent(motorSpeed(RIGHT_MOTOR_CORRECTION_FACTOR * POWER));
        leftMotor.SetPercent(motorSpeed(LEFT_MOTOR_CORRECTION_FACTOR * POWER * -1));
    }
    else
    {
        rightMotor.SetPercent(motorSpeed(RIGHT_MOTOR_CORRECTION_FACTOR * POWER * -1));
        leftMotor.SetPercent(motorSpeed(LEFT_MOTOR_CORRECTION_FACTOR * POWER));
    }

    while (encoderR.Counts() + encoderL.Counts() < counts)
        ;

    rightMotor.Stop();
    leftMotor.Stop();
}

/**
 * @brief Moved robot given distance.
 *
 * @param distance desired distance for robot to go
 */
void drive(float distance)
{
    int counts = 2 * UNIT_COUNTS * distance;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    rightMotor.SetPercent(motorSpeed(RIGHT_MOTOR_CORRECTION_FACTOR * POWER));
    leftMotor.SetPercent(motorSpeed(LEFT_MOTOR_CORRECTION_FACTOR * POWER));

    while (encoderR.Counts() + encoderL.Counts() < counts)
        ;

    rightMotor.Stop();
    leftMotor.Stop();
}

int main(void)
{
    RCS.InitializeTouchMenu(IDENTIFIER);
    int lever = RCS.GetLever();
    while (true)
    {
        turn(360);
    }
    
}
