/**
 * @file main.cpp
 * @brief Code for Milestone 1
 * @date 02/24/2025
 * @author Luca Sivilotti
 */

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>

/*Generic constants*/
#define BOT_WIDTH 7
#define PI 3.141592
#define DEGREES 360

/*Direction constants*/
#define LEFT -1
#define RIGHT 1
#define BACKWARDS -1
#define FORWARDS 1

/*Switch constants*/
#define ON 0
#define OFF 1

/*Motor constants*/
#define VOLTAGE 9.0
#define F_POWER 25.
#define B_POWER -12.

/*Encoder constants*/
#define UNIT_COUNTS 40.489

/*Optosensor constants*/
#define L_DIV 1.45
#define M_DIV 1.45
#define R_DIV 1.45

/*State variables*/
enum Line
{
    LINE_OFF_LEFT,
    LINE_ON_LEFT,
    LINE_MIDDLE,
    LINE_ON_RIGHT,
    LINE_OFF_RIGHT
};

/*Left Optosensor*/
AnalogInputPin optol(FEHIO::P1_0);
/*Middle Optosensor*/
AnalogInputPin optom(FEHIO::P1_1);
/*Right Optosensor*/
AnalogInputPin optor(FEHIO::P1_2);
/*Front right switch*/
DigitalInputPin fr(FEHIO::P2_0);
/*Front left switch*/
DigitalInputPin fl(FEHIO::P2_1);
/*Back right switch*/
DigitalInputPin br(FEHIO::P2_2);
/*Back left switch*/
DigitalInputPin bl(FEHIO::P2_3);
/*Motor powering right wheel*/
FEHMotor rightMotor(FEHMotor::Motor1, VOLTAGE);
/*Motor powering left wheel*/
FEHMotor leftMotor(FEHMotor::Motor0, VOLTAGE);
/*Right encoder*/
DigitalEncoder encoderR(FEHIO::P0_1);
/*Left encoder*/
DigitalEncoder encoderL(FEHIO::P0_0);

/*Methods*/
int followLine(int);
int stateSense(int);
void turnOff(int);
void turnOn(int);
void straight(int);
void forward(int, double);
void backward(int, double);
void turn(int, int, int);
void stop();

/**
 * @brief Main method.
 */
int main(void)
{
    LCD.Clear();
    float x, y;
    while (!LCD.Touch(&x, &y))
        ;
    while (LCD.Touch(&x, &y))
        ;
    forward(F_POWER, 36);
    float a, b;
    while (!LCD.Touch(&a, &b))
        ;
    while (LCD.Touch(&a, &b))
        ;
    forward(F_POWER, 24);
    forward(B_POWER, 24);
}

/**
 * @brief Follows a sensed line.
 * @param prevState previous position of the line in relation to the sensors
 * @return state of line
 */
int followLine(int prevState)
{
    int state = stateSense(prevState);
    switch (state)
    {
    case LINE_OFF_LEFT:
        turnOff(LEFT);
        break;
    case LINE_OFF_RIGHT:
        turnOff(RIGHT);
        break;
    case LINE_ON_LEFT:
        turnOn(LEFT);
        break;
    case LINE_ON_RIGHT:
        turnOn(RIGHT);
        break;
    default:
        straight(F_POWER);
        break;
    }
    return state;
}

/**
 * @brief Identifies the state of the sensed line.
 * @param prev previous state of the line
 * @return state of line
 */
int stateSense(int prev)
{
    double left = optol.Value();
    double middle = optom.Value();
    double right = optor.Value();
    if (left > L_DIV)
    {
        return LINE_ON_LEFT;
    }
    else if (right > R_DIV)
    {
        return LINE_ON_RIGHT;
    }
    else if (left < L_DIV && prev == LINE_ON_LEFT)
    {
        return LINE_OFF_LEFT;
    }
    else if (right < R_DIV && prev == LINE_ON_RIGHT)
    {
        return LINE_OFF_RIGHT;
    }
    else
    {
        return LINE_MIDDLE;
    }
}

/**
 * @brief Turns the robot from off the line back on to the line.
 * @param dir direction the robot needs to turn, [-1 for left; 1 for right]
 */
void turnOff(int dir)
{
    while (optom.Value() < M_DIV)
    {
        switch (dir)
        {
        case LEFT:
            leftMotor.Stop();
            rightMotor.SetPercent(F_POWER);
            break;
        case RIGHT:
            rightMotor.Stop();
            leftMotor.SetPercent(F_POWER);
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Centers the robot on the line.
 * @param dir direction the robot needs to turn, [-1 for left; 1 for right]
 */
void turnOn(int dir)
{
    while (optom.Value() < M_DIV)
    {
        switch (dir)
        {
        case LEFT:
            leftMotor.SetPercent(F_POWER / 2);
            rightMotor.SetPercent(F_POWER);
            break;
        case RIGHT:
            rightMotor.SetPercent(F_POWER / 2);
            leftMotor.SetPercent(F_POWER);
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Drives the robot straight ahead.
 */
void straight(int percent)
{
    rightMotor.SetPercent(percent);
    leftMotor.SetPercent(percent);
}

/**
 * @brief Moves robot a specified amount.
 * @param percent motor speed (if < 0 robot will drive backwards)
 * @param dist distance for the bot to travel (inches)
 */
void forward(int percent, double dist)
{
    int counts = UNIT_COUNTS * dist;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    straight(percent);

    while ((encoderR.Counts() + encoderL.Counts()) / 2. < counts)
        ;

    stop();
}

/**
 * @brief Turns the robot in place a specified amount.
 * @param percent motor speed
 * @param deg degrees for the bot to turn
 * @param dir direction the robot turns (-1 for left, 1 for right)
 */
void turn(int percent, int deg, int dir)
{
    int counts = UNIT_COUNTS * (BOT_WIDTH * PI) * deg / DEGREES;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    rightMotor.SetPercent(percent * -1 * dir);
    leftMotor.SetPercent(percent * dir);

    while (encoderR.Counts() + encoderL.Counts() < 2 * counts)
        ;

    stop();
}

/**
 * @brief Stops the robot.
 */
void stop()
{
    rightMotor.Stop();
    leftMotor.Stop();
    Sleep(0.25);
}
