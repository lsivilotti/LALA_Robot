/**
 * @file main.cpp
 * @brief Code for Activity R07
 * @date 02/14/2025
 * @author Luca Sivilotti
 */

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>

/*Switch constants*/
#define ON 0
#define OFF 1

/*Motor constants*/
#define VOLTAGE 9.0
#define F_POWER 12.
#define B_POWER -12.

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

/*Optosensor*/
AnalogInputPin optol(FEHIO::P1_0);
AnalogInputPin optom(FEHIO::P1_1);
AnalogInputPin optor(FEHIO::P1_2);
/*Front right switch*/
DigitalInputPin fr(FEHIO::P2_0);
/*Front left switch*/
DigitalInputPin fl(FEHIO::P2_1);
/*Back right switch*/
DigitalInputPin br(FEHIO::P2_2);
/*Back left switch*/
DigitalInputPin bl(FEHIO::P2_7);
/*Motor powering right wheel*/
FEHMotor rightMotor(FEHMotor::Motor0, VOLTAGE);
/*Motor powering left wheel*/
FEHMotor leftMotor(FEHMotor::Motor1, VOLTAGE);

/*Methods*/
int followLine(int);
int stateSense(int);
void turnOff(int);
void turnOn(int);
void straight();

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
    int prevState = LINE_MIDDLE;
    while (fr.Value() == OFF && fl.Value() == OFF)
    {
        prevState = followLine(prevState);
    }
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
        turnOff(-1);
        break;
    case LINE_OFF_RIGHT:
        turnOff(1);
        break;
    case LINE_ON_LEFT:
        turnOff(-1);
        break;
    case LINE_ON_RIGHT:
        turnOff(1);
        break;
    default:
        straight();
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
        case -1:
            leftMotor.Stop();
            rightMotor.SetPercent(F_POWER);
            break;
        case 1:
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
        case -1:
            leftMotor.SetPercent(F_POWER / 2);
            rightMotor.SetPercent(F_POWER);
            break;
        case 1:
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
void straight()
{
    rightMotor.SetPercent(F_POWER);
    rightMotor.SetPercent(F_POWER);
}