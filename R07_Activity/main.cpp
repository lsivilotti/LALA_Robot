/**
 * @file main.cpp
 * @brief Code for Activity R07
 * @date 02/15/2025
 * @author Luca Sivilotti
 */

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>

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
#define F_POWER 12.
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
DigitalInputPin bl(FEHIO::P2_7);
/*Motor powering right wheel*/
FEHMotor rightMotor(FEHMotor::Motor0, VOLTAGE);
/*Motor powering left wheel*/
FEHMotor leftMotor(FEHMotor::Motor1, VOLTAGE);
/*Right encoder*/
DigitalEncoder encoderR(FEHIO::P0_0);
/*Left encoder*/
DigitalEncoder encoderL(FEHIO::P0_1);

/*Methods*/
int followLine(int);
int stateSense(int);
void turnOff(int);
void turnOn(int);
void straight();
void forward(int, int);
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
    /*
    Part 1
    */
    // int prevState = LINE_MIDDLE;
    // while (fr.Value() == OFF && fl.Value() == OFF)
    // {
    //     prevState = followLine(prevState);
    // }
    // stop();

    /*
    Part 2: Step 4
    */
    int encoderCounts = UNIT_COUNTS * 6;
    int motorPercent = 25;
    // int motorPercent = 40;
    // int motorPercent = 60;
    forward(motorPercent, encoderCounts);

    /*
    Part 2: Step 5
    */
    // int encoderCounts = UNIT_COUNTS * (2 * 3.141592 * 3) / 4;
    // int motorPercent = 25;
    // int motorPercent = 40;
    // int motorPercent = 60;
    // turn(motorPercent, encoderCounts, RIGHT);

    /*
    Part 2: Step 6
    */
    // int encoderCounts = UNIT_COUNTS * (2 * 3.141592 * 3) / 4;
    // int motorPercent = 25;
    // int motorPercent = 40;
    // int motorPercent = 60;
    // turn(motorPercent, encoderCounts, LEFT);

    /*
    Part 2: Step 7
    */
    // int encoderCounts = UNIT_COUNTS * 6;
    // int encoderCounts = UNIT_COUNTS * (2 * 3.141592 * 3) / 4;
    // int motorPercent = 25;
    // int motorPercent = 40;
    // int motorPercent = 60;
    // forward(motorPercent, UNIT_COUNTS * 14);
    // turn(motorPercent, UNIT_COUNTS * (2 * 3.141592 * 3) / 4, LEFT);
    // forward(motorPercent, UNIT_COUNTS * 10);
    // turn(motorPercent, UNIT_COUNTS * (2 * 3.141592 * 3) / 4, RIGHT);
    // forward(motorPercent, UNIT_COUNTS * 4);

    int actualCountsL = encoderL.Counts();
    int actualCountsR = encoderR.Counts();
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
        turnOff(LEFT);
        break;
    case LINE_ON_RIGHT:
        turnOff(RIGHT);
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

/**
 * @brief Moves robot forwards a specified amount.
 * @param percent motor speed
 * @param counts counts of the encoder on the wheel
 */
void forward(int percent, int counts)
{
    encoderR.ResetCounts();
    encoderL.ResetCounts();

    rightMotor.SetPercent(percent);
    leftMotor.SetPercent(percent);

    while (encoderR.Counts() < counts && encoderL.Counts() < counts)
        ;

    stop();
}

/**
 * @brief Turns the robot in place a specified amount.
 * @param percent motor speed
 * @param counts counts of the encoder on the wheel
 * @param dir direction the robot turns (-1 for left, 1 for right)
 */
void turn(int percent, int counts, int dir)
{
    encoderR.ResetCounts();
    encoderL.ResetCounts();

    rightMotor.SetPercent(percent * -1 * dir);
    leftMotor.SetPercent(percent * dir);

    while (encoderR.Counts() < counts && encoderL.Counts() < counts)
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
    Sleep(0.5);
}