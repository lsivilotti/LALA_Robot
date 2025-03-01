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
void straight();
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
    // int motorPercent = 25;
    // forward(motorPercent, 6);
    // while (!LCD.Touch(&x, &y))
    //     ;
    // while (LCD.Touch(&x, &y))
    //     ;
    // backward(motorPercent, 6);
    // while (!LCD.Touch(&x, &y))
    //     ;
    // while (LCD.Touch(&x, &y))
    //     ;
    // int motorPercent = 40;
    // forward(motorPercent, 6);
    // while (!LCD.Touch(&x, &y))
    //     ;
    // while (LCD.Touch(&x, &y))
    //     ;
    // backward(motorPercent, 6);
    // while (!LCD.Touch(&x, &y))
    //     ;
    // while (LCD.Touch(&x, &y))
    //     ;
    // int motorPercent = 60;
    // forward(motorPercent, 6);
    // while (!LCD.Touch(&x, &y))
    //     ;
    // while (LCD.Touch(&x, &y))
    //     ;
    // backward(motorPercent, 6);
    // while (!LCD.Touch(&x, &y))
    //     ;
    // while (LCD.Touch(&x, &y))
    //     ;
    
    /*
    Part 2: Step 5
    */
    // int motorPercent = 25;
    // int motorPercent = 40;
    // int motorPercent = 60;
    // turn(motorPercent, 90, RIGHT);

    /*
    Part 2: Step 6
    */
    // int motorPercent = 25;
    // int motorPercent = 40;
    // int motorPercent = 60;
    // turn(motorPercent, 90, LEFT);

    /*
    Part 2: Step 7
    */
    int drive = 30;
    int turnSpeed = 40;
    forward(drive, 14);
    turn(turnSpeed, 90, LEFT);
    forward(drive, 10);
    turn(turnSpeed, 90, RIGHT);
    forward(drive, 4);

    // int actualCountsL = encoderL.Counts();
    // int actualCountsR = encoderR.Counts();
    // int expected = UNIT_COUNTS * (BOT_WIDTH * PI) * 90 / DEGREES;
    // LCD.Write("Expected counts: ");
    // LCD.Write(expected);
    // LCD.Write("Right counts: ");
    // LCD.Write(actualCountsR);
    // LCD.Write("Left counts: ");
    // LCD.Write(actualCountsL);
    // FEHFile *ptr = SD.FOpen("output.txt", "w");
    // SD.FPrintf(ptr, "Expected counts: %d\n", expected);
    // SD.FPrintf(ptr, "Actual counts; Right: %d, Left: %d", actualCountsR, actualCountsL);
    // SD.FClose(ptr);
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
void straight()
{
    rightMotor.SetPercent(F_POWER);
    rightMotor.SetPercent(F_POWER);
}

/**
 * @brief Moves robot forwards a specified amount.
 * @param percent motor speed
 * @param dist distance for the bot to travel
 */
void forward(int percent, double dist)
{
    int counts = UNIT_COUNTS * dist;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    rightMotor.SetPercent(percent);
    leftMotor.SetPercent(percent);

    while ((encoderR.Counts() + encoderL.Counts()) / 2. < counts)
        ;

    stop();
}

/**
 * @brief Moves robot backwards a specified amount.
 * @param percent motor speed
 * @param dist distance for the bot to travel
 */
void backward(int percent, double dist)
{
    int counts = UNIT_COUNTS * dist;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    leftMotor.SetPercent(-1 * percent);
    rightMotor.SetPercent(-1 * percent);

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
    Sleep(0.5);
}