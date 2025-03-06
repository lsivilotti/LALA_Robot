/**
 * @file main.cpp
 * @brief Code for Milestone 2
 * @date 03/07/2025
 * @author Luca Sivilotti
 */

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHBattery.h>
#include <FEHRCS.h>
#include <math.h>

/*Generic constants*/
#define IDENTIFIER "1130D6KKR"
#define BOT_WIDTH 7.25
#define PI 3.141592
#define DEGREES 360.

/*Direction constants*/
enum Direction
{
    LEFT = 1,
    RIGHT = -1,
    BACKWARDS = -1,
    FORWARDS = 1
};

/*Switch constants*/
#define ON 0
#define OFF 1

/*Motor constants*/
#define VOLTAGE 9.0
#define F_POWER 25.
#define B_POWER -25.
#define RIGHT_MOTOR_CORRECTION 1.2
#define LEFT_MOTOR_CORRECTION 1

/*Encoder constants*/
#define UNIT_COUNTS 40.489

/*Optosensor constants*/
#define L_DIV 1.45
#define M_DIV 1.45
#define R_DIV 1.45

/*CdS constants (upper limits)*/
#define NONE_LIM 1.9
#define BLUE_LIM 1.5
#define RED_LIM 1.2
enum Color
{
    FIRE,
    WATER,
    NONE
};

/*State variables*/
enum Line
{
    LINE_OFF_LEFT,
    LINE_ON_LEFT,
    LINE_MIDDLE,
    LINE_ON_RIGHT,
    LINE_OFF_RIGHT
};

/*CdS sensor*/
AnalogInputPin cds(FEHIO::P0_1);
/*Left Optosensor*/
AnalogInputPin optol(FEHIO::P1_0);
/*Middle Optosensor*/
AnalogInputPin optom(FEHIO::P1_1);
/*Right Optosensor*/
AnalogInputPin optor(FEHIO::P1_2);
/*Motor powering right wheel*/
FEHMotor rightMotor(FEHMotor::Motor1, VOLTAGE);
/*Motor powering left wheel*/
FEHMotor leftMotor(FEHMotor::Motor0, VOLTAGE);
/*Right encoder*/
DigitalEncoder encoderR(FEHIO::P0_7);
/*Left encoder*/
DigitalEncoder encoderL(FEHIO::P0_0);

/*Methods*/
void activateHumidifier(Color);
void findCDS();
Color getCDS();
float motorSpeed(float);
void findLine();
int followLine(Line);
int followLine(Line, float);
int stateSense(int);
void turnOff(float, Direction);
void turnOn(float, Direction);
void straight(float);
void forward(float, double);
void rotate(float, float, Direction);
void turn(float, float, Direction);
void stop();

/**
 * @brief Main method.
 */
int main(void)
{
    RCS.InitializeTouchMenu(IDENTIFIER);
    LCD.Clear();
    // float x, y;
    // while (!LCD.Touch(&x, &y))
    //     ;
    // while (LCD.Touch(&x, &y))
    //     ;
    // double start = TimeNow();
    // while (TimeNow() - start < 20)
    // {
    //     LCD.Clear();
    //     LCD.Write(cds.Value());
    //     switch (getCDS())
    //     {
    //     case FIRE:
    //         LCD.Write("RED");
    //         break;
    //     case WATER:
    //         LCD.Write("BLUE");
    //         break;
    //     case NONE:
    //         LCD.Write("NONE");
    //         break;
    //     default:
    //         LCD.Write("NULL");
    //         break;
    //     }
    //     Sleep(0.5);
    // }
    while (cds.Value() > NONE_LIM)
    {
        LCD.Write(cds.Value());
        Sleep(0.25);
    }

    forward(40, 1.9);
    forward(B_POWER, 1);
    rotate(F_POWER, 135, LEFT);
    forward(F_POWER, 38.);
    findLine();
    followLine(LINE_MIDDLE, 5);
    Color color = Color::NONE;
    activateHumidifier(color);
    forward(B_POWER, 15);
    rotate(F_POWER, 90, RIGHT);
    forward(B_POWER, 40);
}

/**
 * @brief Navigates to and presses humidifier button.
 *
 * @param col color of the humidifer light
 */
void activateHumidifier(Color col)
{
    while (col == NONE)
    {
        col = getCDS();
        switch (col)
        {
        case FIRE:
            rotate(F_POWER, 20, RIGHT);
            followLine(Line::LINE_MIDDLE, 5);
            break;
        case WATER:
            rotate(F_POWER, 20, RIGHT);
            followLine(Line::LINE_MIDDLE, 5);
            break;
        case NONE:
            findCDS();
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Positions robot over light source.
 */
void findCDS()
{
    forward(B_POWER, 4);
    followLine(LINE_MIDDLE, 4.);
}

/**
 * @brief Determines the color of the light sensed by CdS.
 *
 * @return a value in Color {NONE, WATER, FIRE}
 */
Color getCDS()
{
    float val = cds.Value();
    if (val > NONE_LIM)
    {
        return NONE;
    }
    else if (val > BLUE_LIM)
    {
        return WATER;
    }
    else if (val > RED_LIM)
    {
        return FIRE;
    }
    else
    {
        return NONE;
    }
}

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
 * @brief Moves the robot in a pattern to find the line.
 */
void findLine()
{
    int count = 0;
    turn(F_POWER, 45., LEFT);
    while ((optol.Value() < L_DIV || optom.Value() < M_DIV || optor.Value() < R_DIV))
    {
        if (count % 2 == 0)
        {
            turn(F_POWER, 90., RIGHT);
        }
        else
        {
            turn(F_POWER, 90., LEFT);
        }
        count++;
    }
}

/**
 * @brief Follows a sensed line.
 *
 * @param prevState previous position of the line in relation to the sensors
 * @return state of line
 */
int followLine(Line prevState)
{
    int state = stateSense(prevState);
    switch (state)
    {
    case LINE_OFF_LEFT:
        turnOff(F_POWER, LEFT);
        break;
    case LINE_OFF_RIGHT:
        turnOff(F_POWER, RIGHT);
        break;
    case LINE_ON_LEFT:
        turnOn(F_POWER, LEFT);
        break;
    case LINE_ON_RIGHT:
        turnOn(F_POWER, RIGHT);
        break;
    default:
        straight(F_POWER);
        break;
    }
    return state;
}

/**
 * @brief Follows a sensed line for a specified distance.
 *
 * @param prevState previous position of the line in relation to the sensors
 * @return state of line
 */
int followLine(Line prevState, float dist)
{
    int counts = 2 * UNIT_COUNTS * dist;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    int state = prevState;

    while (encoderL.Counts() + encoderR.Counts() < counts /*|| (optol.Value() < L_DIV || optom.Value() < M_DIV || optor.Value() < R_DIV)*/)
    {
        state = stateSense(prevState);
        switch (state)
        {
        case LINE_OFF_LEFT:
            turnOff(F_POWER, LEFT);
            break;
        case LINE_OFF_RIGHT:
            turnOff(F_POWER, RIGHT);
            break;
        case LINE_ON_LEFT:
            turnOn(F_POWER, LEFT);
            break;
        case LINE_ON_RIGHT:
            turnOn(F_POWER, RIGHT);
            break;
        default:
            straight(F_POWER);
            break;
        }
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
 * @param percent motor speed
 * @param dir direction the robot needs to turn, [-1 for left; 1 for right]
 */
void turnOff(float percent, Direction dir)
{
    float speed = motorSpeed(percent);
    while (optom.Value() < M_DIV)
    {
        switch (dir)
        {
        case LEFT:
            leftMotor.Stop();
            rightMotor.SetPercent(speed);
            break;
        case RIGHT:
            rightMotor.Stop();
            leftMotor.SetPercent(speed);
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Centers the robot on the line.
 * @param percent motor speed
 * @param dir direction the robot needs to turn, [-1 for left; 1 for right]
 */
void turnOn(float percent, Direction dir)
{
    float speed = motorSpeed(percent);
    while (optom.Value() < M_DIV)
    {
        switch (dir)
        {
        case LEFT:
            leftMotor.SetPercent(speed / 2);
            rightMotor.SetPercent(speed);
            break;
        case RIGHT:
            rightMotor.SetPercent(speed / 2);
            leftMotor.SetPercent(speed);
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Drives the robot straight ahead.
 */
void straight(float percent)
{
    rightMotor.SetPercent(percent * RIGHT_MOTOR_CORRECTION);
    leftMotor.SetPercent(percent * LEFT_MOTOR_CORRECTION);
}

/**
 * @brief Moves robot a specified amount.
 * @param percent motor speed (if < 0 robot will drive backwards)
 * @param dist distance for the bot to travel (inches)
 */
void forward(float percent, double dist)
{
    int counts = 2 * UNIT_COUNTS * dist;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    straight(motorSpeed(percent));

    while (encoderR.Counts() + encoderL.Counts() < counts)
        ;

    stop();
}

/**
 * @brief Rotates the robot in place a specified amount.
 * @param percent motor speed
 * @param deg degrees for the bot to turn
 * @param dir direction the robot turns (-1 for left, 1 for right)
 */
void rotate(float percent, float deg, Direction dir)
{
    int counts = UNIT_COUNTS * (BOT_WIDTH * PI) * deg / DEGREES;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    float speed = motorSpeed(percent * dir);

    rightMotor.SetPercent(speed * -1);
    leftMotor.SetPercent(speed);

    while (encoderR.Counts() + encoderL.Counts() < counts)
        ;

    stop();
}

/**
 * @brief Turns the robot a specified amount, breaking out if it encounters a pathing line.
 * @param percent motor speed
 * @param deg degrees for the bot to turn
 * @param dir direction the robot turns (-1 for left, 1 for right)
 */
void turn(float percent, float deg, Direction dir)
{
    int counts = UNIT_COUNTS * (BOT_WIDTH * PI) * deg / DEGREES;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    float speed = motorSpeed(percent);

    switch (dir)
    {
    case LEFT:
        rightMotor.SetPercent(speed);
        break;
    case RIGHT:
        leftMotor.SetPercent(speed);
        break;
    default:
        break;
    }

    while (encoderR.Counts() + encoderL.Counts() < counts || (optol.Value() < L_DIV || optom.Value() < M_DIV || optor.Value() < R_DIV))
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
