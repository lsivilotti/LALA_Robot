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

/*Generic constants–––––––––––––––––––––––––––––––––––––––––––*/

/*Team string identifier*/
#define IDENTIFIER "1130D6KKR"
/*Width of robot in inches*/
#define BOT_WIDTH 7.25
/*Value of pi*/
#define PI 3.141592
/*Degrees in a circle*/
#define DEGREES 360.

/*Direction constants–––––––––––––––––––––––––––––––––––––––––––*/

/*Controls robot traveling direction*/
enum Direction
{
    LEFT = -1,
    RIGHT = 1,
    BACKWARDS = -1,
    FORWARDS = 1
};

/*Switch constants–––––––––––––––––––––––––––––––––––––––––––*/

/*When sensor is active*/
#define ON 0
/*When sensor is inactive*/
#define OFF 1

/*Motor constants–––––––––––––––––––––––––––––––––––––––––––*/

/*Motor voltage*/
#define VOLTAGE 9.0
/*Forward power*/
#define F_POWER 25.
/*Reverse power*/
#define B_POWER -25.
/*Correction for right motor (make very very small changes)*/
#define RIGHT_MOTOR_CORRECTION 1
/*Correction for left motor (make very very small changes)*/
#define LEFT_MOTOR_CORRECTION 1

/*Encoder constants–––––––––––––––––––––––––––––––––––––––––––*/

/*Counts per inch traveled*/
#define UNIT_COUNTS 40.489

/*Optosensor constants–––––––––––––––––––––––––––––––––––––––––––*/

/*Minimum optosensor value when line is read on left*/
#define L_DIV 1.45
/*Minimum optosensor value when line is read in middle*/
#define M_DIV 1.45
/*Minimum optosensor value when line is read on right*/
#define R_DIV 1.45

/*CdS constants (lower limits)–––––––––––––––––––––––––––––––––––––––––––*/
/**
 * @todo change limits with more testing to get more precise bands
 */

/*Lower limit of no light*/
#define NONE_LIM 1.9
/*Lower limit of blue light*/
#define BLUE_LIM 1.5
/*Lower limit of red light*/
#define RED_LIM 1.2
/*Possible color values {FIRE = RED, WATER = BLUE, NONE = Neither}*/
enum Color
{
    FIRE,
    WATER,
    NONE
};

/*State variables–––––––––––––––––––––––––––––––––––––––––––*/

/*Where the line is under the optosensor*/
enum Line
{
    /*Line is to the left of the robot*/
    LINE_OFF_LEFT,
    /*Line is being sensed by left optosensor*/
    LINE_ON_LEFT,
    /*Line is only sensed by middle optosensor*/
    LINE_MIDDLE,
    /*Line is being sensed by right optosensor*/
    LINE_ON_RIGHT,
    /*Line is to the right of the robot*/
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
FEHMotor rightMotor(FEHMotor::Motor0, VOLTAGE);
/*Motor powering left wheel*/
FEHMotor leftMotor(FEHMotor::Motor1, VOLTAGE);
/*Right encoder*/
DigitalEncoder encoderR(FEHIO::P0_7);
/*Left encoder*/
DigitalEncoder encoderL(FEHIO::P0_0);

/*Methods–––––––––––––––––––––––––––––––––––––––––––*/

void activateHumidifier(Color);
float findCDS();
Color getCDS(float);
float motorSpeed(float);
void findLine();
Line followLine(Line);
Line followLine(Line, float);
Line stateSense(Line);
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
    LCD.Clear();
    while (cds.Value() > NONE_LIM)
    {
        LCD.Write(cds.Value());
        Sleep(0.25);
    }
    forward(50, 2);
    forward(B_POWER, 1);
    rotate(F_POWER, 135, LEFT);
    forward(F_POWER, 40.);
    rotate(F_POWER, 90., LEFT);
    forward(B_POWER, 4.5);
    Color color = Color::NONE;
    activateHumidifier(color);
}

/**
 * @brief Navigates to and presses humidifier button.
 *
 * @param col color of the humidifer light
 */
void activateHumidifier(Color col)
{
    /**
     * @todo change instructions to use optosensors
     */
    while (col == NONE)
    {
        /*looks for then gets the color of LED*/
        col = getCDS(findCDS());
        /*actions based on color read by CdS*/
        switch (col)
        {
            /*if red*/
        case FIRE:
            rotate(F_POWER, 90, RIGHT);
            forward(F_POWER, 1);
            rotate(F_POWER, 90, LEFT);
            forward(F_POWER * 1.5, 7);
            break;
            /*if blue*/
        case WATER:
            rotate(F_POWER, 90, LEFT);
            forward(F_POWER, 1);
            rotate(F_POWER, 90, RIGHT);
            forward(F_POWER * 1.5, 7);
            break;
            /*if value is outside red or blue*/
        case NONE:
            LCD.WriteLine("404");
            forward(B_POWER, 21);
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Positions robot over light source.
 *
 * @return value read by CdS
 */
float findCDS()
{
    /**
     * @todo Change to use optosensors
     */
    forward(F_POWER, 21.);
    float minCDS = 3.3;
    float val = cds.Value();
    int count = 0;
    int turned = 0;
    while (val - .1 < minCDS && turned < DEGREES)
    {
        int round = count % 11;
        if (val < minCDS)
        {
            minCDS = val;
        }
        if (round < 4)
        {
            forward(B_POWER, 0.5);
        }
        else if (round > 4 && round < 10)
        {
            forward(F_POWER, 0.5);
        }
        else
        {
            rotate(F_POWER, 10, RIGHT);
            turned += 10;
        }
        val = cds.Value();
        count++;
        LCD.Write("Loop iteration i=");
        LCD.Write(count);
        LCD.Write(", val=");
        LCD.Write(val);
        LCD.Write(", minCdS=");
        LCD.WriteLine(minCDS);
    }
    rotate(F_POWER, turned % (int)DEGREES, LEFT);
    return minCDS;
}

/**
 * @brief Determines the color of the light sensed by CdS.
 *
 * @param val value read by CdS
 * @return a value in Color {NONE, WATER, FIRE}
 */
Color getCDS(float val)
{
    /*Compares val to Color limits, returning state of color*/
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
    /*Whether the robot should turn right*/
    bool right = true;

    /*Positions robot to begin search*/
    rotate(F_POWER, 90., LEFT);

    /*Loops while line is not sensed by any sensors*/
    while ((optol.Value() < L_DIV || optom.Value() < M_DIV || optor.Value() < R_DIV))
    {
        if (right)
        {
            turn(F_POWER, 180., RIGHT);
        }
        else
        {
            turn(F_POWER, 180., LEFT);
        }
        right = !right;
    }
}

/**
 * @brief Follows a sensed line.
 *
 * @param prevState previous position of the line in relation to the sensors
 * @return state of line
 */
Line followLine(Line prevState)
{
    /*State of line under robot*/
    Line state = stateSense(prevState);
    /*Instruction for each state*/
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
    case LINE_MIDDLE:
        straight(F_POWER);
        break;
    default:
        break;
    }
    return state;
}

/**
 * @brief Follows a sensed line for a specified distance.
 *
 * Will continue driving even after line is no longer sensed.
 *
 * @param prevState previous position of the line in relation to the sensors
 * @return state of line
 */
Line followLine(Line prevState, float dist)
{
    /*
    Translates distance to counts

    Explanation of multiplication by 2 is in while loop
    */
    int counts = 2 * UNIT_COUNTS * dist;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    Line state = prevState;

    /*
    Takes the average of the encoder counts and compares to (desired) counts

    Division by 2 for average moved to other side of inequality and into
    calculation of counts to save fractions of a second in computation time each loop
    */
    while (encoderL.Counts() + encoderR.Counts() < counts /*|| (optol.Value() < L_DIV || optom.Value() < M_DIV || optor.Value() < R_DIV)*/)
    {
        state = followLine(prevState);
    }
    return state;
}

/**
 * @brief Identifies the state of the sensed line.
 *
 * @param prev previous state of the line
 * @return state of line
 */
Line stateSense(Line prev)
{
    /*Value of left optosensor*/
    float left = optol.Value();
    /*Value of middle optosensor*/
    float middle = optom.Value();
    /*Value of right optosensor*/
    float right = optor.Value();

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
 *
 * @param percent motor speed
 * @param dir direction the robot needs to turn, [-1 for left; 1 for right]
 */
void turnOff(float percent, Direction dir)
{
    /*Actual power directed to motors to produce desired speed*/
    float speed = motorSpeed(percent);
    /*Directs robot in direction to turn*/
    switch (dir)
    {
    case LEFT:
        leftMotor.Stop();
        rightMotor.SetPercent(speed);
        /*Loops while the middle optosensor doesn't "see" the line*/
        while (optom.Value() < M_DIV)
            ;
        break;
    case RIGHT:
        rightMotor.Stop();
        leftMotor.SetPercent(speed);
        /*Loops while the middle optosensor doesn't "see" the line*/
        while (optom.Value() < M_DIV)
            ;
        break;
    default:
        break;
    }
}

/**
 * @brief Centers the robot on the line.
 *
 * @param percent motor speed
 * @param dir direction the robot needs to turn, [-1 for left; 1 for right]
 */
void turnOn(float percent, Direction dir)
{
    /*Actual power directed to motors to produce desired speed*/
    float speed = motorSpeed(percent);
    /*Directs robot in direction to turn*/
    switch (dir)
    {
    case LEFT:
        leftMotor.SetPercent(speed / 2);
        rightMotor.SetPercent(speed);
        /*Loops while the middle optosensor doesn't "see" the line and the left sensor is on the line*/
        while (optom.Value() < M_DIV && optol.Value() > L_DIV)
            ;
        break;
    case RIGHT:
        rightMotor.SetPercent(speed / 2);
        leftMotor.SetPercent(speed);
        /*Loops while the middle optosensor doesn't "see" the line and the right sensor is on the line*/
        while (optom.Value() < M_DIV && optor.Value() > R_DIV)
            ;
        break;
    default:
        break;
    }
}

/**
 * @brief Drives the robot straight ahead.
 *
 * @param percent speed the robot moves
 */
void straight(float percent)
{
    /*Moves each motor forward at same speed*/
    rightMotor.SetPercent(percent * RIGHT_MOTOR_CORRECTION);
    leftMotor.SetPercent(percent * LEFT_MOTOR_CORRECTION);
}

/**
 * @brief Moves robot a specified amount.
 *
 * @param percent motor speed (if < 0 robot will drive backwards)
 * @param dist distance for the bot to travel (inches)
 */
void forward(float percent, double dist)
{
    /*
    Translates distance to counts

    Explanation of multiplication by 2 is in while loop
    */
    int counts = 2 * (UNIT_COUNTS * dist);

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    /*Actual power directed to motors to produce desired speed*/
    float speed = motorSpeed(percent);

    straight(speed);

    /*
    Takes the average of the encoder counts and compares to (desired) counts

    Division by 2 for average moved to other side of inequality and into
    calculation of counts to save fractions of a second in computation time each loop
    */
    while (encoderR.Counts() + encoderL.Counts() < counts)
        ;

    stop();
}

/**
 * @brief Rotates the robot in place a specified amount.
 *
 * @param percent motor speed
 * @param deg degrees for the bot to turn
 * @param dir direction the robot turns
 */
void rotate(float percent, float deg, Direction dir)
{
    /*
    Calculates circumference the wheels travel (BOT_WIDTH treated as diameter)
    Multiples by fraction of the circle desired
    Translates to counts

    Explanation of multiplication by 2 is in while loop
    */
    float circumference = BOT_WIDTH * PI;
    float circleFraction = deg / DEGREES;
    int counts = 2 * (UNIT_COUNTS * (circumference * circleFraction));

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    /*Actual power directed to motors to produce desired speed*/
    float speed = motorSpeed(percent * dir);

    /*Starts each motor moving in different directions*/
    rightMotor.SetPercent(RIGHT_MOTOR_CORRECTION * speed * -1);
    leftMotor.SetPercent(LEFT_MOTOR_CORRECTION * speed);

    /*
    Takes the average of the encoder counts and compares to (desired) counts

    Division by 2 for average moved to other side of inequality and into
    calculation of counts to save fractions of a second in computation time each loop
    */
    while (encoderR.Counts() + encoderL.Counts() < counts)
        ;

    stop();
}

/**
 * @brief Turns the robot a specified amount, breaking out if it encounters a pathing line.
 *
 * @param percent motor speed
 * @param deg degrees for the bot to turn
 * @param dir direction the robot turns (-1 for left, 1 for right)
 */
void turn(float percent, float deg, Direction dir)
{
    /*
    Calculates circumference the wheels travel (BOT_WIDTH treated as radius)
    Multiples by fraction of the circle desired
    Translates to counts
    */
    float circumference = 2 * BOT_WIDTH * PI;
    float circleFraction = deg / DEGREES;
    int counts = (UNIT_COUNTS * (circumference * circleFraction));

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    /*Actual power directed to motors to produce desired speed*/
    float speed = motorSpeed(percent);

    /*Activates motors to turn the robot*/
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

    /*
    Sums encoder counts (one will be zero) and compares to counts

    Breaks out if optosensors encounters a line
    */
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
    /*Ensures robot comes to a complete stop*/
    Sleep(0.25);
}
