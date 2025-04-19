/**
 * @file main.cpp
 * @brief Code for Full Robot Course
 * @date 04/11/2025
 * @author LALA
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
#define F_POWER 35.
/*Reverse power*/
#define B_POWER -35.
/*Correction for right motor (make very very small changes)*/
#define RIGHT_MOTOR_CORRECTION 1.05
/*Correction for left motor (make very very small changes)*/
#define LEFT_MOTOR_CORRECTION 1

/*Encoder constants–––––––––––––––––––––––––––––––––––––––––––*/

/*Counts per motor rotation*/
#define ROTATION_COUNTS 318
/*Diameter of wheel*/
#define WHEEL_DIAMETER 2.5
/*Counts per inch traveled*/
#define UNIT_COUNTS ROTATION_COUNTS / (WHEEL_DIAMETER * M_PI)

/*Optosensor constants–––––––––––––––––––––––––––––––––––––––––––*/

/*Minimum optosensor value when line is read on left*/
#define L_DIV 2.94
/*Minimum optosensor value when line is read in middle*/
#define M_DIV 2.99
/*Minimum optosensor value when line is read on right*/
#define R_DIV 2.95

/*CdS constants –––––––––––––––––––––––––––––––––––––––––––––––––––*/
/**
 * @todo change limits with more testing to get more precise bands
 */

/*Lower and upper limits read from color*/
struct CdSLimits
{
    float maxOutput = 3.3;
    float lightOffMin = 1.5;
    float blueMax = 1.4;
    float blueMin = 1.1;
    float redMax = 0.9;
    float redMin = 0.4;
};
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
    OFF_LEFT,
    /*Line is being sensed by left optosensor*/
    ON_LEFT,
    /*Line is only sensed by middle optosensor*/
    MIDDLE,
    /*Line is being sensed by right optosensor*/
    ON_RIGHT,
    /*Line is to the right of the robot*/
    OFF_RIGHT
};

/*Servo Constants –––––––––––––––––––––––––––––––––––––––––––––––––––*/

/*Degrees motor turns in 1 second at speed 10*/
#define DEG_PER_SEC 42

/*CdS sensor | Port: (1,0)*/
AnalogInputPin cds(FEHIO::P1_0);
/*Left Optosensor | Port: (0,0)*/
AnalogInputPin optol(FEHIO::P0_0);
/*Middle Optosensor | Port: (0,1)*/
AnalogInputPin optom(FEHIO::P0_1);
/*Right Optosensor | Port: (0,2)*/
AnalogInputPin optor(FEHIO::P0_2);
/*Motor powering right wheel | Port: Motor 0*/
FEHMotor rightMotor(FEHMotor::Motor0, VOLTAGE);
/*Motor powering left wheel | Port: Motor 1*/
FEHMotor leftMotor(FEHMotor::Motor1, VOLTAGE);
/*Right encoder | Port: (0,5)*/
DigitalEncoder encoderR(FEHIO::P0_5);
/*Left encoder | Port: (3,1)*/
DigitalEncoder encoderL(FEHIO::P3_1);
/*Robot Servo | Port: Motor 2*/
FEHMotor vex(FEHMotor::Motor2, 7.2);

/*Methods–––––––––––––––––––––––––––––––––––––––––––*/

/**
 * @brief Navigates to and presses humidifier button.
 *
 * @param col color of the humidifer light
 */
void activateHumidifier(Color col);

/**
 * @brief Gets LALA from the crate to the position to hit the lever
 * closest to the crate.
 */
void crateToLever();

/**
 * @brief Moves robot a specified distance.
 *
 * @param percent motor speed (if < 0 robot will drive backwards)
 * @param dist distance for the bot to travel (inches)
 */
void drive(float percent, double dist);

/**
 * @brief Positions robot over light source.
 *
 * @return value read by CdS
 */
float findCDS();

/**
 * @brief Moves the robot in a pattern to find the line.
 */
void findLine();

/**
 * Navigates from compost to end button.
 */
void finish();

/**
 * @brief Follows a sensed line.
 *
 * @param percent motor speed
 * @param prevState previous position of the line in relation to the sensors
 * @return state of line
 */
Line followLine(float percent, Line prevState);

/**
 * @brief Follows a sensed line for a specified distance.
 *
 * Will continue driving even after line is no longer sensed.
 *
 * @param percent motor speed
 * @param dist distance for bot to travel (inches)
 * @param prevState previous position of the line in relation to the sensors
 * @return state of line
 */
Line followLine(float percent, float dist, Line prevState);

/**
 * @brief Determines the color of the light sensed by CdS.
 *
 * @param val value read by CdS
 * @return a value in Color {NONE, WATER, FIRE}
 */
Color getCDS(float val);

/**
 * @brief Gets LALA from where it finishes with humidifier to the start
 * of the window.
 */
void humidifierToWindow();

/**
 * @brief Flips fertilizer lever down using static ramp on the back of LALA.
 */
void leverD();

/**
 * @brief Flips fertilizer lever down then up.
 */
void levers();

/**
 * @brief Flips fertilizer lever up using static ramp on the back of LALA.
 */
void leverU();

/**
 * @brief Gets LALA from where it finishes with fertilizer to the start
 * of the humidifier.
 */
void leverToHumidifier();

/**
 * @brief Lifts apples from stump.
 */
void liftApples();

/**
 * @brief Calculates actual motor speed based on remaining battery power.
 *
 * @param percent desired motor percent
 * @return actual motor percent
 */
float motorSpeed(float percent);

/**
 * @brief Opens/closes window.
 *
 * @param dir direction of opening/closing {FORWARDS = open, BACKWARDS = close}
 */
void moveWindow(Direction dir);

/**
 * @brief Whole process of opening and closing window.
 */
void openCloseWindow();

/**
 * @brief Rotates the robot in place a specified amount.
 *
 * @param percent motor speed
 * @param deg degrees for the bot to turn
 * @param dir direction the robot turns
 */
void rotate(float percent, float deg, Direction dir);

/**
 * @brief Places apples in crate (can be changed to table later).
 */
void setApples();

/**
 * @brief Spins compost bin using vex motor and pulley system.
 *
 * @param dir direction the compost is spun (FORWARDS = ccw, BACKWARDS = cw)
 *
 * @attention initial spin of compost should be BACKWARDS, returning compost
 * to original position is FORWARDS
 */
void spinCompost(Direction dir);

/**
 * @brief Identifies the state of the sensed line.
 *
 * @param prev previous state of the line
 * @return state of line
 */
Line stateSense(Line prev);

/**
 * @brief Stops the robot.
 */
void stop();

/**
 * @brief Drives the robot straight ahead.
 *
 * @param percent speed the robot moves
 */
void straight(float percent);

/**
 * Navigates from start area to apple pick-up.
 */
void toApples();

/**
 * @brief Turns vex motor the given degrees.
 *
 * @param degree degrees for vex motor to turn lever
 * @attention Whatever degree the motor is at when the method is call is the new "zero" degree.
 * @attention Positive degree is cw, negative is ccw (looking from the back)
 */
void toDegree(float degree);

/**
 * @brief Turns vex motor the given degrees at given power.
 *
 * @param percent input power for vex motor to turn lever
 * @param degree degrees for vex motor to turn lever
 * @attention Whatever degree the motor is at when the method is call is the new "zero" degree.
 * @attention Positive degree is cw, negative is ccw (looking from the back)
 */
void toDegree(double percent, double degree);

/**
 * Used for testing
 */
void toDegree(float degree, int perSec);

/**
 * Brings apple bucket from stump to crate.
 */
void transportApples();

/**
 * @brief Turns the robot a specified amount.
 *
 * @param percent motor speed
 * @param deg degrees for the bot to turn
 * @param dir direction the robot turns
 */
void turn(float percent, float deg, Direction dir);

/**
 * @brief Turns the robot from off the line back on to the line.
 *
 * @param percent motor speed
 * @param dir direction the robot needs to turn
 */
void turnOff(float percent, Direction dir);

/**
 * @brief Centers the robot on the line.
 *
 * @param percent motor speed
 * @param dir direction the robot needs to turn
 */
void turnOn(float percent, Direction dir);

/**
 * @brief Repositions robot on other side of window handle.
 */
void windowReposition();

/**
 * @brief Gets LALA from where it finishes with window to the compost.
 */
void windowToCompost();

/**
 * Main method.
 *
 * Start conditions:
 * - LALA facing away from button
 * - rotating arm left
 * - light beneath is off
 * - connected to RCS
 */
int main(void)
{
    RCS.InitializeTouchMenu(IDENTIFIER);
    CdSLimits lims;
    LCD.SetBackgroundColor(RED);
    LCD.Clear();
    LCD.SetBackgroundColor(BLACK);
    LCD.WriteLine(Battery.Voltage());
    while (cds.Value() > lims.lightOffMin)
    {
        LCD.Write(cds.Value());
        Sleep(0.25);
    }

    /*
     * compost - consistent
     */
    drive(B_POWER, 4.);
    drive(F_POWER, 8.);
    rotate(F_POWER, 45., Direction::LEFT);
    drive(F_POWER, 18);
    turn(B_POWER, 45, Direction::LEFT);
    turn(B_POWER, 45, Direction::RIGHT);
    /*Spins compost*/
    /*to rotate compost 300˚: vex needs to rotate 2.1877... times
    AND (facing from the back) vex needs to rotate ccw*/
    spinCompost(Direction::BACKWARDS);
    Sleep(1);
    spinCompost(Direction::FORWARDS);
    /*
     * apples - consistent
     */
    turn(F_POWER, 90., Direction::RIGHT);
    turn(F_POWER, 15., Direction::LEFT);
    toDegree(F_POWER, 400.);
    turn(B_POWER, 10., Direction::LEFT);
    /*
     to humidifier - not bad
     */
    turn(B_POWER, 90., Direction::LEFT);
    drive(F_POWER, 36.);
    drive(B_POWER, 1.);
    rotate(F_POWER, 90., Direction::LEFT);
    drive(F_POWER, 42.);
    drive(B_POWER, 1.5);
    rotate(F_POWER, 90., Direction::LEFT);
    drive(B_POWER, 10.);

    /*
    humidifier - not consistent
    */
    activateHumidifier(Color::NONE);

    /*
    window - to test
    */
    drive(B_POWER, 6.);
    rotate(F_POWER, 90., Direction::LEFT);
    turn(F_POWER, 90., Direction::RIGHT);
    if (RCS.isWindowOpen() == ON)
    {
        drive(F_POWER, 3.);
    }
    rotate(F_POWER, 20., Direction::RIGHT);

    /*
    back to button
    */
    drive(B_POWER, 24.);
    drive(F_POWER, 1.);
    rotate(F_POWER, 90., Direction::LEFT);
    drive(F_POWER, 60.);
    turn(2 * F_POWER, 90., Direction::LEFT);
}

void finish()
{
    turn(F_POWER, 10., Direction::RIGHT);
    turn(F_POWER, 10., Direction::LEFT);
    drive(F_POWER, 12.);
    drive(B_POWER, 30);
    rotate(F_POWER, 45., Direction::RIGHT);
    drive(B_POWER, 10.);
    drive(B_POWER * 2, 1.);
}

void toApples()
{
    drive(F_POWER, 21.);
    rotate(F_POWER, 45., Direction::LEFT);
    followLine(F_POWER, 6., Line::MIDDLE);
}

void transportApples()
{
    turn(B_POWER, 90., Direction::LEFT);
    turn(B_POWER, 90., Direction::RIGHT);
    drive(B_POWER, 30.);
    drive(F_POWER, 3.);
    rotate(F_POWER, 90., Direction::RIGHT);
    drive(F_POWER * 1.25, 36.);
    rotate(F_POWER, 90., Direction::LEFT);
    drive(B_POWER, 6.);
    drive(F_POWER, 10.);
    rotate(F_POWER, 90., Direction::RIGHT);
    followLine(F_POWER, 18., Line::MIDDLE);
}

void windowToCompost()
{
    turn(B_POWER, 90., Direction::RIGHT);
    turn(B_POWER, 90., Direction::LEFT);
    drive(B_POWER, 18.);
    drive(F_POWER, 2.);
    rotate(F_POWER, 90., Direction::LEFT);
    drive(F_POWER, 24.);
    rotate(F_POWER, 90., Direction::RIGHT);
    drive(F_POWER, 12.);
    rotate(F_POWER, 90., Direction::RIGHT);
    drive(B_POWER, 12.);
    drive(F_POWER, 8.);
    rotate(F_POWER, 90., Direction::LEFT);
    drive(F_POWER, 12.);
}

void humidifierToWindow()
{
    drive(B_POWER, 24.);
    rotate(F_POWER, 30., Direction::LEFT);
    drive(F_POWER, 12);
    turn(F_POWER, 30., Direction::RIGHT);
}

void activateHumidifier(Color col)
{
    followLine(F_POWER, 14., Line::MIDDLE);
    int c = 0;
    while (col == Color::NONE && c < 4)
    {
        /*looks for then gets the color of LED*/
        col = getCDS(findCDS());
        /*actions based on color read by CdS*/
        if (c == 3)
        {
            col = Color::WATER;
        }
        switch (col)
        {
        /*if red*/
        case Color::FIRE:
        {
            rotate(F_POWER, 90, Direction::RIGHT);
            drive(F_POWER, 1);
            rotate(F_POWER, 90, Direction::LEFT);
            followLine(F_POWER, 6., Line::MIDDLE);
            drive(2 * F_POWER, 1.);
            drive(B_POWER, 2.);
            rotate(F_POWER, 90., Direction::RIGHT);
            drive(B_POWER, 6.);
            rotate(F_POWER, 90, Direction::LEFT);
            break;
        }
        /*if blue*/
        case Color::WATER:
        {
            rotate(F_POWER, 90, Direction::LEFT);
            drive(F_POWER, 1);
            rotate(F_POWER, 90, Direction::RIGHT);
            followLine(F_POWER, 6., Line::MIDDLE);
            drive(2 * F_POWER, 1.);
            drive(B_POWER, 2.);
            break;
        }
        /*if value is outside red or blue*/
        case Color::NONE:
        {
            LCD.WriteLine("404");
            drive(B_POWER, 8.);
            c++;
            break;
        }
        default:
            break;
        }
    }
}

void crateToLever()
{
    // drive(B_POWER, 4.);
    // rotate(F_POWER, 30., Direction::LEFT);
    // drive(F_POWER, 4.);
    turn(B_POWER, 90., Direction::LEFT);
    turn(B_POWER, 140., Direction::RIGHT);
    rotate(F_POWER, 180., Direction::RIGHT);
    drive(B_POWER, 6.);
}

void drive(float percent, double dist)
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

    int sumPrev = 0;
    int sumCurr = encoderR.Counts() + encoderL.Counts();
    int i = 0;
    /*
    Takes the average of the encoder counts and compares to (desired) counts

    Division by 2 for average moved to other side of inequality and into
    calculation of counts to save fractions of a second in computation time each loop

    Breaks out early if the counts read by the wheels is the same from one loop
    iteration to the next
    */
    while (sumCurr < counts && i < 100000)
    {
        if (sumCurr == sumPrev)
        {
            i++;
        }
        else
        {
            i = 0;
        }
        sumPrev = sumCurr;
        sumCurr = encoderR.Counts() + encoderL.Counts();
    }

    stop();
}

float findCDS()
{
    CdSLimits lim;
    int counts = 2 * UNIT_COUNTS * 8.;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    Line state = Line::MIDDLE;

    int sumCurr = encoderR.Counts() + encoderL.Counts();
    int sumPrev = 0;
    int i = 0;

    float minCds = lim.maxOutput;

    /*
    Takes the average of the encoder counts and compares to (desired) counts

    Division by 2 for average moved to other side of inequality and into
    calculation of counts to save fractions of a second in computation time each loop

    Breaks out early if the counts read by the wheels is the same from one loop
    iteration to the next
    */
    while (sumCurr < counts && i < 500)
    {
        if (sumCurr == sumPrev)
        {
            i++;
        }
        else
        {
            i = 0;
        }
        sumPrev = sumCurr;
        sumCurr = encoderR.Counts() + encoderL.Counts();
        float read = cds.Value();
        if (read < minCds)
        {
            minCds = read;
        }
        state = followLine(F_POWER, state);
        LCD.WriteLine(read);
    }
    stop();
    return minCds;
}

void findLine()
{
    /*Whether the robot should turn right*/
    bool right = true;

    float t = 90;

    /*Positions robot to begin search*/
    rotate(F_POWER, t / 2, Direction::LEFT);

    /*Loops while line is not sensed by any sensors*/
    while ((optol.Value() < L_DIV && optom.Value() < M_DIV && optor.Value() < R_DIV))
    {
        if (right)
        {
            turn(F_POWER, t, Direction::RIGHT);
        }
        else
        {
            turn(F_POWER, t, Direction::LEFT);
        }
        right = !right;
    }
}

Line followLine(float percent, Line prevState)
{
    /*State of line under robot*/
    Line state = stateSense(prevState);
    /*Instruction for each state*/
    switch (state)
    {
    case Line::OFF_LEFT:
        turnOff(percent / 2, Direction::LEFT);
        break;
    case Line::OFF_RIGHT:
        turnOff(percent / 2, Direction::RIGHT);
        break;
    case Line::ON_LEFT:
        turnOn(percent / 2, Direction::LEFT);
        break;
    case Line::ON_RIGHT:
        turnOn(percent / 2, Direction::RIGHT);
        break;
    case Line::MIDDLE:
        straight(percent / 2);
        break;
    default:
        break;
    }
    return state;
}

Line followLine(float percent, float dist, Line prevState)
{
    /*
    Translates distance to counts

    Explanation of multiplication by 2 is in while loop
    */
    int counts = 2 * UNIT_COUNTS * dist;

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    Line state = prevState;

    int sumCurr = encoderR.Counts() + encoderL.Counts();
    int sumPrev = 0;
    int i = 0;

    /*
    Takes the average of the encoder counts and compares to (desired) counts

    Division by 2 for average moved to other side of inequality and into
    calculation of counts to save fractions of a second in computation time each loop

    Breaks out early if the counts read by the wheels is the same from one loop
    iteration to the next
    */
    while (sumCurr < counts && i < 200)
    {
        if (sumCurr == sumPrev)
        {
            i++;
        }
        else
        {
            i = 0;
        }
        sumPrev = sumCurr;
        sumCurr = encoderR.Counts() + encoderL.Counts();
        state = followLine(percent, state);
    }
    stop();
    return state;
}

Color getCDS(float val)
{
    CdSLimits lim;
    /*Compares val to Color limits, returning state of color*/
    if (val < lim.blueMax && val > lim.blueMin)
    {
        return Color::WATER;
    }
    else if (val < lim.redMax && val > lim.redMin)
    {
        return Color::FIRE;
    }
    else
    {
        return Color::NONE;
    }
}

void leverU()
{
    drive(B_POWER, 8.);
}
void leverD()
{
    drive(B_POWER * 1.25, 8.);
}

void levers()
{
    int count = 0;
    leverD();
    while (RCS.isLeverFlipped() == ON && count < 5)
    {
        count++;
        drive(F_POWER, 6.);
        rotate(F_POWER, 10., Direction::RIGHT);
        leverD();
    }
    rotate(F_POWER, 10. * count, Direction::LEFT);
    drive(F_POWER, 4.);
    count = 0;
    rotate(F_POWER, 90., Direction::RIGHT);
    drive(F_POWER, 1.5);
    rotate(F_POWER, 90., Direction::LEFT);
    // Sleep(5.);
    leverU();
    while (RCS.isLeverFlipped() == OFF && count < 5)
    {
        count++;
        drive(F_POWER, 6.);
        rotate(F_POWER, 10., Direction::RIGHT);
        leverU();
    }
    rotate(F_POWER, 10. * count, Direction::LEFT);
}

void leverToHumidifier()
{
    drive(F_POWER, 3.);
    rotate(F_POWER, 45., Direction::RIGHT);
    drive(F_POWER, 12.);
    rotate(F_POWER, 90., Direction::RIGHT);
    drive(B_POWER, 24.);
    drive(F_POWER, 12.);
}

void liftApples()
{
    toDegree(20., -180.);
}

float motorSpeed(float percent)
{
    return (11.5 / Battery.Voltage()) * percent;
}

void moveWindow(Direction dir)
{
    /*distance can be adjusted to more precisely open window*/
    float dist = 5.;
    int counts = 2 * (UNIT_COUNTS * dist);

    /*Actual power directed to motors to produce desired speed*/
    float speed = motorSpeed(5 * (F_POWER / 3) * dir);

    /*Corrects for force of pushing against window*/
    float leftCorrection = 1.1;

    /*
    Takes the average of the encoder counts and compares to (desired) counts

    Division by 2 for average moved to other side of inequality and into
    calculation of counts to save fractions of a second in computation time each loop
    */
    int c = 0;
    while (RCS.isWindowOpen() == ON && c < 3)
    {
        encoderR.ResetCounts();
        encoderL.ResetCounts();
        rightMotor.SetPercent(speed);
        leftMotor.SetPercent(speed * leftCorrection);
        int sumCurr = encoderR.Counts() + encoderL.Counts();
        int sumPrev = 0;
        int i = 0;
        while (sumCurr < counts && i < 100000)
        {
            if (sumCurr == sumPrev)
            {
                i++;
            }
            else
            {
                i = 0;
            }
            sumPrev = sumCurr;
            sumCurr = encoderR.Counts() + encoderL.Counts();
        }
        stop();
        turn(F_POWER, 5., Direction::RIGHT);
        c++;
    }
}

void openCloseWindow()
{
    /*Opens window*/
    moveWindow(Direction::FORWARDS);
    // /*Moves to other side of window*/
    // windowReposition();
    // /*Closes window*/
    // moveWindow(Direction::BACKWARDS);
}

void rotate(float percent, float deg, Direction dir)
{
    /*
    Calculates circumference the wheels travel (BOT_WIDTH treated as diameter)
    Multiples by fraction of the circle desired
    Translates to counts

    Explanation of multiplication by 2 is in while loop
    */
    float circumference = BOT_WIDTH * M_PI;
    float circleFraction = deg / DEGREES;
    int counts = 2 * (UNIT_COUNTS * (circumference * circleFraction));

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    /*Actual power directed to motors to produce desired speed*/
    float speed = motorSpeed(percent * dir);

    /*Starts each motor moving in different directions*/
    rightMotor.SetPercent(RIGHT_MOTOR_CORRECTION * speed * -1);
    leftMotor.SetPercent(LEFT_MOTOR_CORRECTION * speed);

    int sumCurr = encoderR.Counts() + encoderL.Counts();
    int sumPrev = 0;
    int i = 0;

    /*
    Takes the average of the encoder counts and compares to (desired) counts

    Division by 2 for average moved to other side of inequality and into
    calculation of counts to save fractions of a second in computation time each loop
    */
    while (encoderR.Counts() + encoderL.Counts() < counts && i < 100000)
    {
        if (sumCurr == sumPrev)
        {
            i++;
        }
        else
        {
            i = 0;
        }
        sumPrev = sumCurr;
        sumCurr = encoderR.Counts() + encoderL.Counts();
    }

    stop();
}

void setApples()
{
    toDegree(180.);
    drive(B_POWER, 3.);
    toDegree(180.);
}

void spinCompost(Direction dir)
{
    /*Keeps pulley pressed against compost*/
    rightMotor.SetPercent(motorSpeed(12));
    /*Rotates pulley against compost*/
    toDegree(50., dir * DEGREES * 3);
    /*Stops right wheel*/
    stop();
}

Line stateSense(Line prev)
{
    /*Value of left optosensor*/
    float left = optol.Value();
    /*Value of middle optosensor*/
    float middle = optom.Value();
    /*Value of right optosensor*/
    float right = optor.Value();

    if (left > L_DIV && right < R_DIV)
    {
        // LCD.WriteLine("on left");
        // LCD.WriteLine(optol.Value());
        return Line::ON_LEFT;
    }
    else if (right > R_DIV && left < L_DIV)
    {
        // LCD.WriteLine("on right");
        // LCD.WriteLine(optor.Value());
        return Line::ON_RIGHT;
    }
    else if (left < L_DIV && right < R_DIV && prev == ON_LEFT)
    {
        // LCD.WriteLine("off left");
        return Line::OFF_LEFT;
    }
    else if (right < R_DIV && left < L_DIV && prev == ON_RIGHT)
    {
        // LCD.WriteLine("off right");
        return Line::OFF_RIGHT;
    }
    else
    {
        // LCD.WriteLine("middle");
        return Line::MIDDLE;
    }
}

void stop()
{
    rightMotor.Stop();
    leftMotor.Stop();
    /*Ensures robot comes to a complete stop*/
    Sleep(0.2);
}

void straight(float percent)
{
    /*Moves each motor forward at same speed*/
    rightMotor.SetPercent(percent * RIGHT_MOTOR_CORRECTION);
    leftMotor.SetPercent(percent * LEFT_MOTOR_CORRECTION);
}

void toDegree(float degree)
{
    /*desired motor power*/
    float percent = 10;
    /*actual speed put into vex motor*/
    float speed = motorSpeed((degree / abs(degree)) * percent);
    /*time vex turns for*/
    float tt = degree / DEG_PER_SEC;
    /*how long PROTEUS has been on*/
    float time = TimeNow();
    LCD.WriteLine(tt);
    /*turns vex at actual speed*/
    vex.SetPercent(speed);
    /*vex keeps turning for calculated time*/
    Sleep(abs(tt));
    vex.Stop();
}

void toDegree(double percent, double degree)
{
    /*actual speed put into vex motor*/
    float speed = motorSpeed((degree / abs(degree)) * percent);
    /*time vex turns for*/
    float tt = degree / (DEG_PER_SEC * (speed / 10.));
    /*how long PROTEUS has been on*/
    float time = TimeNow();
    LCD.WriteLine(tt);
    /*turns vex at actual speed*/
    vex.SetPercent(speed);
    /*vex keeps turning for calculated time*/
    Sleep(abs(tt));
    vex.Stop();
}

void toDegree(float degree, int perSec)
{
    LCD.WriteLine(degree);
    float speed = motorSpeed((degree / abs(degree)) * 10);
    LCD.WriteLine(speed);
    float tt = degree / perSec;
    LCD.WriteLine(tt);
    float time = TimeNow();
    vex.SetPercent(speed);
    Sleep(abs(tt));
    LCD.WriteLine(TimeNow() - time);
    vex.Stop();
}

void turn(float percent, float deg, Direction dir)
{
    /*
    Calculates circumference the wheels travel (BOT_WIDTH treated as radius)
    Multiples by fraction of the circle desired
    Translates to counts
    */
    float circumference = 2 * BOT_WIDTH * M_PI;
    float circleFraction = deg / DEGREES;
    int counts = (UNIT_COUNTS * (circumference * circleFraction));

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    /*Actual power directed to motors to produce desired speed*/
    float speed = motorSpeed(percent);

    /*Activates motors to turn the robot*/
    switch (dir)
    {
    case Direction::LEFT:
        rightMotor.SetPercent(speed);
        break;
    case Direction::RIGHT:
        leftMotor.SetPercent(speed);
        break;
    default:
        break;
    }

    int sumCurr = encoderR.Counts() + encoderL.Counts();
    int sumPrev = 0;
    int i = 0;

    /*
    Sums encoder counts (one will be zero) and compares to counts

    Breaks out if optosensors encounters a line
    */
    while (encoderR.Counts() + encoderL.Counts() < counts && i < 100000 /*&& (optol.Value() < L_DIV || optom.Value() < M_DIV || optor.Value() < R_DIV)*/)
    {
        if (sumCurr == sumPrev)
        {
            i++;
        }
        else
        {
            i = 0;
        }
        sumPrev = sumCurr;
        sumCurr = encoderR.Counts() + encoderL.Counts();
    }

    stop();
}

void turnOff(float percent, Direction dir)
{
    /*Actual power directed to motors to produce desired speed*/
    float speed = motorSpeed(percent);
    /*Directs robot in direction to turn*/
    switch (dir)
    {
    case Direction::LEFT:
        leftMotor.Stop();
        rightMotor.SetPercent(speed);
        /*Loops while the middle optosensor doesn't "see" the line*/
        // while (optom.Value() < M_DIV)
        //     ;
        break;
    case Direction::RIGHT:
        rightMotor.Stop();
        leftMotor.SetPercent(speed);
        /*Loops while the middle optosensor doesn't "see" the line*/
        // while (optom.Value() < M_DIV)
        //     ;
        break;
    default:
        break;
    }
}

void turnOn(float percent, Direction dir)
{
    /*Actual power directed to motors to produce desired speed*/
    float speed = motorSpeed(percent);
    /*Directs robot in direction to turn*/
    switch (dir)
    {
    case Direction::LEFT:
        leftMotor.SetPercent(speed / 2);
        rightMotor.SetPercent(speed);
        // /*Loops while the middle optosensor doesn't "see" the line and the left sensor is on the line*/
        // while (optom.Value() < M_DIV && optol.Value() > L_DIV)
        //     ;
        break;
    case Direction::RIGHT:
        rightMotor.SetPercent(speed / 2);
        leftMotor.SetPercent(speed);
        /*Loops while the middle optosensor doesn't "see" the line and the right sensor is on the line*/
        // while (optom.Value() < M_DIV && optor.Value() > R_DIV)
        //     ;
        break;
    default:
        break;
    }
}

void windowReposition()
{
    drive(B_POWER, 1.5);
    rotate(B_POWER, 45., Direction::LEFT);
    drive(F_POWER, sqrt(2));
    rotate(F_POWER, 45., Direction::LEFT);
    drive(F_POWER, 1.5);
    rotate(F_POWER, 45., Direction::LEFT);
    drive(F_POWER, sqrt(2));
    turn(B_POWER, 45., Direction::LEFT);
    drive(B_POWER, 1.);
}
