/**
 * @file main.cpp
 * @brief Code for Checkpoint 5
 * @date 04/04/2025
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
#define F_POWER 30.
/*Reverse power*/
#define B_POWER -30.
/*Correction for right motor (make very very small changes)*/
#define RIGHT_MOTOR_CORRECTION 1
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
#define R_DIV 2.81

/*CdS constants –––––––––––––––––––––––––––––––––––––––––––––––––––*/
/**
 * @todo change limits with more testing to get more precise bands
 */

/*Lower and upper limits read from color*/
struct CdSLimits
{
    float maxOutput = 3.3;
    float lightOffMin = 1.9;
    float blueMax = 1.9;
    float blueMin = 1.5;
    float redMax = 1.5;
    float redMin = 1.2;
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
void activateHumidifier(Color);
void drive(float, double);
float findCDS();
void findLine();
Line followLine(Line);
Line followLine(Line, float);
Color getCDS(float);
void leverDown();
void leverUp();
void liftApples();
float motorSpeed(float);
void moveWindow(Direction);
void openCloseWindow();
void rotate(float, float, Direction);
void setApples();
void spinCompost(Direction);
Line stateSense(Line);
void stop();
void straight(float);
void toDegree(float);
void toDegree(double, double);
void toDegree(float, int);
void turn(float, float, Direction);
void turnOff(float, Direction);
void turnOn(float, Direction);
void windowReposition();

/**
 * @brief Main method.
 */
int main(void)
{
    /**
     * Start conditions:
     * - LALA facing button
     * - rotating arm not on
     * - light beneath is off
     * - connected to RCS
     */
    RCS.InitializeTouchMenu(IDENTIFIER);
    CdSLimits lims;
    float buttonPressSpeed = (2 * F_POWER);
    LCD.Clear();
    while (cds.Value() > lims.lightOffMin && TimeNow() < 30)
    {
        LCD.Write(cds.Value());
        Sleep(0.25);
    }
    /*Presses button*/
    drive(buttonPressSpeed, 1);
    /*Navigates to compost*/
    drive(B_POWER, 6);
    rotate(F_POWER, 135, RIGHT);
    drive(F_POWER, 15);
    turn(B_POWER, 45, LEFT);
    turn(B_POWER, 45, RIGHT);
    /*Spins compost*/
    /*to rotate compost 300˚: vex needs to rotate 2.1877... times
    AND (facing from the back) vex needs to rotate ccw*/
    spinCompost(BACKWARDS);
    Sleep(1.5);
    spinCompost(FORWARDS);
    /*Navigates back to button*/
    turn(F_POWER, 45, RIGHT);
    turn(F_POWER, 45, LEFT);
    drive(B_POWER, 30);
    drive(F_POWER, 3);
    rotate(F_POWER, 135, LEFT);
    /*Presses button*/
    drive(buttonPressSpeed, 3);
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
        /*looks for then gets the color of LED*/
        col = getCDS(findCDS());
        /*actions based on color read by CdS*/
        switch (col)
        {
            /*if red*/
        case FIRE:
            rotate(F_POWER, 90, RIGHT);
            drive(F_POWER, 1);
            rotate(F_POWER, 90, LEFT);
            followLine(LINE_MIDDLE, 6);
            break;
            /*if blue*/
        case WATER:
            rotate(F_POWER, 90, LEFT);
            drive(F_POWER, 1);
            rotate(F_POWER, 90, RIGHT);
            followLine(LINE_MIDDLE, 6);
            break;
            /*if value is outside red or blue*/
        case NONE:
            LCD.WriteLine("404");
            drive(B_POWER, 6);
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Moves robot a specified amount.
 *
 * @param percent motor speed (if < 0 robot will drive backwards)
 * @param dist distance for the bot to travel (inches)
 */
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
 * @brief Positions robot over light source.
 *
 * @return value read by CdS
 */
float findCDS()
{
    followLine(LINE_MIDDLE, 6);
    return cds.Value();
}

/**
 * @brief Moves the robot in a pattern to find the line.
 */
void findLine()
{
    /*Whether the robot should turn right*/
    bool right = true;

    float t = 90;

    /*Positions robot to begin search*/
    rotate(F_POWER, t / 2, LEFT);

    /*Loops while line is not sensed by any sensors*/
    while ((optol.Value() < L_DIV && optom.Value() < M_DIV && optor.Value() < R_DIV))
    {
        if (right)
        {
            turn(F_POWER, t, RIGHT);
        }
        else
        {
            turn(F_POWER, t, LEFT);
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
        turnOff(F_POWER / 2, LEFT);
        break;
    case LINE_OFF_RIGHT:
        turnOff(F_POWER / 2, RIGHT);
        break;
    case LINE_ON_LEFT:
        turnOn(F_POWER / 2, LEFT);
        break;
    case LINE_ON_RIGHT:
        turnOn(F_POWER / 2, RIGHT);
        break;
    case LINE_MIDDLE:
        straight(F_POWER / 2);
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
    while (encoderL.Counts() + encoderR.Counts() < counts)
    {
        state = followLine(state);
    }
    stop();
    return state;
}

/**
 * @brief Determines the color of the light sensed by CdS.
 *
 * @param val value read by CdS
 * @return a value in Color {NONE, WATER, FIRE}
 */
Color getCDS(float val)
{
    CdSLimits lim;
    /*Compares val to Color limits, returning state of color*/
    if (val < lim.blueMax && val > lim.blueMin)
    {
        return WATER;
    }
    else if (val < lim.redMax && val > lim.redMin)
    {
        return FIRE;
    }
    else
    {
        return NONE;
    }
}

/**
 * @brief Flips fertilizer lever down using rotating arm
 */
void leverDown()
{
    toDegree(-270);
}

/**
 * @brief Flips fertilizer lever up using rotating arm
 */
void leverUp()
{
    toDegree(370);
}

/**
 * @brief Lifts apples from stump
 */
void liftApples()
{
    // toDegree(30);
    // followLine(Line::LINE_MIDDLE, 6.);
    // rotate(F_POWER, 90, LEFT);
    drive(F_POWER, 6);
    // rotate(F_POWER, 90, RIGHT);
    // drive(F_POWER, 2);
    toDegree(180);
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
 * @brief Opens/closes window.
 *
 * @param dir direction of opening/closing {FORWARDS = open, BACKWARDS = close}
 */
void moveWindow(Direction dir)
{
    /*distance can be adjusted to more precisely open window*/
    float dist = 5.;
    int counts = 2 * (UNIT_COUNTS * dist);

    encoderR.ResetCounts();
    encoderL.ResetCounts();

    /*Actual power directed to motors to produce desired speed*/
    float speed = motorSpeed(5 * (F_POWER / 3) * dir);

    float leftCorrection = 1.1;

    rightMotor.SetPercent(speed);
    leftMotor.SetPercent(speed * leftCorrection);

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
 * @brief Whole process of opening and closing window.
 */
void openCloseWindow()
{
    /*Opens window*/
    moveWindow(FORWARDS);
    /*Moves to other side of window*/
    windowReposition();
    /*Closes window*/
    moveWindow(BACKWARDS);
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
 * @brief Places apples in crate (can be changed to table later)
 */
void setApples()
{
    toDegree(180.);
    drive(B_POWER, 3.);
}

/**
 * @brief Spins compost bin using vex motor and pulley system.
 *
 * @param dir direction the compost is spun (FORWARDS = ccw, BACKWARDS = cw)
 */
void spinCompost(Direction dir)
{
    /*Keeps pulley pressed against compost*/
    rightMotor.SetPercent(motorSpeed(10));
    /*Rotates pulley against compost*/
    toDegree(50., dir * DEGREES / 2);
    /*Stops right wheel*/
    stop();
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

    if (left > L_DIV && right < R_DIV)
    {
        LCD.WriteLine("on left");
        LCD.WriteLine(optol.Value());
        return LINE_ON_LEFT;
    }
    else if (right > R_DIV && left < L_DIV)
    {
        LCD.WriteLine("on right");
        LCD.WriteLine(optor.Value());
        return LINE_ON_RIGHT;
    }
    else if (left < L_DIV && right < R_DIV && prev == LINE_ON_LEFT)
    {
        LCD.WriteLine("off left");
        return LINE_OFF_LEFT;
    }
    else if (right < R_DIV && left < L_DIV && prev == LINE_ON_RIGHT)
    {
        LCD.WriteLine("off right");
        return LINE_OFF_RIGHT;
    }
    else
    {
        LCD.WriteLine("middle");
        return LINE_MIDDLE;
    }
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
 * @brief Turns vex motor the given degrees.
 *
 * @param degree degrees for vex motor to turn lever
 * @attention Whatever degree the motor is at when the method is call is the new "zero" degree.
 * @attention Positive degree is cw, negative is ccw (looking from the back)
 */
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
    /*turns vex at actual speed*/
    vex.SetPercent(speed);
    /*vex keeps turning for calculated time*/
    Sleep(abs(tt));
    vex.Stop();
}

/**
 * @brief Turns vex motor the given degrees at given power.
 *
 * @param percent input power for vex motor to turn lever
 * @param degree degrees for vex motor to turn lever
 * @attention Whatever degree the motor is at when the method is call is the new "zero" degree.
 * @attention Positive degree is cw, negative is ccw (looking from the back)
 */
void toDegree(double percent, double degree)
{
    /*actual speed put into vex motor*/
    float speed = motorSpeed((degree / abs(degree)) * percent);
    /*time vex turns for*/
    float tt = degree / DEG_PER_SEC;
    /*how long PROTEUS has been on*/
    float time = TimeNow();
    /*turns vex at actual speed*/
    vex.SetPercent(speed);
    /*vex keeps turning for calculated time*/
    Sleep(abs(tt));
    vex.Stop();
}

/**
 * Used for testing
 */
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
    while (encoderR.Counts() + encoderL.Counts() < counts /*&& (optol.Value() < L_DIV || optom.Value() < M_DIV || optor.Value() < R_DIV)*/)
        ;

    stop();
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
        // while (optom.Value() < M_DIV)
        //     ;
        break;
    case RIGHT:
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
        // /*Loops while the middle optosensor doesn't "see" the line and the left sensor is on the line*/
        // while (optom.Value() < M_DIV && optol.Value() > L_DIV)
        //     ;
        break;
    case RIGHT:
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

/**
 * @brief Repositions robot on other side of window handle.
 */
void windowReposition()
{
    drive(B_POWER, 1.5);
    rotate(B_POWER, 45., LEFT);
    drive(F_POWER, sqrt(2));
    rotate(F_POWER, 45., LEFT);
    drive(F_POWER, 1.5);
    rotate(F_POWER, 45., LEFT);
    drive(F_POWER, sqrt(2));
    turn(B_POWER, 45., LEFT);
    drive(B_POWER, 1.);
}
