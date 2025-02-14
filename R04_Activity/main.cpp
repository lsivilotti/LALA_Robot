/**
 * @file main.cpp
 * @brief Code for Activity R04
 * @date 02/10/2025
 * @author Luca Sivilotti
 */

#include <FEHLCD.h>
#include <FEHIO.h>
// #include <FEHSD.h>
#include <FEHUtility.h>
#include <FEHServo.h>
#include <FEHMotor.h>

/*Servo constants*/
#define SERVO_MAX 2500
#define SERVO_MIN 500

/*CdS Cell constants*/
#define FULL_LIGHT 0
#define COVERED 3.3

/*Switch constants*/
#define ON 0
#define OFF 1

/*Motor constants*/
#define VOLTAGE 9.0
#define F_POWER 25.
#define B_POWER -25.

/*Cadmium Sulfide Cell*/
AnalogInputPin cds(FEHIO::P3_0);
/*Front right switch*/
DigitalInputPin fr(FEHIO::P1_0);
/*Front left switch*/
DigitalInputPin fl(FEHIO::P1_1);
/*Back right switch*/
DigitalInputPin br(FEHIO::P1_2);
/*Back left switch*/
DigitalInputPin bl(FEHIO::P1_7);
/*Motor powering right wheel*/
FEHMotor rightMotor(FEHMotor::Motor0, VOLTAGE);
/*Motor powering left wheel*/
FEHMotor leftMotor(FEHMotor::Motor1, VOLTAGE);

/**
 * @brief Moves both motors same direction with the same power.
 *
 * @param percent power that motors move at
 * @warning -100 <= percent <= 100
 */
void moveInLine(double percent)
{
    rightMotor.SetPercent(percent);
    leftMotor.SetPercent(percent * -1);
}

/**
 * @brief Stops both motors.
 */
void stop()
{
    rightMotor.Stop();
    leftMotor.Stop();
    Sleep(0.5);
}

/**
 * @brief Moves motors forward or backwards.
 *
 * @param right DigitalInputPin on the right side of the bot
 * @param left DigitalInputPin on the left side of the bot
 * @param dir direction bot moves in
 * @warning -1 <= dir <= 1, dir is not 0
 */
void linearMove(DigitalInputPin right, DigitalInputPin left, int dir)
{
    int rVal = right.Value();
    int lVal = left.Value();
    while ((rVal == OFF) || (lVal == OFF))
    {
        if (rVal == ON)
        {
            leftMotor.Stop();
        }
        else if (lVal == ON)
        {
            rightMotor.Stop();
        }
        else
        {
            moveInLine(F_POWER * dir);
        }
        rVal = right.Value();
        lVal = left.Value();
    }
    stop();
}

/**
 * @brief Turns bot.
 *
 * @param motor the motor used to turn the robot while the other stays still
 * @param dir direction bot moves in
 * @warning -1 <= dir <= 1, dir is not 0
 */
void turn(FEHMotor motor, int dir)
{
    double t = TimeNow();
    stop();
    t = TimeNow();
    while (TimeNow() - t < 1.5)
    {
        motor.SetPercent(B_POWER * dir);
    }
    stop();
    linearMove(br, bl, -1);
}

/**
 * @brief Main method.
 */
int main(void)
{
    float x, y;
    // while (true)
    {
        while (!LCD.Touch(&x, &y))
            ;
        while (LCD.Touch(&x, &y))
            ;
        linearMove(fr, fl, 1);
        turn(leftMotor, -1);
        linearMove(fr, fl, 1);
        turn(rightMotor, 1);
        linearMove(fr, fl, 1);
    }
}
