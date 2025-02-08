/**
 * @file main.cpp
 * @brief Code for Activity R04
 * @date 02/07/2025
 * @author Luca Sivilotti
 */

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
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
/*Experimentation servo*/
// FEHServo servo(FEHServo::Servo7);
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
    leftMotor.SetPercent(percent);
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
            rightMotor.Stop();
        }
        else if (lVal == ON)
        {
            leftMotor.Stop();
        }
        else
        {
            moveInLine(F_POWER * dir);
        }
        rVal = fr.Value();
        lVal = fl.Value();
    }
    stop();
}

/**
 * @brief Turns bot.
 *
 * @param motor the motor used to turn the robot while the other stays still
 */
void turn(FEHMotor motor)
{
    double t = TimeNow();
    while (TimeNow() - t < 0.5)
    {
        moveInLine(B_POWER);
    }
    stop();
    t = TimeNow();
    while (TimeNow() - t < 1)
    {
        motor.SetPercent(B_POWER);
    }
    stop();
    linearMove(br, bl, -1);
}

/**
 * @brief Main method.
 */
int main(void)
{
    linearMove(fr, fl, 1);
    turn(rightMotor);
    linearMove(fr, fl, 1);
    turn(leftMotor);
    linearMove(fr, fl, 1);

    // servo.SetMin(SERVO_MIN);
    // servo.SetMax(SERVO_MAX);
    // while (true)
    // {
    //     float deg = 180*(cds.Value() / 3.3);
    //     servo.SetDegree(deg);
    //     Sleep(1.0);
    // }
    // return 0;

    // servo.TouchCalibrate();

    // float x;
    // float y;
    // while (!LCD.Touch(&x, &y)) {
    //     float val = cds.Value();
    //     LCD.Clear(BLACK);
    //     LCD.WriteLine(val);
    //     Sleep(0.5);
    // }
    // return 0;
}
