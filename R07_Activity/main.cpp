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
#define L_POWER 25.
#define R_POWER -25.

/*Optosensor constants*/
#define L_DIV 1.65
#define M_DIV 1.65
#define R_DIV 1.65

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

int followLine(int);
int stateSense(int);

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
    FEHFile *ptr = SD.FOpen("DATA.txt", "w");
    // while (true)
    {
        double time = TimeNow();
        while (TimeNow() - time < 300)
        {
            LCD.Clear();
            LCD.Write(optol.Value());
            SD.FPrintf(ptr, "Left: %f,\t", optol.Value());
            LCD.Write(optom.Value());
            SD.FPrintf(ptr, "Middle: %f,\t", optom.Value());
            LCD.Write(optor.Value());
            SD.FPrintf(ptr, "Right: %f,\t", optor.Value());
            Sleep(1.0);
        }

        // while (!LCD.Touch(&x, &y))
        //     ;
        // while (LCD.Touch(&x, &y))
        //     ;
        // int prevState = LINE_MIDDLE;
        // while (fr.Value() == OFF && fl.Value() == OFF)
        // {
        //     prevState = followLine(prevState);
        // }
    }
    SD.FClose(ptr);
}

int followLine(int prevState)
{
    int state = stateSense(prevState);
    switch (state)
    {
    case LINE_OFF_LEFT:
        // turnOff()
        break;

    default:
        break;
    }
    return 0;
}

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