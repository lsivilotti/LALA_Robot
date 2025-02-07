#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <FEHUtility.h>
#include <FEHServo.h>

#define SERVO_MAX 2500
#define SERVO_MIN 500
#define FULL_LIGHT 0
#define COVERED 3.3

AnalogInputPin cds(FEHIO::P3_0);
FEHServo servo(FEHServo::Servo7);


int main(void)
{
    servo.SetMin(SERVO_MIN);
    servo.SetMax(SERVO_MAX);
    while (true)
    {
        float deg = 180*(cds.Value() / 3.3);
        servo.SetDegree(deg);
        Sleep(1.0);
    }
    return 0;
    
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
