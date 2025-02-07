#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <FEHUtility.h>

int main(void)
{
    AnalogInputPin cds(FEHIO::P3_0);
    float x;
    float y;
    while (!LCD.Touch(&x, &y)) {
        float val = cds.Value();
        LCD.Clear(BLACK);
        LCD.WriteLine(val);
        Sleep(0.5);
    }
	return 0;
}
