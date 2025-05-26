#include <math.h>
#include "MAX30100_SpO2Calculator.h"

// SaO2 Look-up Table
// http://www.ti.com/lit/an/slaa274b/slaa274b.pdf
const uint8_t SpO2Calculator_spO2LUT[43] = {
    100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98,
    98, 97, 97, 97, 97, 97, 97, 96, 96, 96, 96, 96, 96, 95, 95,
    95, 95, 95, 95, 94, 94, 94, 94, 94, 93, 93, 93, 93, 93};

void SpO2Calculator_Reset(SpO2Calculator *calc)
{
    calc->samplesRecorded = 0;
    calc->redACValueSqSum = 0;
    calc->irACValueSqSum = 0;
    calc->beatsDetectedNum = 0;
    calc->spO2 = 0;
}

void SpO2Calculator_Update(SpO2Calculator *calc, float irACValue, float redACValue, bool beatDetected)
{
    calc->irACValueSqSum += irACValue * irACValue;
    calc->redACValueSqSum += redACValue * redACValue;
    calc->samplesRecorded++;

    if (beatDetected)
    {
        calc->beatsDetectedNum++;
        if (calc->beatsDetectedNum == CALCULATE_EVERY_N_BEATS)
        {
            float acSqRatio = 100.0f * logf(calc->redACValueSqSum / calc->samplesRecorded) / logf(calc->irACValueSqSum / calc->samplesRecorded);
            uint8_t index = 0;

            if (acSqRatio > 66)
            {
                index = (uint8_t)acSqRatio - 66;
            }
            else if (acSqRatio > 50)
            {
                index = (uint8_t)acSqRatio - 50;
            }
            // Reset sau mỗi lần tính toán
            SpO2Calculator_Reset(calc);

            if (index < 43)
            {
                calc->spO2 = SpO2Calculator_spO2LUT[index];
            }
            else
            {
                calc->spO2 = SpO2Calculator_spO2LUT[42];
            }
        }
    }
}

uint8_t SpO2Calculator_GetSpO2(SpO2Calculator *calc)
{
    return calc->spO2;
}