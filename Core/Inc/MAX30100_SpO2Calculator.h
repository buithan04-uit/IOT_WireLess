#ifndef MAX30100_SPO2CALCULATOR_H
#define MAX30100_SPO2CALCULATOR_H

#include <stdint.h>
#include <stdbool.h>

#define CALCULATE_EVERY_N_BEATS 3

typedef struct
{
    float irACValueSqSum;
    float redACValueSqSum;
    uint8_t beatsDetectedNum;
    uint32_t samplesRecorded;
    uint8_t spO2;
} SpO2Calculator;

// Bảng tra cứu SpO2 (lookup table)
extern const uint8_t SpO2Calculator_spO2LUT[43];

// Khởi tạo lại bộ tính SpO2
void SpO2Calculator_Reset(SpO2Calculator *calc);

// Cập nhật giá trị mới, gọi mỗi lần có mẫu mới
void SpO2Calculator_Update(SpO2Calculator *calc, float irACValue, float redACValue, bool beatDetected);

// Lấy giá trị SpO2 hiện tại
uint8_t SpO2Calculator_GetSpO2(SpO2Calculator *calc);

#endif