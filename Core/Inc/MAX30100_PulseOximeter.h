#ifndef MAX30100_PULSEOXIMETER_H
#define MAX30100_PULSEOXIMETER_H

#define SAMPLING_FREQUENCY 100
#define CURRENT_ADJUSTMENT_PERIOD_MS 500
#define DEFAULT_IR_LED_CURRENT MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT_START MAX30100_LED_CURR_27_1MA
#define DC_REMOVER_ALPHA 0.95f

#include <stdint.h>
#include "MAX30100.h"
#include "MAX30100_BeatDetector.h"
#include "MAX30100_Filters.h"
#include "MAX30100_SpO2Calculator.h"

typedef enum
{
    PULSEOXIMETER_STATE_INIT,
    PULSEOXIMETER_STATE_IDLE,
    PULSEOXIMETER_STATE_DETECTING
} PulseOximeterState;

typedef enum
{
    PULSEOXIMETER_DEBUGGINGMODE_NONE,
    PULSEOXIMETER_DEBUGGINGMODE_RAW_VALUES,
    PULSEOXIMETER_DEBUGGINGMODE_AC_VALUES,
    PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT
} PulseOximeterDebuggingMode;

typedef struct
{
    PulseOximeterState state;
    PulseOximeterDebuggingMode debuggingMode;
    uint32_t tsFirstBeatDetected;
    uint32_t tsLastBeatDetected;
    uint32_t tsLastBiasCheck;
    uint32_t tsLastCurrentAdjustment;
    BeatDetector beatDetector;
    DCRemover irDCRemover;
    DCRemover redDCRemover;
    FilterBuLp1 lpf;
    uint8_t redLedCurrentIndex;
    LEDCurrent irLedCurrent;
    SpO2Calculator spO2calculator;
    MAX30100 hrm;
    void (*onBeatDetected)(void);
} PulseOximeter;

// Khởi tạo PulseOximeter
void PulseOximeter_Init(PulseOximeter *po);

// Bắt đầu cảm biến, trả về true nếu thành công
bool PulseOximeter_Begin(PulseOximeter *po, PulseOximeterDebuggingMode debuggingMode);

// Cập nhật dữ liệu, gọi định kỳ (ví dụ mỗi 10ms)
void PulseOximeter_Update(PulseOximeter *po);

// Lấy nhịp tim hiện tại (bpm)
float PulseOximeter_GetHeartRate(PulseOximeter *po);

// Lấy giá trị SpO2 hiện tại (%)
uint8_t PulseOximeter_GetSpO2(PulseOximeter *po);

// Lấy giá trị bias dòng LED đỏ
uint8_t PulseOximeter_GetRedLedCurrentBias(PulseOximeter *po);

// Đăng ký callback khi phát hiện nhịp tim
void PulseOximeter_SetOnBeatDetectedCallback(PulseOximeter *po, void (*cb)(void));

// Cấu hình dòng IR LED
void PulseOximeter_SetIRLedCurrent(PulseOximeter *po, LEDCurrent irLedCurrent);

// Shutdown cảm biến
void PulseOximeter_Shutdown(PulseOximeter *po);

// Resume cảm biến
void PulseOximeter_Resume(PulseOximeter *po);

#endif