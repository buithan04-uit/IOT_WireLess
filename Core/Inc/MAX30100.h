#ifndef MAX30100_H
#define MAX30100_H

#include <stdint.h>
#include <stdbool.h>
#include "CircularBuffer.h"
#include "MAX30100_Registers.h"

// Giá trị mặc định
#define EXPECTED_PART_ID 0x11
#define DEFAULT_MODE MAX30100_MODE_SPO2_HR
#define DEFAULT_SAMPLING_RATE MAX30100_SAMPRATE_100HZ
#define DEFAULT_PULSE_WIDTH MAX30100_SPC_PW_1600US_16BITS
#define DEFAULT_RED_LED_CURRENT MAX30100_LED_CURR_50MA
#define DEFAULT_IR_LED_CURRENT MAX30100_LED_CURR_50MA

typedef struct
{
    CircularBuffer readoutsBuffer;
    // Thêm biến cấu hình nếu cần
} MAX30100;

// Khởi tạo cảm biến
bool MAX30100_Begin(MAX30100 *dev);

// Cấu hình chế độ đo
void MAX30100_SetMode(MAX30100 *dev, Mode mode);

// Cấu hình SPO2 (sampling rate, pulse width, highres)
void MAX30100_ConfigureSPO2(MAX30100 *dev, SamplingRate sr, LEDPulseWidth pw, bool highres);

// Cấu hình dòng điện LED
void MAX30100_SetLedsCurrent(MAX30100 *dev, LEDCurrent irLedCurrent, LEDCurrent redLedCurrent);

// Đọc dữ liệu mới từ FIFO, cập nhật buffer
void MAX30100_Update(MAX30100 *dev);

// Lấy giá trị raw IR/RED mới nhất
bool MAX30100_GetRawValues(MAX30100 *dev, uint16_t *ir, uint16_t *red);

// Reset FIFO
void MAX30100_ResetFifo(MAX30100 *dev);

// Shutdown/Resume
void MAX30100_Shutdown(MAX30100 *dev);
void MAX30100_Resume(MAX30100 *dev);

// Đọc Part ID
uint8_t MAX30100_GetPartId(MAX30100 *dev);

// --- Các hàm giao tiếp I2C nội bộ ---
uint8_t MAX30100_ReadRegister(uint8_t address);
void MAX30100_WriteRegister(uint8_t address, uint8_t data);
void MAX30100_BurstRead(uint8_t baseAddress, uint8_t *buffer, uint8_t length);
void MAX30100_ReadFifoData(MAX30100 *dev);

#endif
