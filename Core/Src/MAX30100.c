#include "MAX30100.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

// Khởi tạo cảm biến
bool MAX30100_Begin(MAX30100 *dev)
{
    if (MAX30100_GetPartId(dev) != EXPECTED_PART_ID)
        return false;

    // Thoát shutdown, chọn chế độ SPO2_HR
    MAX30100_WriteRegister(MAX30100_REG_MODE_CONFIGURATION, MAX30100_MODE_SPO2_HR);
    HAL_Delay(2);

    // Cấu hình SPO2: sampling rate, pulse width, highres
    MAX30100_ConfigureSPO2(dev, DEFAULT_SAMPLING_RATE, DEFAULT_PULSE_WIDTH, true);
    HAL_Delay(2);

    // Cấu hình dòng điện LED
    MAX30100_SetLedsCurrent(dev, DEFAULT_IR_LED_CURRENT, DEFAULT_RED_LED_CURRENT);
    HAL_Delay(2);

    // Reset FIFO
    MAX30100_ResetFifo(dev);
    HAL_Delay(2);

    return true;
}

// Chọn chế độ đo (HR, SPO2, ...)
void MAX30100_SetMode(MAX30100 *dev, Mode mode)
{
    MAX30100_WriteRegister(MAX30100_REG_MODE_CONFIGURATION, mode);
}

// Cấu hình sampling rate, pulse width, highres
void MAX30100_ConfigureSPO2(MAX30100 *dev, SamplingRate sr, LEDPulseWidth pw, bool highres)
{
    uint8_t value = 0;
    value |= (sr << 2); // Sampling rate
    value |= pw;        // Pulse width
    if (highres)
        value |= MAX30100_SPC_SPO2_HI_RES_EN;
    MAX30100_WriteRegister(MAX30100_REG_SPO2_CONFIGURATION, value);
}

// Cấu hình dòng điện LED
void MAX30100_SetLedsCurrent(MAX30100 *dev, LEDCurrent irLedCurrent, LEDCurrent redLedCurrent)
{
    MAX30100_WriteRegister(MAX30100_REG_LED_CONFIGURATION, (redLedCurrent << 4) | irLedCurrent);
}

// Đọc dữ liệu mới từ FIFO, cập nhật buffer
void MAX30100_Update(MAX30100 *dev)
{
    MAX30100_ReadFifoData(dev);
}

// Lấy giá trị raw IR/RED mới nhất
bool MAX30100_GetRawValues(MAX30100 *dev, uint16_t *ir, uint16_t *red)
{
    SensorReadout readout;
    if (!CircularBuffer_Pop(&dev->readoutsBuffer, &readout))
        return false;
    *ir = readout.ir;
    *red = readout.red;
    return true;
}

// Reset FIFO
void MAX30100_ResetFifo(MAX30100 *dev)
{
    MAX30100_WriteRegister(MAX30100_REG_FIFO_WRITE_POINTER, 0);
    MAX30100_WriteRegister(MAX30100_REG_FIFO_READ_POINTER, 0);
    MAX30100_WriteRegister(MAX30100_REG_FIFO_OVERFLOW_COUNTER, 0);
}

// Shutdown/Resume
void MAX30100_Shutdown(MAX30100 *dev)
{
    uint8_t modeConfig = MAX30100_ReadRegister(MAX30100_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30100_MC_SHDN;
    MAX30100_WriteRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

void MAX30100_Resume(MAX30100 *dev)
{
    uint8_t modeConfig = MAX30100_ReadRegister(MAX30100_REG_MODE_CONFIGURATION);
    modeConfig &= ~MAX30100_MC_SHDN;
    MAX30100_WriteRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

// Đọc Part ID
uint8_t MAX30100_GetPartId(MAX30100 *dev)
{
    return MAX30100_ReadRegister(MAX30100_REG_PART_ID);
}

// --- I2C giao tiếp sử dụng HAL ---
uint8_t MAX30100_ReadRegister(uint8_t address)
{
    uint8_t value = 0;
    HAL_I2C_Mem_Read(&hi2c1, MAX30100_I2C_ADDRESS << 1, address, 1, &value, 1, 100);
    return value;
}

void MAX30100_WriteRegister(uint8_t address, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, MAX30100_I2C_ADDRESS << 1, address, 1, &data, 1, 100);
}

void MAX30100_BurstRead(uint8_t baseAddress, uint8_t *buffer, uint8_t length)
{
    HAL_I2C_Mem_Read(&hi2c1, MAX30100_I2C_ADDRESS << 1, baseAddress, 1, buffer, length, 100);
}

// Đọc dữ liệu từ FIFO, đẩy vào buffer
void MAX30100_ReadFifoData(MAX30100 *dev)
{
    uint8_t buffer[4];
    MAX30100_BurstRead(MAX30100_REG_FIFO_DATA, buffer, 4);

    SensorReadout readout;
    readout.ir = (uint16_t)((buffer[0] << 8) | buffer[1]);
    readout.red = (uint16_t)((buffer[2] << 8) | buffer[3]);
    CircularBuffer_Push(&dev->readoutsBuffer, readout);
}
