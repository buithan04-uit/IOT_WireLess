#ifndef PMS_H
#define PMS_H

#include "stm32f1xx_hal.h"  // Điều chỉnh theo dòng STM32 của bạn
#include <stdint.h>
#include <stdbool.h>

#define PMS_PAYLOAD_SIZE 12
#define PMS_FRAME_MAX_LEN 32

#define PMS_BAUD_RATE 9600
#define PMS_SINGLE_RESPONSE_TIME 1000
#define PMS_TOTAL_RESPONSE_TIME 10000
#define PMS_STEADY_RESPONSE_TIME 30000

typedef struct {
    uint16_t PM_SP_UG_1_0;
    uint16_t PM_SP_UG_2_5;
    uint16_t PM_SP_UG_10_0;
    uint16_t PM_AE_UG_1_0;
    uint16_t PM_AE_UG_2_5;
    uint16_t PM_AE_UG_10_0;
} PMS_DATA;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t buffer[PMS_FRAME_MAX_LEN];
    uint8_t index;
    uint8_t payload[PMS_PAYLOAD_SIZE];
    uint16_t frameLen;
    uint16_t checksum;
    uint16_t calculatedChecksum;

    PMS_DATA *data;
    enum { STATUS_WAITING, STATUS_OK } status;
    enum { MODE_ACTIVE, MODE_PASSIVE } mode;
} PMS_HandleTypeDef;

void PMS_Init(PMS_HandleTypeDef *pms, UART_HandleTypeDef *huart);
void PMS_Sleep(PMS_HandleTypeDef *pms);
void PMS_WakeUp(PMS_HandleTypeDef *pms);
void PMS_ActiveMode(PMS_HandleTypeDef *pms);
void PMS_PassiveMode(PMS_HandleTypeDef *pms);
void PMS_RequestRead(PMS_HandleTypeDef *pms);
bool PMS_Read(PMS_HandleTypeDef *pms, PMS_DATA *data);
bool PMS_ReadUntil(PMS_HandleTypeDef *pms, PMS_DATA *data, uint32_t timeout_ms);

#endif
