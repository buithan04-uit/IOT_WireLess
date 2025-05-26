/*
 * PMS.c
 *
 *  Created on: May 24, 2025
 *      Author: buith
 */
#include "PMS.h"

static uint16_t makeWord(uint8_t high, uint8_t low) {
    return (high << 8) | low;
}

void PMS_Init(PMS_HandleTypeDef *pms, UART_HandleTypeDef *huart) {
    pms->huart = huart;
    pms->index = 0;
    pms->status = STATUS_WAITING;
    pms->mode = MODE_ACTIVE;
}

void PMS_Sleep(PMS_HandleTypeDef *pms) {
    uint8_t cmd[] = { 0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73 };
    HAL_UART_Transmit(pms->huart, cmd, sizeof(cmd), 100);
}

void PMS_WakeUp(PMS_HandleTypeDef *pms) {
    uint8_t cmd[] = { 0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74 };
    HAL_UART_Transmit(pms->huart, cmd, sizeof(cmd), 100);
}

void PMS_ActiveMode(PMS_HandleTypeDef *pms) {
    uint8_t cmd[] = { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 };
    HAL_UART_Transmit(pms->huart, cmd, sizeof(cmd), 100);
    pms->mode = MODE_ACTIVE;
}

void PMS_PassiveMode(PMS_HandleTypeDef *pms) {
    uint8_t cmd[] = { 0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70 };
    HAL_UART_Transmit(pms->huart, cmd, sizeof(cmd), 100);
    pms->mode = MODE_PASSIVE;
}

void PMS_RequestRead(PMS_HandleTypeDef *pms) {
    if (pms->mode == MODE_PASSIVE) {
        uint8_t cmd[] = { 0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71 };
        HAL_UART_Transmit(pms->huart, cmd, sizeof(cmd), 100);
    }
}

static void PMS_Loop(PMS_HandleTypeDef *pms) {
    pms->status = STATUS_WAITING;

    uint8_t ch;
    while (HAL_UART_Receive(pms->huart, &ch, 1, 10) == HAL_OK) {
        switch (pms->index) {
            case 0:
                if (ch != 0x42) return;
                pms->calculatedChecksum = ch;
                break;

            case 1:
                if (ch != 0x4D) {
                    pms->index = 0;
                    return;
                }
                pms->calculatedChecksum += ch;
                break;

            case 2:
                pms->calculatedChecksum += ch;
                pms->frameLen = ch << 8;
                break;

            case 3:
                pms->frameLen |= ch;
                pms->calculatedChecksum += ch;
                if (pms->frameLen != 2 * 9 + 2 && pms->frameLen != 2 * 13 + 2) {
                    pms->index = 0;
                    return;
                }
                break;

            default:
                if (pms->index == pms->frameLen + 2) {
                    pms->checksum = ch << 8;
                } else if (pms->index == pms->frameLen + 3) {
                    pms->checksum |= ch;

                    if (pms->calculatedChecksum == pms->checksum) {
                        pms->status = STATUS_OK;

                        pms->data->PM_SP_UG_1_0  = makeWord(pms->payload[0], pms->payload[1]);
                        pms->data->PM_SP_UG_2_5  = makeWord(pms->payload[2], pms->payload[3]);
                        pms->data->PM_SP_UG_10_0 = makeWord(pms->payload[4], pms->payload[5]);

                        pms->data->PM_AE_UG_1_0  = makeWord(pms->payload[6], pms->payload[7]);
                        pms->data->PM_AE_UG_2_5  = makeWord(pms->payload[8], pms->payload[9]);
                        pms->data->PM_AE_UG_10_0 = makeWord(pms->payload[10], pms->payload[11]);
                    }

                    pms->index = 0;
                    return;
                } else {
                    pms->calculatedChecksum += ch;
                    uint8_t payloadIndex = pms->index - 4;
                    if (payloadIndex < PMS_PAYLOAD_SIZE) {
                        pms->payload[payloadIndex] = ch;
                    }
                }
                break;
        }

        pms->index++;
    }
}

bool PMS_Read(PMS_HandleTypeDef *pms, PMS_DATA *data) {
    pms->data = data;
    PMS_Loop(pms);
    return pms->status == STATUS_OK;
}

bool PMS_ReadUntil(PMS_HandleTypeDef *pms, PMS_DATA *data, uint32_t timeout_ms) {
    uint32_t startTick = HAL_GetTick();
    pms->data = data;
    do {
        PMS_Loop(pms);
        if (pms->status == STATUS_OK) return true;
    } while (HAL_GetTick() - startTick < timeout_ms);

    return false;
}


