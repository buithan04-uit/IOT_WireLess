#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "stm32f1xx.h"  // hoặc core_cm3.h/core_cm4.h tùy dòng chip bạn dùng

void DWT_Delay_Init(void);
void delayMicroseconds(uint32_t us);

#endif /* INC_DELAY_H_ */
