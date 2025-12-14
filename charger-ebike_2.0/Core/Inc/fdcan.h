#ifndef __FDCAN_H
#define __FDCAN_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32g4xx_hal.h"
#include "main.h"

#define MAX_MESSAGE_ID 12
#define FDCAN_TX_DELAY_MS 5

#define BMS_ALIVE 0xAA

extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;

typedef uint8_t (*CanHandlerFunction_t)(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader);
typedef struct {
    uint32_t CanID;
    CanHandlerFunction_t Handler;
} CanIDHandlerMap_t;

void FDCAN_Config(void);
int CAN_BMS0_read_data(void);
int CAN_BMS1_read_data(void);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H */