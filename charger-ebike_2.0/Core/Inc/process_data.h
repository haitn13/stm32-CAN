#ifndef __COMMON_H
#define __COMMON_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32g4xx_hal.h"
#include "main.h"



void query_data();
void send_data();
void received_command();
  

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_H */