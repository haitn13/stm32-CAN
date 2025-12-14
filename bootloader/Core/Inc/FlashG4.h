#ifndef __FLASHG4_H
#define __FLASHG4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"


uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint64_t *Data, uint16_t numberofwords);

void Flash_Read_Data (uint32_t StartPageAddress, uint64_t *RxBuf, uint16_t numberofwords);
	
uint32_t Set_Update_Firm_Completed(uint64_t ModeSel);




/********************  FLASH_Error_Codes   ***********************/
/*
HAL_FLASH_ERROR_NONE      0x00U  // No error
HAL_FLASH_ERROR_PROG      0x01U  // Programming error
HAL_FLASH_ERROR_WRP       0x02U  // Write protection error
HAL_FLASH_ERROR_OPTV      0x04U  // Option validity error
*/

#ifdef __cplusplus
}
#endif


#endif /* __FLASHG4_H */