#include "FlashG4.h"
#include "string.h"
#include "stdio.h"

uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint64_t *Data, uint16_t numberofwords)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	volatile int sofar=0;

	  /* Unlock the Flash to enable the flash control register access *************/
	   HAL_FLASH_Unlock();

		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_SR_PROGERR | FLASH_FLAG_PGSERR);
	   /* Erase the user Flash area*/

		volatile uint32_t StartPageIndex = ((StartPageAddress - 0x08000000) / FLASH_PAGE_SIZE) - 128;

	   /* Fill EraseInit structure*/
	   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	   EraseInitStruct.Page 			 = StartPageIndex;
		 EraseInitStruct.Banks			 = FLASH_BANK_2;
		 EraseInitStruct.NbPages     = 1;
	   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	   {
	     /*Error occurred while page erase.*/
		  return HAL_FLASH_GetError ();
	   }

	   /* Program the user Flash area word by word*/

	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, StartPageAddress, Data[sofar]) == HAL_OK)
	     {
	    	 StartPageAddress += 8;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	   /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/
	   HAL_FLASH_Lock();

	   return 0;
}


void Flash_Read_Data (uint32_t StartPageAddress, uint64_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{

		*RxBuf = *(__IO uint64_t *)StartPageAddress;
		StartPageAddress += 8;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
}


uint32_t Set_Update_Firm_Completed(uint64_t ModeSel)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;

	  /* Unlock the Flash to enable the flash control register access *************/
	   HAL_FLASH_Unlock();

		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_SR_PROGERR | FLASH_FLAG_PGSERR);
	   /* Erase the user Flash area*/


	   /* Fill EraseInit structure*/
	   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	   EraseInitStruct.Page 			 = 127;
		 EraseInitStruct.Banks			 = FLASH_BANK_1;
		 EraseInitStruct.NbPages     = 1;
	   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	   {
	     /*Error occurred while page erase.*/
		  return HAL_FLASH_GetError ();
	   }

	   /* Program the user Flash area word by word*/

				
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0803F800, ModeSel) == HAL_OK)
	     {

	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
			 }

	   /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/
	   HAL_FLASH_Lock();
		return 0;
}
