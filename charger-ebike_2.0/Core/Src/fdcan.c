
#include "fdcan.h"



FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader0;
FDCAN_RxHeaderTypeDef RxHeader1;

uint8_t RxData0[8] = {0};
uint8_t RxData1[8] = {0};

extern volatile uint16_t AuxBattery0_Info[];
extern volatile uint16_t AuxBattery1_Info[];
extern uint8_t enable_charger_controller;
extern volatile uint8_t flag_BMS0_alive, flag_BMS1_alive;

uint16_t BMS0_TXID[MAX_MESSAGE_ID] = {
	0x700,
	0x710,
	0x720,
	0x730,
	0x740,
	0x750,
	0x760,
	0x770,
	0x780,
	0x790,
	0x7a0,
	0x7b0,
};

uint16_t BMS1_TXID[MAX_MESSAGE_ID] = {
	0x701,
	0x711,
	0x721,
	0x731,
	0x741,
	0x751,
	0x761,
	0x771,
	0x781,
	0x791,
	0x7a1,
	0x7b1,
};


/**
  * @brief  Configures the FDCAN.
  * @param  None
  * @retval None
  */
void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter 0 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x700;						// filter
  sFilterConfig.FilterID2 = 0x701;						// mask
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
  /* Configure Rx filter 1 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x701;						// filter
  sFilterConfig.FilterID2 = 0x701;						// mask 
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
  /* Configure Rx filter 2, filter only IP 0x590 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 2;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x590;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
//	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
	
  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }	

//  /* Prepare Tx Header */
  TxHeader.Identifier = 0x700;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}


// Handler for BMS0
uint8_t handlerBMS0_0(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {
	if (RxData0[0] == BMS_ALIVE)
		flag_BMS0_alive = 1;
	else
		flag_BMS0_alive = 0;
	return 0x00;
}

uint8_t handlerBMS0_1(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {

	AuxBattery0_Info[0] = ((RxData0[0] & 0xFF) << 8) | (RxData0[1] & 0xFF);
	AuxBattery0_Info[1] = ((RxData0[2] & 0xFF) << 8) | (RxData0[3] & 0xFF);
	AuxBattery0_Info[2] = ((RxData0[4] & 0xFF) << 8) | (RxData0[5] & 0xFF);
	AuxBattery0_Info[3] = ((RxData0[6] & 0xFF) << 8) | (RxData0[7] & 0xFF);
	return 0x00;
}


uint8_t handlerBMS0_2(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {
	uint8_t TxData[8] = {0};
	AuxBattery0_Info[4] = ((RxData0[0] & 0xFF) << 8) | (RxData0[1] & 0xFF);
	AuxBattery0_Info[5] = ((RxData0[2] & 0xFF) << 8) | (RxData0[3] & 0xFF);
	AuxBattery0_Info[6] = ((RxData0[4] & 0xFF) << 8) | (RxData0[5] & 0xFF);
	AuxBattery0_Info[7] = ((RxData0[6] & 0xFF) << 8) | (RxData0[7] & 0xFF);

	return 0x00;
}

uint8_t handlerBMS0_3(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {
	AuxBattery0_Info[8] = ((RxData0[0] & 0xFF) << 8) | (RxData0[1] & 0xFF);
	AuxBattery0_Info[9] = ((RxData0[2] & 0xFF) << 8) | (RxData0[3] & 0xFF);
	AuxBattery0_Info[10] = ((RxData0[4] & 0xFF) << 8) | (RxData0[5] & 0xFF);
	AuxBattery0_Info[11] = ((RxData0[6] & 0xFF) << 8) | (RxData0[7] & 0xFF);

	return 0x00;
}

uint8_t handlerBMS0_4(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {
	AuxBattery0_Info[12] = ((RxData0[0] & 0xFF) << 8) | (RxData0[1] & 0xFF);
	AuxBattery0_Info[13] = ((RxData0[2] & 0xFF) << 8) | (RxData0[3] & 0xFF);
	AuxBattery0_Info[14] = ((RxData0[4] & 0xFF) << 8) | (RxData0[5] & 0xFF);
	AuxBattery0_Info[15] = ((RxData0[6] & 0xFF) << 8) | (RxData0[7] & 0xFF);

	return 0x00;
}

uint8_t handlerBMS0_b(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {

	return 0x00;
}

// Handler for BMS1
uint8_t handlerBMS1_0(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {
	if (RxData1[0] == BMS_ALIVE)
		flag_BMS1_alive = 1;
	else
		flag_BMS1_alive = 0;
	return 0x00;
}

uint8_t handlerBMS1_1(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {

	AuxBattery1_Info[0] = ((RxData1[0] & 0xFF) << 8) | (RxData1[1] & 0xFF);
	AuxBattery1_Info[1] = ((RxData1[2] & 0xFF) << 8) | (RxData1[3] & 0xFF);
	AuxBattery1_Info[2] = ((RxData1[4] & 0xFF) << 8) | (RxData1[5] & 0xFF);
	AuxBattery1_Info[3] = ((RxData1[6] & 0xFF) << 8) | (RxData1[7] & 0xFF);
	return 0x00;
}


uint8_t handlerBMS1_2(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {
	uint8_t TxData[8] = {0};
	AuxBattery1_Info[4] = ((RxData1[0] & 0xFF) << 8) | (RxData1[1] & 0xFF);
	AuxBattery1_Info[5] = ((RxData1[2] & 0xFF) << 8) | (RxData1[3] & 0xFF);
	AuxBattery1_Info[6] = ((RxData1[4] & 0xFF) << 8) | (RxData1[5] & 0xFF);
	AuxBattery1_Info[7] = ((RxData1[6] & 0xFF) << 8) | (RxData1[7] & 0xFF);

	return 0x00;
}

uint8_t handlerBMS1_3(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {
	AuxBattery1_Info[8] = ((RxData1[0] & 0xFF) << 8) | (RxData1[1] & 0xFF);
	AuxBattery1_Info[9] = ((RxData1[2] & 0xFF) << 8) | (RxData1[3] & 0xFF);
	AuxBattery1_Info[10] = ((RxData1[4] & 0xFF) << 8) | (RxData1[5] & 0xFF);
	AuxBattery1_Info[11] = ((RxData1[6] & 0xFF) << 8) | (RxData1[7] & 0xFF);

	return 0x00;
}

uint8_t handlerBMS1_4(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {
	AuxBattery1_Info[12] = ((RxData1[0] & 0xFF) << 8) | (RxData1[1] & 0xFF);
	AuxBattery1_Info[13] = ((RxData1[2] & 0xFF) << 8) | (RxData1[3] & 0xFF);
	AuxBattery1_Info[14] = ((RxData1[4] & 0xFF) << 8) | (RxData1[5] & 0xFF);
	AuxBattery1_Info[15] = ((RxData1[6] & 0xFF) << 8) | (RxData1[7] & 0xFF);

	return 0x00;
}

uint8_t handlerBMS1_b(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef RxHeader) {
	return 0x00;
}


CanIDHandlerMap_t CanHandlers[MAX_MESSAGE_ID] = {
	{0x700, handlerBMS0_0},
	{0x710, handlerBMS0_1},
	{0x720, handlerBMS0_2},
	{0x730, handlerBMS0_3},
	{0x740, handlerBMS0_4},
	{0x750, handlerBMS0_0},
	{0x760, handlerBMS0_0},
	{0x770, handlerBMS0_0},
	{0x780, handlerBMS0_0},
	{0x790, handlerBMS0_0},
	{0x7a0, handlerBMS0_0},
	{0x7b0, handlerBMS0_b},
};

CanIDHandlerMap_t CanHandlersFIFO1[MAX_MESSAGE_ID] = {
	{0x701, handlerBMS1_0},
	{0x711, handlerBMS1_1},
	{0x721, handlerBMS1_2},
	{0x731, handlerBMS1_3},
	{0x741, handlerBMS1_4},
	{0x751, handlerBMS1_0},
	{0x761, handlerBMS1_0},
	{0x771, handlerBMS1_0},
	{0x781, handlerBMS1_0},
	{0x791, handlerBMS1_0},
	{0x7a1, handlerBMS1_0},
	{0x7b1, handlerBMS1_b},
};

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
int count_rev = 0;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader0, RxData0) != HAL_OK)
    {
      Error_Handler();
    }
		uint32_t receivedID = RxHeader0.Identifier;
		if (receivedID == 0x590) {
			enable_charger_controller = 1;
		}
		if (RxHeader0.RxFrameType == FDCAN_DATA_FRAME) {
			for (int i = 0; i < MAX_MESSAGE_ID; i++) {
				if (CanHandlers[i].CanID == receivedID) { 
					CanHandlers[i].Handler(hfdcan, RxHeader0);
					break;
				}
			}
		}
  }
}


/**
  * @brief  Rx FIFO 1 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0)
  {
    /* Retrieve Rx messages from RX FIFO1 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader1, RxData1) != HAL_OK)
    {
      Error_Handler();
    }
		uint32_t receivedID = RxHeader1.Identifier;

		if (RxHeader1.RxFrameType == FDCAN_DATA_FRAME) {
			for (int i = 0; i < MAX_MESSAGE_ID; i++) {
				if (CanHandlersFIFO1[i].CanID == receivedID) { 
					CanHandlersFIFO1[i].Handler(hfdcan, RxHeader1);
					break;
				}
			}
		}
  }
}

int CAN_BMS0_read_data(void)
{
	// Loop thorugh CanHandlers to send remote framte with id
	static uint8_t index_BMS0 = 0;
	uint32_t err_count = 0;

	TxHeader.Identifier = BMS0_TXID[index_BMS0];
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	if ((hfdcan2.Instance->TXFQS & FDCAN_TXFQS_TFQF) == 0U) {
		while (err_count < 4) {
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, NULL) != HAL_OK) {
				err_count ++;
			} else {
				break;
			}
		}
		index_BMS0++;
		if (index_BMS0 >= MAX_MESSAGE_ID) {
			index_BMS0 = 0;
			return 0;
		}
	}
	return 1;
}

int CAN_BMS1_read_data(void)
{
	// Loop thorugh CanHandlers to send remote framte with id
	static uint8_t index_BMS1 = 0;
	uint32_t err_count = 0;

	TxHeader.Identifier = BMS1_TXID[index_BMS1];
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	if ((hfdcan2.Instance->TXFQS & FDCAN_TXFQS_TFQF) == 0U) {
		while (err_count < 4) {
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, NULL) != HAL_OK) {
				err_count ++;
			} else {
				break;
			}
		}
		index_BMS1++;
		if (index_BMS1 >= MAX_MESSAGE_ID) {
			index_BMS1 = 0;
			return 0;
		}
	}
	return 1;
}