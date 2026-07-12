#include "CAN.h"

float test_angle_sp = 0.0f;
uint32_t TxMailbox;

// Filter
void CAN_FilterConfig(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 0;
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig.FilterIdHigh = 0x0000;
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0x0000;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &canfilterconfig);
}

// Transmit
void CAN_Send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef TxHeader;

    TxHeader.DLC = len;
    TxHeader.StdId = id;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
        if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox) != HAL_OK) {
            // Error handling
        }
    }
}

void CAN_init(CAN_HandleTypeDef *hcan) {
    CAN_FilterConfig(hcan);
	HAL_CAN_Start(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/******************************************************************************/


/******************************************************************************/
