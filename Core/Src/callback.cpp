//
// Created by chill on 2024/10/26.
//
//
// Created by chill on 2024/10/12.
//
#include "can.h"
#include "M2006_Motor.h"
#include "main.h"

extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t rx_data[8];
extern CAN_TxHeaderTypeDef TxHeader;
extern uint8_t tx_data[8];
M2006_Motor Motor;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if (hcan == &hcan1){
    HAL_CAN_GetRxMessage(&hcan1, CAN_FilterFIFO0, &RxHeader, rx_data);
    Motor.CanRxMsgCallback(rx_data);
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, CAN_FilterFIFO0);
  }
}
