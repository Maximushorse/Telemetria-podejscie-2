/**
* @file rs485.h
* @brief Biblioteka dla przerwania od RX
* @author Piotr Durakiewicz
* @date 08.12.2020
* @todo
* @bug
* @copyright 2020 HYDROGREEN TEAM
*/
#pragma once

#include <stdint-gcc.h>

// ******************************************************************************************************************************************************** //


extern void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
// ******************************************************************************************************************************************************** //


