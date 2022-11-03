/**
* @file rs485.c
* @brief Biblioteka dla przerwania od RX
* @author Alicja Miekina
* @date 22.04.2021
* @todo
* @bug
* @copyright 2021 HYDROGREEN TEAM
*/

#include "rs485EF.h"
#include "rs485SW.h"
#include "rs485DL.h"




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 if (huart == &UART_PORT_RS485_SW){
		 HAL_UART_Receive_DMA(&UART_PORT_RS485_SW, &RS485_BUFF_SW.rx, 1);			//Ponownie rozpocznij nasluchiwanie nasluchiwanie

		  intRxCplt_SW = 1;								//Ustaw flage informujaca o otrzymaniu nowych danych

		  if (posInRxTab_SW > RX_FRAME_LENGHT_SW) posInRxTab_SW = 0;				//Zabezpieczenie przed wyjsciem poza zakres tablicy

		  dataFromRx_SW[posInRxTab_SW] = RS485_BUFF_SW.rx;					//Przypisz otrzymany bajt do analizowanej tablicy
		  posInRxTab_SW++;
	}
	 else if(huart == &UART_PORT_RS485_EF ){
  HAL_UART_Receive_DMA(&UART_PORT_RS485_EF, &RS485_BUFF_EF.rx, 1);			//Ponownie rozpocznij nasluchiwanie nasluchiwanie

  intRxCplt_EF = 1;								//Ustaw flage informujaca o otrzymaniu nowych danych

  if (posInRxTab_EF > RX_FRAME_LENGHT_EF) posInRxTab_EF = 0;				//Zabezpieczenie przed wyjsciem poza zakres tablicy

  dataFromRx_EF[posInRxTab_EF] = RS485_BUFF_EF.rx;					//Przypisz otrzymany bajt do analizowanej tablicy
  posInRxTab_EF++;
	 }

	 else if(huart == &UART_PORT_RS485_DL ){
  HAL_UART_Receive_DMA(&UART_PORT_RS485_DL, &RS485_BUFF_DL.rx, 1);			//Ponownie rozpocznij nasluchiwanie nasluchiwanie

  intRxCplt_DL = 1;								//Ustaw flage informujaca o otrzymaniu nowych danych

  if (posInRxTab_DL > RX_FRAME_LENGHT_DL) posInRxTab_DL = 0;				//Zabezpieczenie przed wyjsciem poza zakres tablicy

  dataFromRx_DL[posInRxTab_DL] = RS485_BUFF_DL.rx;					//Przypisz otrzymany bajt do analizowanej tablicy
  posInRxTab_DL++;}


}
