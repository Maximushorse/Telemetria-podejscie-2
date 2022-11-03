/**
* @file rs485.h
* @brief Biblioteka do obslugi komunikacji UART <-> RS485 <-> UART
* @author Piotr Durakiewicz
* @date 08.12.2020
* @todo
* @bug
* @copyright 2020 HYDROGREEN TEAM
*/
#pragma once

#include <stdint-gcc.h>

// ******************************************************************************************************************************************************** //

#define RS485_FLT_NONE_DL 0x00					///< Brak bledu
#define RS485_NEW_DATA_TIMEOUT_DL 0x11				///< Nie otrzymano nowych dane (polaczenie zostalo zerwane)

extern uint8_t rs485_flt_DL; 					///< Zmienna przechowujaca aktualny kod bledu magistrali

// ******************************************************************************************************************************************************** //

extern void rs485_init_DL(void);					///< Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void)
extern void rs485_step_DL(void);					///< Funkcja obslugujaca magistrale, umiescic wewnatrz hydrogreen_step(void)

#define UART_PORT_RS485_DL		huart4
#define RX_FRAME_LENGHT_DL 		21					///< Dlugosc otrzymywanej ramki danych (z suma CRC)#define EOT_BYTE_EF			    0x17				///< Bajt wskazujacy na koniec ramki

// ******************************************************************************************************************************************************** //
//Zmienne dla transmisji danych z przeplywem energii
extern uint8_t dataFromRx_DL[RX_FRAME_LENGHT_DL]; 				///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
extern uint16_t posInRxTab_DL;		///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
extern uint8_t crcSumOnMCU_DL;
volatile static uint8_t intRxCplt_DL; 									///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)
uint8_t crc_calc_TX_DL(void);
uint8_t crc_calc_DL(void);
extern uint32_t rejectedFramesInRow_DL;
typedef struct
{
  uint8_t tx;
  uint8_t rx;
} RS485_BUFFER_DL;
extern RS485_BUFFER_DL RS485_BUFF_DL;
// ******************************************************************************************************************************************************** //

/**
 * @struct RS485_RECEIVED_VERIFIED_DATA_DL
 * @brief Struktura zawierajaca otrzymane dane
 */
typedef struct
{
  ///< ELEMENTY W STRUKTURZE MUSZA BYC POSORTOWANE W PORZADKU MALEJACYM
  ///< https://www.geeksforgeeks.org/is-sizeof-for-a-struct-equal-to-the-sum-of-sizeof-of-each-member/
  union
  {
	 float value;
	 uint8_t array[4];
  } FCC_V;
  union
   {
     float value;
     uint8_t array[4];
   } FCC_TEMP;
   union
    {
      float value;
      char array[4];
    } CURRENT_SENSOR_FC_TO_SCC;
    union
     {
       float value;
       uint8_t array[4];
     } SCC_V;
     union
     {
       uint16_t value;
       uint8_t array[2];
     } fcFanRPMC;

 // uint8_t fcToScMosfetPWM;
  uint8_t emergencyC;

} RS485_RECEIVED_VERIFIED_DATA_DL;
extern RS485_RECEIVED_VERIFIED_DATA_DL RS485_RX_VERIFIED_DATA_DL;
/**
* @struct RS485_NEW_DATA_DL
* @brief Struktura zawierajaca wysylane dane
*/
/*
typedef struct
{
	uint8_t fuellCellModeButtonsC;				//Stan 3 przycisków: fuellCellPrepareToRace, fuelCellRace, fuelCellOff
	uint8_t scOnC;
	uint8_t emergencyScenarioC;
	uint8_t motorPWMC;
}RS485_NEW_DATA_DL;
extern RS485_NEW_DATA_DL RS485_TX_DATA_DL;
*/
