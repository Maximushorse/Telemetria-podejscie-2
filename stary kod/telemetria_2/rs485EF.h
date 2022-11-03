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

#define RS485_FLT_NONE_EF 0x00					///< Brak bledu
#define RS485_NEW_DATA_TIMEOUT_EF 0x11				///< Nie otrzymano nowych dane (polaczenie zostalo zerwane)

extern uint8_t rs485_flt_EF; 					///< Zmienna przechowujaca aktualny kod bledu magistrali
extern uint8_t dupa;
// ******************************************************************************************************************************************************** //

extern void rs485_init_EF(void);					///< Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void)
extern void rs485_step_EF(void);					///< Funkcja obslugujaca magistrale, umiescic wewnatrz hydrogreen_step(void)

#define UART_PORT_RS485_EF 		huart1
#define RX_FRAME_LENGHT_EF 		21					///< Dlugosc otrzymywanej ramki danych (z suma CRC)#define EOT_BYTE_EF			    0x17				///< Bajt wskazujacy na koniec ramki

// ******************************************************************************************************************************************************** //
//Zmienne dla transmisji danych z przeplywem energii
extern uint8_t dataFromRx_EF[RX_FRAME_LENGHT_EF]; 				///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
extern uint16_t posInRxTab_EF;		///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
extern uint8_t crcSumOnMCU_EF;
volatile static uint8_t intRxCplt_EF; 									///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)
uint8_t crc_calc_TX(void);
uint8_t crc_calc(void);
extern uint32_t rejectedFramesInRow_EF;
typedef struct
{
  uint8_t tx;
  uint8_t rx;
} RS485_BUFFER_EF;
extern RS485_BUFFER_EF RS485_BUFF_EF;
// ******************************************************************************************************************************************************** //
extern uint8_t endofRX_EF, endofTX_EF;
/**
 * @struct RS485_RECEIVED_VERIFIED_DATA_EF
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
  } FC_V;
  union
   {
     float value;
     uint8_t array[4];
   } FC_TEMP;
   union
    {
      float value;
      char array[4];
    } CURRENT_SENSOR_FC_TO_SC;
    union
     {
       float value;
       uint8_t array[4];
     } SC_V;
     union
     {
       uint16_t value;
       uint8_t array[2];
     } fcFanRPM;

 // uint8_t fcToScMosfetPWM;
  uint8_t emergency;

} RS485_RECEIVED_VERIFIED_DATA_EF;
extern RS485_RECEIVED_VERIFIED_DATA_EF RS485_RX_VERIFIED_DATA_EF;
/**
* @struct RS485_NEW_DATA_EF
* @brief Struktura zawierajaca wysylane dane
*/
typedef struct
{
	uint8_t fuellCellModeButtons;				//Stan 3 przycisków: fuellCellPrepareToRace, fuelCellRace, fuelCellOff
	uint8_t scOn;
	uint8_t emergencyScenario;
	uint8_t motorPWM;
}RS485_NEW_DATA_EF;
extern RS485_NEW_DATA_EF RS485_TX_DATA_EF;
