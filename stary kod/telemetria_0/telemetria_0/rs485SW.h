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

#define RS485_FLT_NONE_SW 0x00					///< Brak bledu
#define RS485_NEW_DATA_TIMEOUT_SW 0x11				///< Nie otrzymano nowych dane (polaczenie zostalo zerwane)

extern uint8_t rs485_flt_SW; 					///< Zmienna przechowujaca aktualny kod bledu magistrali

// ******************************************************************************************************************************************************** //

extern void rs485_init_SW(void);					///< Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void)
extern void rs485_step_SW(void);					///< Funkcja obslugujaca magistrale, umiescic wewnatrz hydrogreen_step(void)

#define UART_PORT_RS485_SW 		huart3
#define RX_FRAME_LENGHT_SW 		11					///< Dlugosc otrzymywanej ramki danych (z suma CRC)



typedef struct
{
  uint8_t tx;
  uint8_t rx;
} RS485_BUFFER_SW;
extern RS485_BUFFER_SW RS485_BUFF_SW;
// ******************************************************************************************************************************************************** //

extern uint8_t dataFromRx_SW[RX_FRAME_LENGHT_SW]; 				///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
extern uint16_t posInRxTab_SW;									///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
volatile static uint8_t intRxCplt_SW; 									///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)

// ******************************************************************************************************************************************************** //
extern uint8_t endofRX_SW;
/*
typedef struct
{
  uint8_t endofRX_SW;
  uint8_t endofTX_SW;
  uint8_t endofstep_SW;
} RS485_FLAGS;
extern RS485_FLAGS RS485_FLAGS_SW;
*/
/**
 * @struct RS485_RECEIVED_VERIFIED_DATA_SW
 * @brief Struktura zawierajaca otrzymane dane
 */
typedef struct
{
	uint8_t halfGas;
	uint8_t fullGas;
	uint8_t horn;
	uint8_t speedReset;
	uint8_t powerSupply;
	uint8_t scClose;
	uint8_t fuelcellOff;
	uint8_t fuelcellPrepareToRace;
	uint8_t fuelcellRace;
}RS485_RECEIVED_VERIFIED_DATA_SW;
extern RS485_RECEIVED_VERIFIED_DATA_SW RS485_RX_VERIFIED_DATA_SW;
/**
* @struct RS485_NEW_DATA_SW
* @brief Struktura zawierajaca wysylane dane
*/
typedef struct
{
  ///< ELEMENTY W STRUKTURZE MUSZA BYC POSORTOWANE W PORZADKU MALEJACYM
  ///< https://www.geeksforgeeks.org/is-sizeof-for-a-struct-equal-to-the-sum-of-sizeof-of-each-member/
  union
  {
    float value;
    char array[4];
  } FC_V;

  union
  {
    float value;
    char array[4];
  } H2_SENSOR_V;

  union
  {
    float value;
    char array[4];
  } FC_TEMP;

  union
  {
    float value;
    char array[4];
  } CURRENT_SENSOR_FC_TO_SC;

  union
  {
    float value;
    char array[4];
  } CURRENT_SENSOR_SC_TO_MOTOR;

  union
  {
    float value;
    char array[4];
  } SC_V;

  union
  {
    uint16_t value;
    char array[2];
  } laptime_minutes;

  union
  {
    uint16_t value;
    char array[2];
  } laptime_miliseconds;

  union
  {
    uint16_t value;
    char array[2];
  } fcFanRPM;

  uint8_t interimSpeed;
  uint8_t averageSpeed;
  uint8_t laptime_seconds;

  uint8_t electrovalve;
  uint8_t purgeValve;

  uint8_t fcToScMosfetPWM;
  uint8_t motorPWM;

  uint8_t h2SensorDigitalPin;
  uint8_t emergencyButton;
} RS485_NEW_DATA_SW;
extern RS485_NEW_DATA_SW RS485_TX_DATA_SW;
