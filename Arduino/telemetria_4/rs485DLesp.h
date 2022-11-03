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

//extern uint32_t rejectedFramesInRow_DL;
typedef struct RS485_BUFFER_DL
{
  uint8_t tx;
  uint8_t rx;
} RS485_BUFF_DL;
//extern RS485_BUFFER_DL RS485_BUFF_DL;  /////////////// Poprawic i znalezc

/**
 * @struct RS485_RECEIVED_VERIFIED_DATA_DL
 * @brief Struktura zawierajaca otrzymane dane
 */
 extern uint32_t RS485_RX_VERIFIED_DATA_DL;
typedef struct RS485_RX_VERIFIED_DATA_DL
{
  ///< ELEMENTY W STRUKTURZE MUSZA BYC POSORTOWANE W PORZADKU MALEJACYM
  ///< https://www.geeksforgeeks.org/is-sizeof-for-a-struct-equal-to-the-sum-of-sizeof-of-each-member/
 typedef union FCC_V
  {
	 float value;
	 uint8_t array[4];
  } ;
  typedef union FCC_TEMP 
   {
     float value;
     uint8_t array[4];
   } ;
   typedef union CURRENT_SENSOR_FC_TO_SCC
    {
      float value;
      char array[4];
    } ;
   typedef union SCC_V
     {
       float value;
       uint8_t array[4];
     } ;
     typedef union fcFanRPMC
     {
       uint16_t value;
       uint8_t array[2];
     } ;

 // uint8_t fcToScMosfetPWM;
 typedef uint8_t emergencyC;

//}  ;
} RS485_RECEIVED_VERIFIED_DATA_DL;
//extern RS485_RECEIVED_VERIFIED_DATA_DL RS485_RX_VERIFIED_DATA_DL;
