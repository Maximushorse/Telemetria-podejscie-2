/**
* @file rs485.c
* @brief Biblioteka do obslugi komunikacji UART <-> RS485 <-> UART z data loggerem
* @author Alicja Miekina na podstawie Piotra Durakiewicza
* @date 22.04.2021
* @todo
* @bug
* @copyright 2021 HYDROGREEN TEAM
*/

#include "rs485DL.h"
#include "usart.h"
#include "rxinterrupt.h"
#include "rs485SW.h"
#include "rs485EF.h"

// ******************************************************************************************************************************************************** //
//Ramka danych z przeplywem energii
#define UART_PORT_RS485_DL		huart4
#define TX_FRAME_LENGHT_DL 		41					///< Dlugosc wysylanej ramki danych (z suma CRC)
#define RX_FRAME_LENGHT_DL 		21					///< Dlugosc otrzymywanej ramki danych (z suma CRC)
#define EOT_BYTE_DL			    0x17				///< Bajt wskazujacy na koniec ramki

// ******************************************************************************************************************************************************** //
//Zmienne dla transmisji danych z przeplywem energii
uint8_t dataFromRx_DL[RX_FRAME_LENGHT_DL]; 				///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
uint16_t posInRxTab_DL=0;									///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
volatile static uint8_t intRxCplt_DL; 									///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)
static uint8_t dataToTx_DL[TX_FRAME_LENGHT_DL]; 						///< Tablica w ktorej zawarta jest ramka danych do wyslania
static uint16_t posInTxTab_DL=0;											///< Aktualna pozycja w tabeli wykorzystywanej do wysylania danych
uint8_t rs485_flt_DL = RS485_NEW_DATA_TIMEOUT_DL;						///< Zmienna przechowujaca aktualny kod bledu magistrali
uint32_t rejectedFramesInRow_DL=0;
// ******************************************************************************************************************************************************** //


/**
* @struct RS485_BUFFER
* @brief Struktura zawierajaca bufory wykorzystywane do transmisji danych
*/


RS485_BUFFER_DL RS485_BUFF_DL;

RS485_RECEIVED_VERIFIED_DATA_DL RS485_RX_VERIFIED_DATA_DL;
//RS485_NEW_DATA_DL RS485_TX_DATA_DL;

// ******************************************************************************************************************************************************** //
//dla przeplywu energii
static void receiveData_DL(void);
static void prepareNewDataToSend_DL(void);
static void processReceivedData_DL(void);
static void resetActData_DL(void);

// ******************************************************************************************************************************************************** //
//uint8_t endofRX_DL=0;
/**
 * crc_calc(void)
 * Funkcja obliczajaca sume kontrolna dla RX
 */

uint8_t crc_calc_DL(void)
{
  	uint8_t crcSumOnMCU_DL = 0xFF;
  	uint8_t xbit, data1 =1;
	#define polynomial 0x7;

  	for(uint8_t l = 0; l < RX_FRAME_LENGHT_DL-1; l++){
  	uint8_t data = dataFromRx_DL[l];
  	xbit = data1 << 7;
  	for(uint8_t k = sizeof(RX_FRAME_LENGHT_DL-1)*8; k > 0;--k) // obliczanie wartosci najbardziej znaczacego bitu
  	{
  		if(crcSumOnMCU_DL & 0x80)    //jesli najbardziej znaczacy bit = 1
  		{
  			crcSumOnMCU_DL = (crcSumOnMCU_DL << 1)^polynomial; //XOR i leftshift
  		}else { //jesli = 0
  			crcSumOnMCU_DL = (crcSumOnMCU_DL << 1); //leftshift
  		}
  		if(data & xbit){
  			crcSumOnMCU_DL = crcSumOnMCU_DL ^polynomial;
  		}
  		xbit >>= 1;
  	}
  	}
  	return crcSumOnMCU_DL;

}
/**
 * crc_calc(void)
 * Funkcja obliczajaca sume kontrolna dla TX
 */

uint8_t crc_calc_TX_DL(void)
{
  	uint8_t crcSumOnMCUTX_DL = 0xFF;
  	uint8_t xbit, data1 =1;
	#define polynomial 0x7;

  	for(uint8_t l = 0; l < TX_FRAME_LENGHT_DL-1; l++){
  	uint8_t data = dataToTx_DL[l];
  	xbit = data1 << 7;
  	for(uint8_t k = sizeof(TX_FRAME_LENGHT_DL-1)*8; k > 0;--k) // obliczanie wartosci najbardziej znaczacego bitu
  	{
  		if(crcSumOnMCUTX_DL & 0x80)    //jesli najbardziej znaczacy bit = 1
  		{
  			crcSumOnMCUTX_DL = (crcSumOnMCUTX_DL << 1)^polynomial; //XOR i leftshift
  		}else { //jesli = 0
  			crcSumOnMCUTX_DL = (crcSumOnMCUTX_DL << 1); //leftshift
  		}
  		if(data & xbit){
  			crcSumOnMCUTX_DL = crcSumOnMCUTX_DL ^polynomial;
  		}
  		xbit >>= 1;
  	}
  	}
  	return crcSumOnMCUTX_DL;

}
/**
* @fn rs485_init_DL(void)
* @brief Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void) dla kierownicy
*/
void rs485_init_DL(void)
{
  HAL_UART_Receive_DMA(&UART_PORT_RS485_DL, &RS485_BUFF_DL.rx, 1);				//Rozpocznij nasluchiwanie
  prepareNewDataToSend_DL();								//Przygotuj nowy pakiet danych
}

/**
* @fn rs485_step(void)
* @brief Funkcje obslugujace magistrale, umiescic wewnatrz hydrogreen_step(void) dla kierownicy i przeplywu energii
*/
void rs485_step_DL(void)
{

	if(endofRX_SW == 0 && endofRX_EF==1){
		sendData_DL();
	}

}


/**
* @fn receiveData(void)
* @brief Funkcja ktorej zadaniem jest obsluga linii RX, umiescic wewnatrz rs485_step() przeplywu energii
*/
static void receiveData_DL(void)
{
 // static uint32_t rejectedFramesInRow_EF;							//Zmienna przechowujaca liczbe straconych ramek z rzedu
  static uint32_t cntEndOfRxTick_DL;							//Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

  //Sprawdz czy otrzymano nowe dane
  if (!intRxCplt_DL)
    {
      //Nie otrzymano nowych danych, zacznij odliczac czas
      cntEndOfRxTick_DL++;
    }
  else if (intRxCplt_DL)
    {
      //Nowe dane zostaly otrzymane, zeruj flage informujaca o zakonczeniu transmisji
      intRxCplt_DL = 0;
    }

  //Sprawdz czy minal juz czas wynoszacy RX_FRAME_LENGHT
  if (cntEndOfRxTick_DL > RX_FRAME_LENGHT_DL)
    {
      //Na czas przetwarzania danych wylacz przerwania
      __disable_irq();

      //Czas minal, oznacza to koniec ramki
      cntEndOfRxTick_DL = 0;
      posInRxTab_DL = 0;
    //  endofRX_DL = 1;
     // endofRX_SW = 0;
     // endofRX_EF = 0;
      //OBLICZ SUME KONTROLNA

      //Sprawdz czy sumy kontrolne oraz bajt EOT (End Of Tranmission) sie zgadzaja
      if ( (dataFromRx_DL[RX_FRAME_LENGHT_DL - 2] == EOT_BYTE_DL) && (crc_calc_DL() == dataFromRx_DL[RX_FRAME_LENGHT_DL - 1]) )
	{

	  processReceivedData_DL();
	  rs485_flt_DL = RS485_FLT_NONE_DL;
	  rejectedFramesInRow_DL = 0;
	}
      else
	{
    	//processReceivedData_EF();
	    rejectedFramesInRow_DL++;

	  //Jezeli odrzucono wiecej niz 50 ramek z rzedu uznaj ze tranmisja zostala zerwana
	  if (rejectedFramesInRow_DL > 100)
	    {
	      resetActData_DL();
	      rs485_flt_DL = RS485_NEW_DATA_TIMEOUT_DL;
			RS485_TX_DATA_SW.emergencyButton = 1;
			RS485_TX_DATA_EF.emergencyScenario = 1;
			HAL_GPIO_WritePin(GPIOC, Solenoid_Valve_GPIO_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Emergency_Relay_GPIO_Pin, GPIO_PIN_RESET);
			RS485_TX_DATA_EF.motorPWM = 0;
			RS485_TX_DATA_SW.motorPWM = 0;
	    }
	}

      //Wyczysc bufor odbiorczy
      for (uint8_t i = 0; i < RX_FRAME_LENGHT_DL; i++)
	{
	  dataFromRx_DL[i] = 0x00;
	}

      __enable_irq();
    }
}

void HAL_UART_RxCpltCallback();

/**
* @fn prepareNewDataToSend(void)
* @brief Funkcja przygotowujaca dane do wysylki, wykorzystana wewnatrz sendData(void) dla przeplywu energii
*/
static void prepareNewDataToSend_DL(void)
{

  uint8_t j = 0;

  dataToTx_DL[j] = RS485_TX_DATA_EF.fuellCellModeButtons;
  dataToTx_DL[++j] = RS485_RX_VERIFIED_DATA_SW.scClose;
  dataToTx_DL[++j] = RS485_TX_DATA_EF.emergencyScenario;
  dataToTx_DL[++j] = RS485_TX_DATA_EF.motorPWM;

  dataToTx_DL[++j] = RS485_TX_DATA_SW.interimSpeed;
   dataToTx_DL[++j] = RS485_TX_DATA_SW.averageSpeed;

   for(uint8_t k = 0; k < 2; k++)
   {
 	  dataToTx_DL[++j] = RS485_TX_DATA_SW.laptime_minutes.array[k];
   }
   dataToTx_DL[++j] = RS485_TX_DATA_SW.laptime_seconds;

   for(uint8_t k = 0; k < 2; k++)
   {
 	  dataToTx_DL[++j] = RS485_TX_DATA_SW.laptime_miliseconds.array[k];
   }
   for(uint8_t k = 0; k < 4; k++)
   {
 	  dataToTx_DL[++j] =  RS485_RX_VERIFIED_DATA_EF.FC_V.array[k];
   }
   for(uint8_t k = 0; k < 4; k++)
   {
 	  dataToTx_DL[++j] = RS485_RX_VERIFIED_DATA_EF.FC_TEMP.array[k];
   }
   for(uint8_t k = 0; k < 2; k++)
   {
 	  dataToTx_DL[++j] = RS485_RX_VERIFIED_DATA_EF.fcFanRPM.array[k];
   }
   for(uint8_t k = 0; k < 4; k++)
   {
 	  dataToTx_DL[++j] = RS485_RX_VERIFIED_DATA_EF.CURRENT_SENSOR_FC_TO_SC.array[k];
   }
   for(uint8_t k = 0; k < 4; k++)
   {
 	  dataToTx_DL[++j] = RS485_TX_DATA_SW.CURRENT_SENSOR_SC_TO_MOTOR.array[k];
   }
   dataToTx_DL[++j] = RS485_TX_DATA_SW.fcToScMosfetPWM;
   dataToTx_DL[++j] = RS485_TX_DATA_SW.motorPWM;

   for(uint8_t k = 0; k < 4; k++)
   {
 	  dataToTx_DL[++j] = RS485_RX_VERIFIED_DATA_EF.SC_V.array[k];
   }
   for (uint8_t k = 0; k < 4; k++)
     {
 	  dataToTx_DL[++j]=RS485_TX_DATA_SW.H2_SENSOR_V.array[k];
     }

   dataToTx_DL[++j] = RS485_TX_DATA_SW.h2SensorDigitalPin;
   dataToTx_DL[++j] = RS485_TX_DATA_SW.emergencyButton;
  dataToTx_DL[++j] = EOT_BYTE_DL;

  //OBLICZ SUME KONTROLNA

  //Wrzuc obliczona sume kontrolna na koniec wysylanej tablicy
  dataToTx_DL[TX_FRAME_LENGHT_DL - 1] = crc_calc_TX_DL();
}


/**
* @fn processReveivedData_EF()
* @brief Funkcja przypisujaca odebrane dane do zmiennych docelowych dla przeplywu energii
*/
static void processReceivedData_DL(void)
{
  uint8_t i = 0;


  RS485_RX_VERIFIED_DATA_DL.emergencyC = dataFromRx_DL[i];

  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_DL.FCC_V.array[k] = dataFromRx_DL[++i];
    }

  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_DL.FCC_TEMP.array[k] = dataFromRx_DL[++i];
    }
  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_DL.CURRENT_SENSOR_FC_TO_SCC.array[k] = dataFromRx_DL[++i];
    }

  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_DL.SCC_V.array[k] = dataFromRx_DL[++i];
    }

  for (uint8_t k = 0; k < 2; k++)
    {
      RS485_RX_VERIFIED_DATA_DL.fcFanRPMC.array[k] = dataFromRx_DL[++i];
    }
}
/**
* @fn resetActData_DL
* @brief Zerowanie zmiennych docelowych (odbywa sie m.in w przypadku zerwania transmisji) dla przeplywu energii
*/
static void resetActData_DL(void)
{
	RS485_RX_VERIFIED_DATA_DL.emergencyC = 0;

  for (uint8_t k = 0; k < 4; k++)
    {
	  RS485_RX_VERIFIED_DATA_DL.FCC_V.array[k] = 0;
    }

  for (uint8_t k = 0; k < 4; k++)
    {
	  RS485_RX_VERIFIED_DATA_DL.FCC_TEMP.array[k] = 0;
    }

  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_DL.CURRENT_SENSOR_FC_TO_SCC.array[k] = 0;
    }

  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_DL.SCC_V.array[k] = 0;
    }
  RS485_RX_VERIFIED_DATA_DL.fcFanRPMC.value = 0;
}
