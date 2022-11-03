/**
* @file rs485.c
* @brief Biblioteka do obslugi komunikacji UART <-> RS485 <-> UART z energyflow
* @author Alicja Miekina na podstawie Piotra Durakiewicza
* @date 22.04.2021
* @todo
* @bug
* @copyright 2021 HYDROGREEN TEAM
*/

#include "rs485EF.h"
#include "usart.h"
#include "rxinterrupt.h"
#include "rs485SW.h"
#include "rs485DL.h"

// ******************************************************************************************************************************************************** //
//Ramka danych z przeplywem energii
#define UART_PORT_RS485_EF 		huart1
#define TX_FRAME_LENGHT_EF 		6					///< Dlugosc wysylanej ramki danych (z suma CRC)
#define RX_FRAME_LENGHT_EF 		21					///< Dlugosc otrzymywanej ramki danych (z suma CRC)
#define EOT_BYTE_EF			    0x17				///< Bajt wskazujacy na koniec ramki

// ******************************************************************************************************************************************************** //
//Zmienne dla transmisji danych z przeplywem energii
uint8_t dataFromRx_EF[RX_FRAME_LENGHT_EF]; 				///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
uint16_t posInRxTab_EF=0;									///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
volatile static uint8_t intRxCplt_EF; 									///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)
static uint8_t dataToTx_EF[TX_FRAME_LENGHT_EF]; 						///< Tablica w ktorej zawarta jest ramka danych do wyslania
static uint16_t posInTxTab_EF=0;											///< Aktualna pozycja w tabeli wykorzystywanej do wysylania danych
uint8_t rs485_flt_EF = RS485_NEW_DATA_TIMEOUT_EF;						///< Zmienna przechowujaca aktualny kod bledu magistrali
uint32_t rejectedFramesInRow_EF=0;
// ******************************************************************************************************************************************************** //


/**
* @struct RS485_BUFFER
* @brief Struktura zawierajaca bufory wykorzystywane do transmisji danych
*/

RS485_BUFFER_EF RS485_BUFF_EF;

RS485_RECEIVED_VERIFIED_DATA_EF RS485_RX_VERIFIED_DATA_EF;
RS485_NEW_DATA_EF RS485_TX_DATA_EF;

// ******************************************************************************************************************************************************** //
//dla przeplywu energii
static void sendData_EF(void);
static void receiveData_EF(void);
static void prepareNewDataToSend_EF(void);
static void processReceivedData_EF(void);
static void resetActData_EF(void);

// ******************************************************************************************************************************************************** //
uint8_t endofRX_EF=0, endofTX_EF=0;
/**
 * crc_calc(void)
 * Funkcja obliczajaca sume kontrolna dla RX
 */

uint8_t crc_calc(void)
{
  	uint8_t crcSumOnMCU_EF = 0xFF;	//suma kontrolna
  	uint8_t xbit, data1 =1;			//najbardziej znaczacy bit
	#define polynomial 0x7;			//wielomian X2+X1+X0

  	for(uint8_t l = 0; l < RX_FRAME_LENGHT_EF-1; l++){	//dla wielkosci ramki danych
  	uint8_t data = dataFromRx_EF[l];					//przypisanie otrzymanej wartosci
  	xbit = data1 << 7;									//leftshift
		for(uint8_t k = sizeof(RX_FRAME_LENGHT_EF-1)*8; k > 0;--k) // obliczanie wartosci najbardziej znaczacego bitu
		{
			if(crcSumOnMCU_EF & 0x80)    //jesli najbardziej znaczacy bit = 1
			{
				crcSumOnMCU_EF = (crcSumOnMCU_EF << 1)^polynomial; //XOR i leftshift
			}else { //jesli = 0
				crcSumOnMCU_EF = (crcSumOnMCU_EF << 1); //leftshift
			}
			if(data & xbit){
				crcSumOnMCU_EF = crcSumOnMCU_EF ^polynomial; //XOR
			}
			xbit >>= 1;			//right shift o 1
		}
  	}
  	return crcSumOnMCU_EF;

}
/**
 * crc_calc(void)
 * Funkcja obliczajaca sume kontrolna dla TX
 */

uint8_t crc_calc_TX(void)
{

  	uint8_t crcSumOnMCUTX_EF = 0xFF;
  	uint8_t xbit, data1 =1;
	#define polynomial 0x7;

  	for(uint8_t l = 0; l < TX_FRAME_LENGHT_EF-1; l++){
  	uint8_t data = dataToTx_EF[l];
  	xbit = data1 << 7;
  	for(uint8_t k = sizeof(TX_FRAME_LENGHT_EF-1)*8; k > 0;--k) // obliczanie wartosci najbardziej znaczacego bitu
  	{
  		if(crcSumOnMCUTX_EF & 0x80)    //jesli najbardziej znaczacy bit = 1
  		{
  			crcSumOnMCUTX_EF = (crcSumOnMCUTX_EF << 1)^polynomial; //XOR i leftshift
  		}else { //jesli = 0
  			crcSumOnMCUTX_EF = (crcSumOnMCUTX_EF << 1); //leftshift
  		}
  		if(data & xbit){
  			crcSumOnMCUTX_EF = crcSumOnMCUTX_EF ^polynomial;
  		}
  		xbit >>= 1;
  	}
  	}
  	return crcSumOnMCUTX_EF;

}
/**
* @fn rs485_init_SW(void)
* @brief Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void) dla kierownicy
*/
void rs485_init_EF(void)
{
  HAL_UART_Receive_DMA(&UART_PORT_RS485_EF, &RS485_BUFF_EF.rx, 1);				//Rozpocznij nasluchiwanie
  prepareNewDataToSend_EF();								//Przygotuj nowy pakiet danych
}

/**
* @fn rs485_step(void)
* @brief Funkcje obslugujace magistrale, umiescic wewnatrz hydrogreen_step(void) dla kierownicy i przeplywu energii
*/
void rs485_step_EF(void)
{

	if(  endofRX_SW == 1 ){
    receiveData_EF();
	}

   sendData_EF();

}

/**
* @fn sendData(void)
* @brief Funkcja ktorej zadaniem jest obsluga linii TX, powinna zostac umieszczona w wewnatrz rs485_step() dla przeplywu energii
*/
static void sendData_EF(void)
{

  static uint16_t cntEndOfTxTick_EF;							//Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

  //Sprawdz czy wyslano cala ramke danych
  if (posInTxTab_EF < TX_FRAME_LENGHT_EF)
    {
      //Nie, wysylaj dalej
      RS485_BUFF_EF.tx = dataToTx_EF[posInTxTab_EF];

//wykonywac tyle razy ile ma dlugosc czyli 6 na raz, bez przeskiwania stma dalej

      //Na czas wysylania danych wylacz przerwania
      __disable_irq();
      HAL_UART_Transmit(&UART_PORT_RS485_EF, &RS485_BUFF_EF.tx, 1, HAL_MAX_DELAY);
      __enable_irq();

      posInTxTab_EF++;
    }
  else if (cntEndOfTxTick_EF < TX_FRAME_LENGHT_EF)
    {
      //Cala ramka danych zostala wyslana, zacznij odliczac "czas przerwy" pomiedzy przeslaniem kolejnej ramki
      cntEndOfTxTick_EF++;
      endofTX_EF=1;
    }
  else
    {
      //Przygotuj nowe dane do wysylki
      cntEndOfTxTick_EF = 0;
      posInTxTab_EF = 0;

      prepareNewDataToSend_EF();
    }
}

/**
* @fn receiveData(void)
* @brief Funkcja ktorej zadaniem jest obsluga linii RX, umiescic wewnatrz rs485_step() przeplywu energii
*/
static void receiveData_EF(void)
{

 // static uint32_t rejectedFramesInRow_EF;							//Zmienna przechowujaca liczbe straconych ramek z rzedu
  static uint32_t cntEndOfRxTick_EF;							//Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

  //Sprawdz czy otrzymano nowe dane
  if (!intRxCplt_EF)
    {
      //Nie otrzymano nowych danych, zacznij odliczac czas
      cntEndOfRxTick_EF++;
    }
  else if (intRxCplt_EF)
    {
      //Nowe dane zostaly otrzymane, zeruj flage informujaca o zakonczeniu transmisji
      intRxCplt_EF = 0;
    }

  //Sprawdz czy minal juz czas wynoszacy RX_FRAME_LENGHT
  if (cntEndOfRxTick_EF > RX_FRAME_LENGHT_EF)
    {
      //Na czas przetwarzania danych wylacz przerwania
      __disable_irq();

      //Czas minal, oznacza to koniec ramki
      cntEndOfRxTick_EF = 0;
      posInRxTab_EF = 0;

      endofRX_EF = 1;
      endofRX_SW = 0;

      //OBLICZ SUME KONTROLNA

      //Sprawdz czy sumy kontrolne oraz bajt EOT (End Of Tranmission) sie zgadzaja
      if ( (dataFromRx_EF[RX_FRAME_LENGHT_EF - 2] == EOT_BYTE_EF) && (crc_calc() == dataFromRx_EF[RX_FRAME_LENGHT_EF - 1]) )
	{

	  processReceivedData_EF();
	  rs485_flt_EF = RS485_FLT_NONE_EF;
	  rejectedFramesInRow_EF = 0;
	}
      else
	{
    	//processReceivedData_EF();
	    rejectedFramesInRow_EF++;

	  //Jezeli odrzucono wiecej niz 50 ramek z rzedu uznaj ze tranmisja zostala zerwana
	  if (rejectedFramesInRow_EF > 100)
	    {
	      resetActData_EF();
	      rs485_flt_EF = RS485_NEW_DATA_TIMEOUT_EF;
		//	RS485_TX_DATA_SW.emergencyButton = 1;
			RS485_TX_DATA_EF.emergencyScenario = 1;
			HAL_GPIO_WritePin(GPIOC, Solenoid_Valve_GPIO_Pin, GPIO_PIN_RESET);
		///	HAL_GPIO_WritePin(GPIOB, Emergency_Relay_GPIO_Pin, GPIO_PIN_RESET);
			RS485_TX_DATA_EF.motorPWM = 0;
			RS485_TX_DATA_SW.motorPWM = 0;
	    }
	}

      //Wyczysc bufor odbiorczy
      for (uint8_t i = 0; i < RX_FRAME_LENGHT_EF; i++)
	{
	  dataFromRx_EF[i] = 0x00;
	}

      __enable_irq();
    }
}

void HAL_UART_RxCpltCallback();

/**
* @fn prepareNewDataToSend(void)
* @brief Funkcja przygotowujaca dane do wysylki, wykorzystana wewnatrz sendData(void) dla przeplywu energii
*/
static void prepareNewDataToSend_EF(void)
{

  uint8_t j = 0;

  dataToTx_EF[j] = RS485_TX_DATA_EF.fuellCellModeButtons;
  dataToTx_EF[++j] = RS485_RX_VERIFIED_DATA_SW.scClose;
  dataToTx_EF[++j] = RS485_TX_DATA_EF.emergencyScenario;
  dataToTx_EF[++j] = RS485_TX_DATA_EF.motorPWM;


  dataToTx_EF[++j] = EOT_BYTE_EF;

  //OBLICZ SUME KONTROLNA

  //Wrzuc obliczona sume kontrolna na koniec wysylanej tablicy
  dataToTx_EF[TX_FRAME_LENGHT_EF - 1] = crc_calc_TX();
}


/**
* @fn processReveivedData_EF()
* @brief Funkcja przypisujaca odebrane dane do zmiennych docelowych dla przeplywu energii
*/
static void processReceivedData_EF(void)
{
  uint8_t i = 0;


  RS485_RX_VERIFIED_DATA_EF.emergency = dataFromRx_EF[i];

  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_EF.FC_V.array[k] = dataFromRx_EF[++i];
    }

  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_EF.FC_TEMP.array[k] = dataFromRx_EF[++i];
    }
  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_EF.CURRENT_SENSOR_FC_TO_SC.array[k] = dataFromRx_EF[++i];
    }

  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_EF.SC_V.array[k] = dataFromRx_EF[++i];
    }

  for (uint8_t k = 0; k < 2; k++)
    {
      RS485_RX_VERIFIED_DATA_EF.fcFanRPM.array[k] = dataFromRx_EF[++i];
    }
}
/**
* @fn resetActData_EF
* @brief Zerowanie zmiennych docelowych (odbywa sie m.in w przypadku zerwania transmisji) dla przeplywu energii
*/
static void resetActData_EF(void)
{
	RS485_RX_VERIFIED_DATA_EF.emergency = 0;

  for (uint8_t k = 0; k < 4; k++)
    {
	  RS485_RX_VERIFIED_DATA_EF.FC_V.array[k] = 0;
    }

  for (uint8_t k = 0; k < 4; k++)
    {
	  RS485_RX_VERIFIED_DATA_EF.FC_TEMP.array[k] = 0;
    }

  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_EF.CURRENT_SENSOR_FC_TO_SC.array[k] = 0;
    }

  for (uint8_t k = 0; k < 4; k++)
    {
      RS485_RX_VERIFIED_DATA_EF.SC_V.array[k] = 0;
    }
  RS485_RX_VERIFIED_DATA_EF.fcFanRPM.value = 0;
}
