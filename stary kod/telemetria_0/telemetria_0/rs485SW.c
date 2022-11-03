/**
* @file rs485.c
* @brief Biblioteka do obslugi komunikacji UART <-> RS485 <-> UART dla kierownicy
* @author Piotr Durakiewicz
* @date 08.12.2020
* @todo
* @bug
* @copyright 2020 HYDROGREEN TEAM
*/

#include "rs485SW.h"
#include "usart.h"
#include "rxinterrupt.h"
#include "rs485EF.h"
#include "rs485DL.h"

// ******************************************************************************************************************************************************** //

#define UART_PORT_RS485_SW 		huart3
#define TX_FRAME_LENGHT_SW 		39					///< Dlugosc wysylanej ramki danych (z suma CRC)
#define RX_FRAME_LENGHT_SW 		11					///< Dlugosc otrzymywanej ramki danych (z suma CRC)
#define EOT_BYTE_SW			    0x17			    ///< Bajt wskazujacy na koniec ramki

// ******************************************************************************************************************************************************** //

uint8_t dataFromRx_SW[RX_FRAME_LENGHT_SW]; 				///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
uint16_t posInRxTab_SW=0;									///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
volatile static uint8_t intRxCplt_SW; 									///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)
static uint8_t dataToTx_SW[TX_FRAME_LENGHT_SW]; 						///< Tablica w ktorej zawarta jest ramka danych do wyslania
static uint16_t posInTxTab_SW=0;											///< Aktualna pozycja w tabeli wykorzystywanej do wysylania danych
uint8_t rs485_flt_SW = RS485_NEW_DATA_TIMEOUT_SW;						///< Zmienna przechowujaca aktualny kod bledu magistrali

// ******************************************************************************************************************************************************** //

/**
* @struct RS485_BUFFER
* @brief Struktura zawierajaca bufory wykorzystywane do transmisji danych
*/


RS485_BUFFER_SW RS485_BUFF_SW;


RS485_NEW_DATA_SW RS485_TX_DATA_SW; 				///< Struktura w ktorej zawarte sa SPRAWDZONE przychodzace dane
RS485_RECEIVED_VERIFIED_DATA_SW RS485_RX_VERIFIED_DATA_SW;
// ******************************************************************************************************************************************************** //

static void sendData_SW(void);
static void receiveData_SW(void);
static void prepareNewDataToSend_SW(void);
static void processReceivedData_SW(void);
static void resetActData_SW(void);

// ******************************************************************************************************************************************************** //
uint8_t endofRX_SW=0;


/**
 * crc_calc(void)
 * Funkcja obliczajaca sume kontrolna dla RX
 */
uint8_t crc_calc_SW(void)
{
  	uint8_t crcSumOnMCU_SW = 0xFF;
  	uint8_t xbit, data1 =1;
	#define polynomial 0x7;

  	for(uint8_t l = 0; l < RX_FRAME_LENGHT_SW-1; l++){
  	uint8_t data = dataFromRx_SW[l];
  	xbit = data1 << 7;
  	for(uint8_t k = sizeof(RX_FRAME_LENGHT_SW-1)*8; k > 0;--k) // obliczanie wartosci najbardziej znaczacego bitu
  	{
  		if(crcSumOnMCU_SW & 0x80)    //jesli najbardziej znaczacy bit = 1
  		{
  			crcSumOnMCU_SW = (crcSumOnMCU_SW << 1)^polynomial; //XOR i leftshift
  		}else { //jesli = 0
  			crcSumOnMCU_SW = (crcSumOnMCU_SW << 1); //leftshift
  		}
  		if(data & xbit){
  			crcSumOnMCU_SW = crcSumOnMCU_SW ^polynomial;
  		}
  		xbit >>= 1;
  	}
  	}
  	return crcSumOnMCU_SW;

}
/**
 * crc_calc(void)
 * Funkcja obliczajaca sume kontrolna dla TX
 */
uint8_t crc_calc_TX_SW(void)
{
  	uint8_t crcSumOnMCUTX_SW = 0xFF;
  	uint8_t xbit, data1 =1;
	#define polynomial 0x7;

  	for(uint8_t l = 0; l < TX_FRAME_LENGHT_SW-1; l++){
  	uint8_t data = dataToTx_SW[l];
  	xbit = data1 << 7;
  	for(uint8_t k = sizeof(TX_FRAME_LENGHT_SW-1)*8; k > 0;--k) // obliczanie wartosci najbardziej znaczacego bitu
  	{
  		if(crcSumOnMCUTX_SW & 0x80)    //jesli najbardziej znaczacy bit = 1
  		{
  			crcSumOnMCUTX_SW = (crcSumOnMCUTX_SW << 1)^polynomial; //XOR i leftshift
  		}else { //jesli = 0
  			crcSumOnMCUTX_SW = (crcSumOnMCUTX_SW << 1); //leftshift
  		}
  		if(data & xbit){
  			crcSumOnMCUTX_SW = crcSumOnMCUTX_SW ^polynomial;
  		}
  		xbit >>= 1;
  	}
  	}
  	return crcSumOnMCUTX_SW;

}
/**
* @fn rs485_init(void)
* @brief Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void)
*/
void rs485_init_SW(void)
{
  HAL_UART_Receive_DMA(&UART_PORT_RS485_SW, &RS485_BUFF_SW.rx, 1);				//Rozpocznij nasluchiwanie
  prepareNewDataToSend_SW();								//Przygotuj nowy pakiet danych

}

/**
* @fn rs485_step(void)
* @brief Funkcja obslugujaca magistrale, umiescic wewnatrz hydrogreen_step(void)
*/
void rs485_step_SW(void)
{

	if( endofRX_SW == 0 ){
    receiveData_SW();
	}

        sendData_SW();

}

/**
* @fn sendData(void)
* @brief Funkcja ktorej zadaniem jest obsluga linii TX, powinna zostac umieszczona w wewnatrz rs485_step()
*/
static void sendData_SW(void)
{
  static uint16_t cntEndOfTxTick_SW=0;							//Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

  //Sprawdz czy wyslano cala ramke danych
  if (posInTxTab_SW < TX_FRAME_LENGHT_SW)
    {
      //Nie, wysylaj dalej
      RS485_BUFF_SW.tx = dataToTx_SW[posInTxTab_SW];

      //Na czas wysylania danych wylacz przerwania
      __disable_irq();
      HAL_UART_Transmit(&UART_PORT_RS485_SW, &RS485_BUFF_SW.tx, 1, HAL_MAX_DELAY);
      __enable_irq();

      posInTxTab_SW++;
    }
  else if (cntEndOfTxTick_SW < TX_FRAME_LENGHT_SW)
    {
      //Cala ramka danych zostala wyslana, zacznij odliczac "czas przerwy" pomiedzy przeslaniem kolejnej ramki
      cntEndOfTxTick_SW++;
    }
  else
    {
      //Przygotuj nowe dane do wysylki
      cntEndOfTxTick_SW = 0;
      posInTxTab_SW = 0;

      prepareNewDataToSend_SW();
    }
}

/**
* @fn receiveData(void)
* @brief Funkcja ktorej zadaniem jest obsluga linii RX, umiescic wewnatrz rs485_step()
*/
static void receiveData_SW(void)
{
  static uint32_t rejectedFramesInRow_SW;							//Zmienna przechowujaca liczbe straconych ramek z rzedu
  static uint32_t cntEndOfRxTick_SW;							//Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

  //Sprawdz czy otrzymano nowe dane
  if (!intRxCplt_SW)
    {
      //Nie otrzymano nowych danych, zacznij odliczac czas
      cntEndOfRxTick_SW++;
    }
  else if (intRxCplt_SW)
    {
      //Nowe dane zostaly otrzymane, zeruj flage informujaca o zakonczeniu transmisji
      intRxCplt_SW = 0;
    }

  //Sprawdz czy minal juz czas wynoszacy RX_FRAME_LENGHT
  if (cntEndOfRxTick_SW > RX_FRAME_LENGHT_SW)
    {
      //Na czas przetwarzania danych wylacz przerwania
      __disable_irq();

      //Czas minal, oznacza to koniec ramki
      cntEndOfRxTick_SW = 0;
      posInRxTab_SW = 0;
      endofRX_SW = 1;
      endofRX_EF = 0;

      //OBLICZ SUME KONTROLNA

      //Sprawdz czy sumy kontrolne oraz bajt EOT (End Of Tranmission) sie zgadzaja
      if ( (dataFromRx_SW[RX_FRAME_LENGHT_SW - 2] == EOT_BYTE_SW) && (crc_calc_SW() == dataFromRx_SW[RX_FRAME_LENGHT_SW - 1]) )
	{

	  processReceivedData_SW();
	  rs485_flt_SW = RS485_FLT_NONE_SW;
	  rejectedFramesInRow_SW = 0;

	}
      else
	{
	  rejectedFramesInRow_SW++;

	  //Jezeli odrzucono wiecej niz 50 ramek z rzedu uznaj ze tranmisja zostala zerwana
	  if (rejectedFramesInRow_SW > 150)
	    {
	      resetActData_SW();
	      rs485_flt_SW = RS485_NEW_DATA_TIMEOUT_SW;
		//	RS485_TX_DATA_SW.emergencyButton = 1;
			RS485_TX_DATA_EF.emergencyScenario = 1;
			HAL_GPIO_WritePin(GPIOC, Solenoid_Valve_GPIO_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Emergency_Relay_GPIO_Pin, GPIO_PIN_RESET);
			RS485_TX_DATA_EF.motorPWM = 0;
			RS485_TX_DATA_SW.motorPWM = 0;
	    }
	}

      //Wyczysc bufor odbiorczy
      for (uint8_t i = 0; i < RX_FRAME_LENGHT_SW; i++)
	{
	  dataFromRx_SW[i] = 0x00;
	}

      __enable_irq();
    }
}

void HAL_UART_RxCpltCallback();

/**
* @fn prepareNewDataToSend(void)
* @brief Funkcja przygotowujaca dane do wysylki, wykorzystana wewnatrz sendData(void)
*/
static void prepareNewDataToSend_SW(void)
{
  uint8_t j = 0;

  dataToTx_SW[j] = RS485_TX_DATA_SW.interimSpeed;
  dataToTx_SW[++j] = RS485_TX_DATA_SW.averageSpeed;

  for(uint8_t k = 0; k < 2; k++)
  {
	  dataToTx_SW[++j] = RS485_TX_DATA_SW.laptime_minutes.array[k];
  }
  dataToTx_SW[++j] = RS485_TX_DATA_SW.laptime_seconds;

  for(uint8_t k = 0; k < 2; k++)
  {
	  dataToTx_SW[++j] = RS485_TX_DATA_SW.laptime_miliseconds.array[k];
  }
  for(uint8_t k = 0; k < 4; k++)
  {
	  dataToTx_SW[++j] =  RS485_RX_VERIFIED_DATA_EF.FC_V.array[k];
  }
  for(uint8_t k = 0; k < 4; k++)
  {
	  dataToTx_SW[++j] = RS485_RX_VERIFIED_DATA_EF.FC_TEMP.array[k];
  }
  for(uint8_t k = 0; k < 2; k++)
  {
	  dataToTx_SW[++j] = RS485_RX_VERIFIED_DATA_EF.fcFanRPM.array[k];
  }
  for(uint8_t k = 0; k < 4; k++)
  {
	  dataToTx_SW[++j] = RS485_RX_VERIFIED_DATA_EF.CURRENT_SENSOR_FC_TO_SC.array[k];
  }
  for(uint8_t k = 0; k < 4; k++)
  {
	  dataToTx_SW[++j] = RS485_TX_DATA_SW.CURRENT_SENSOR_SC_TO_MOTOR.array[k];
  }
  dataToTx_SW[++j] = RS485_TX_DATA_SW.fcToScMosfetPWM;
  dataToTx_SW[++j] = RS485_TX_DATA_SW.motorPWM;

  for(uint8_t k = 0; k < 4; k++)
  {
	  dataToTx_SW[++j] = RS485_RX_VERIFIED_DATA_EF.SC_V.array[k];
  }
  for (uint8_t k = 0; k < 4; k++)
    {
	  dataToTx_SW[++j]=RS485_TX_DATA_SW.H2_SENSOR_V.array[k];
    }

  dataToTx_SW[++j] = RS485_TX_DATA_SW.h2SensorDigitalPin;
  dataToTx_SW[++j] = RS485_TX_DATA_SW.emergencyButton;

  dataToTx_SW[++j] = EOT_BYTE_SW;

  //OBLICZ SUME KONTROLNA

  //Wrzuc obliczona sume kontrolna na koniec wysylanej tablicy
 dataToTx_SW[TX_FRAME_LENGHT_SW - 1] = crc_calc_TX_SW();
}

/**
* @fn processReveivedData()
* @brief Funkcja przypisujaca odebrane dane do zmiennych docelowych
*/
static void processReceivedData_SW(void)
{
  uint8_t i = 0;

  RS485_RX_VERIFIED_DATA_SW.halfGas = dataFromRx_SW[i];
  RS485_RX_VERIFIED_DATA_SW.fullGas = dataFromRx_SW[++i];
  RS485_RX_VERIFIED_DATA_SW.horn = dataFromRx_SW[++i];
  RS485_RX_VERIFIED_DATA_SW.speedReset = dataFromRx_SW[++i];
  RS485_RX_VERIFIED_DATA_SW.powerSupply = dataFromRx_SW[++i];
  RS485_RX_VERIFIED_DATA_SW.scClose = dataFromRx_SW[++i];
  RS485_RX_VERIFIED_DATA_SW.fuelcellOff = dataFromRx_SW[++i];
  RS485_RX_VERIFIED_DATA_SW.fuelcellPrepareToRace = dataFromRx_SW[++i];
  RS485_RX_VERIFIED_DATA_SW.fuelcellRace = dataFromRx_SW[++i];

  //stany przyciskow wysylane do EF
  if(  RS485_RX_VERIFIED_DATA_SW.fuelcellOff == 1){
	  RS485_TX_DATA_EF.fuellCellModeButtons = 1;
  } else if(  RS485_RX_VERIFIED_DATA_SW.fuelcellPrepareToRace == 1){
	  RS485_TX_DATA_EF.fuellCellModeButtons = 2;
  } else if(  RS485_RX_VERIFIED_DATA_SW.fuelcellRace ==1){
	  RS485_TX_DATA_EF.fuellCellModeButtons = 3;
  }
  if(  RS485_RX_VERIFIED_DATA_SW.powerSupply == 1){
	  RS485_TX_DATA_EF.emergencyScenario = 0;
  }else RS485_TX_DATA_EF.emergencyScenario = 1;
}

/**
* @fn resetActData
* @brief Zerowanie zmiennych docelowych (odbywa sie m.in w przypadku zerwania transmisji)
*/
static void resetActData_SW(void)
{
  RS485_RX_VERIFIED_DATA_SW.halfGas = 0;
  RS485_RX_VERIFIED_DATA_SW.fullGas = 0;
  RS485_RX_VERIFIED_DATA_SW.horn = 0;
  RS485_RX_VERIFIED_DATA_SW.speedReset = 0;
  RS485_RX_VERIFIED_DATA_SW.powerSupply = 0;
  RS485_RX_VERIFIED_DATA_SW.scClose = 0;
  RS485_RX_VERIFIED_DATA_SW.fuelcellOff = 0;
  RS485_RX_VERIFIED_DATA_SW.fuelcellPrepareToRace = 0;
  RS485_RX_VERIFIED_DATA_SW.fuelcellRace = 0;
}
