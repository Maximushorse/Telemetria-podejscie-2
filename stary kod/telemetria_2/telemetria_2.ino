#include "FS.h"
#include "SD.h"
#include "SPI.h"


void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}



void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading"); 
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}
float numb;
void setup(){
  
  numb=50.5;
    Serial.begin(115200);
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }


    createDir(SD, "/mydir");    
   // removeDir(SD, "/mydir");


    
    //float write
    File file = SD.open("/hello.txt", FILE_WRITE);
    file.print(numb);
    file.print("\n");
    file.close();


    
    

    testFileIO(SD, "/test.txt");
    //Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    //Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

void loop(){
  numb++;
File file = SD.open("/hello.txt", FILE_WRITE);
    file.print(numb);
    file.print("\n");
    file.close();
     readFile(SD, "/hello.txt");
}










//Kod odbiorczy i konwersyjny

#include "rs485DL.h"
#include "rxinterrupt.h"
#include "rs485SW.h"
#include "rs485EF.h"

#define RX_FRAME_LENGHT_DL    41          ///< Dlugosc otrzymywanej ramki danych (z suma CRC)
#define EOT_BYTE_DL         0x17        ///< Bajt wskazujacy na koniec ramki

//Zmienne dla transmisji danych z przeplywem energii
uint8_t dataFromRx_DL[RX_FRAME_LENGHT_DL];        ///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
uint16_t posInRxTab_DL=0;                 ///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
volatile static uint8_t intRxCplt_DL;                   ///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)
static uint8_t dataToTx_DL[TX_FRAME_LENGHT_DL];             ///< Tablica w ktorej zawarta jest ramka danych do wyslania
static uint16_t posInTxTab_DL=0;                      ///< Aktualna pozycja w tabeli wykorzystywanej do wysylania danych
uint8_t rs485_flt_DL = RS485_NEW_DATA_TIMEOUT_DL;           ///< Zmienna przechowujaca aktualny kod bledu magistrali
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
    uint8_t crcSumOnMCU_DL = 0xFF; // 0x 255 hex 
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
* @fn rs485_init_DL(void)
* @brief Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void) dla kierownicy
*/
void rs485_init_DL(void)
{
  //HAL_UART_Receive_DMA(&UART_PORT_RS485_DL, &RS485_BUFF_DL.rx, 1); //----------- Funkcja inicjalizująca UART w ESP32
  prepareNewDataToSend_DL();                //Przygotuj nowy pakiet danych
}

/**
* @fn rs485_step(void)
* @brief Funkcje obslugujace magistrale, umiescic wewnatrz hydrogreen_step(void) dla kierownicy i przeplywu energii
*/
void rs485_step_DL(void)
{

  if( endofRX_DL == 0 && endofRX_EF == 1 ){
   receiveData_DL();
  }

}


/**
* @fn receiveData(void)
* @brief Funkcja ktorej zadaniem jest obsluga linii RX, umiescic wewnatrz rs485_step() przeplywu energii
*/
static void receiveData_DL(void)
{
 // static uint32_t rejectedFramesInRow_EF;             //Zmienna przechowujaca liczbe straconych ramek z rzedu
  static uint32_t cntEndOfRxTick_DL;              //Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

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
*** @fn processReveivedData_EF()
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