//Obsluga polaczenia sieciowego
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>

//#include  <c_types.h>


#include "FS.h"
#include "SD.h"
#include "SPI.h"
int numb;  

const char* ssid = "Internet_Domowy_5G_60FFD7";
const char* password = "43A44434";

//Your IP address or domain name with URL path
const char* serverName = "http://hydrogreen.waw.pl/esp-outputs-action.php?action=outputs_state&board=1";

// Update interval time set to 50ms
const long interval = 50;
unsigned long previousMillis = 0;

String outputsState;


//Kod odbiorczy i konwersyjny
#define RXp2 16
#define TXp2 17

//Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);

#include <cstdint>
#include "rs485DLesp.h"
//#include "rxinterrupt.h"
//#include "rs485SW.h"
//#include "rs485EF.h"

#define RX_FRAME_LENGHT_DL    21          ///< Dlugosc otrzymywanej ramki danych (z suma CRC)
#define EOT_BYTE_DL         0x17        ///< Bajt wskazujacy na koniec ramki

//Zmienne dla transmisji danych z przeplywem energii
uint8_t dataFromRx_DL[RX_FRAME_LENGHT_DL];        ///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
uint16_t posInRxTab_DL=0;                 ///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
// uint8_t intRxCplt_DL;                   ///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)
// uint8_t dataToTx_DL[TX_FRAME_LENGHT_DL];             ///< Tablica w ktorej zawarta jest ramka danych do wyslania
// uint16_t posInTxTab_DL=0;                      ///< Aktualna pozycja w tabeli wykorzystywanej do wysylania danych
uint8_t rs485_flt_DL = RS485_NEW_DATA_TIMEOUT_DL;           ///< Zmienna przechowujaca aktualny kod bledu magistrali
uint32_t rejectedFramesInRow_DL=0;
// ******************************************************************************************************************************************************** //


 //Funkcja obliczajaca sume kontrolna dla RX
 

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
* @brief //Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void) dla kierownicy
*/
void rs485_init_DL(void)
{
  //HAL_UART_Receive_DMA(&UART_PORT_RS485_DL, &RS485_BUFF_DL.rx, 1); //----------- Funkcja inicjalizująca UART w ESP32 ############
  
   Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);
  //prepareNewDataToSend_DL();                //Przygotuj nowy pakiet danych
}
int endofRX_DL;
/**
* @fn rs485_step(void)
* @brief Funkcje obslugujace magistrale, umiescic wewnatrz hydrogreen_step(void) dla kierownicy i przeplywu energii
*/
void rs485_step_DL(void)
{

  if( endofRX_DL == 0 ){
   receiveData_DL();
  }

}


/**
* @fn receiveData(void)
* @brief Funkcja ktorej zadaniem jest obsluga linii RX, umiescic wewnatrz rs485_step() przeplywu energii
*/
 void receiveData_DL(void)
{
 //  uint32_t rejectedFramesInRow_EF;             //Zmienna przechowujaca liczbe straconych ramek z rzedu
   uint32_t cntEndOfRxTick_DL;              //Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

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
      //__disable_irq();

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
       Serial.println(RS485_RX_VERIFIED_DATA_DL.FCC_V.value);
   // Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.FCC_TMP.value);
    Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.CURRENT_SENSOR_FC_TO_SCC.value);
    Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.SCC_V.value);
    Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.fcFanRPMC.value);
//    Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.emergency);  
    numb+=RS485_RX_VERIFIED_DATA_DL.FCC_V.value;
    //appendFile(SD, "/hello.txt", numb);
//    file.print(numb);
  //  file.print("\n");
    //file.close();
     //readFile(SD, "/hello.txt");
  }
      else
  {
      //processReceivedData_EF();
      rejectedFramesInRow_DL++;
/*
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
      } */
  }

      //Wyczysc bufor odbiorczy
      for (uint8_t i = 0; i < RX_FRAME_LENGHT_DL; i++)
  {
    dataFromRx_DL[i] = 0x00;
  }

      //__enable_irq(); //Po odebraniu danych wlacz przerwania
    }
}

void HAL_UART_RxCpltCallback();

/**
*** @fn processReveivedData_EF()
* @brief Funkcja przypisujaca odebrane dane do zmiennych docelowych dla przeplywu energii
*/
 void processReceivedData_DL(void)
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
/*                //Telemetria powinna zapisywać dane niezaleznie od tego czy dotarla calosc czy cos przerwalo
 void resetActData_DL(void)
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
} */


void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  Serial.println("Hydrogreen Team 2022"); Serial.println(" "); Serial.println(" ");
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  sd_setup();
  rs485_init_DL();
}

void loop() {
  unsigned long currentMillis = millis();

  //Automatyczne
  if(currentMillis - previousMillis >= interval) {
     // Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED ){ 
      outputsState = httpGETRequest(serverName);
      Serial.println(outputsState);
      JSONVar myObject = JSON.parse(outputsState);
  
      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
    
      Serial.print("JSON object = ");
      Serial.println(myObject);
    
      // myObject.keys() can be used to get an array of all the keys in the object
      JSONVar keys = myObject.keys();
    
      for (int i = 0; i < keys.length(); i++) {
        JSONVar value = myObject[keys[i]];
        Serial.print("GPIO: ");
        Serial.print(keys[i]);
        Serial.print(" - SET to: ");
        Serial.println(value);
        pinMode(atoi(keys[i]), OUTPUT);
        digitalWrite(atoi(keys[i]), atoi(value));
      }
      // save the last HTTP GET Request
      previousMillis = currentMillis;
    }
    else {
      Serial.println("WiFi Disconnected");
    }
  }
}

String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
    
  // Your IP address with path or Domain name with URL path 
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
receiveData_DL();
  save();
  
}







//Obsluga kart microSD


/*
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
     uint8_t buf[512];
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


void sd_control(){

  numb=50.5;
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


//    createDir(SD, "/mydir");    
   // removeDir(SD, "/mydir");


    
    //float write
    File file = SD.open("/hello.txt", FILE_WRITE);
    file.print(numb);
    file.print("\n");
    file.close();


    
    

    //testFileIO(SD, "/test.txt");
    //Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    //Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
} */




  //Docelowa wersja obsługi kart sd
void sd_setup(){
  long time_old=millis();
  long time_oldd = millis();
  int min, sec, time;
  string name = "dane_";
}

void save()
{
   time_save();
   name =+ min;
   name =+ ".txt"
   myFile = SD.open(name, FILE_WRITE);
  if (myFile) {    
    myFile.print(time);
    myFile.print(",");    
    myFile.println(int(RS485_RX_VERIFIED_DATA_DL.emergencyC));
    myFile.print(",");
    myFile.println(float(RS485_RX_VERIFIED_DATA_DL.FCC_V));
    myFile.print(",");
    myFile.println(float(RS485_RX_VERIFIED_DATA_DL.FCC_TEMP));
    myFile.print(",");
    myFile.println(float(RS485_RX_VERIFIED_DATA_DL.CURRENT_SENSOR_FC_TO_SCC));
    myFile.close(); // close the file
  }
}

//Kontrola czasu synchronizowanego z serwerem i dopisywanego do nazwy i pomiarów w pliku dane
 void time_save()
{
 //test
 if (millis() - time_oldd >=1000) {
   sec++;
   time_oldd=millis;
   }
   
 if(millis() - time_old >= 6000) {
   time_old = millis();
   min++;
  }

 time=0;
 time=min*100+sec;

}














//Kod odbiorczy i konwersyjny
#define RXp2 16
#define TXp2 17

//Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);

#include <cstdint>
#include "rs485DLesp.h"
//#include "rxinterrupt.h"
//#include "rs485SW.h"
//#include "rs485EF.h"

#define RX_FRAME_LENGHT_DL    21          ///< Dlugosc otrzymywanej ramki danych (z suma CRC)
#define EOT_BYTE_DL         0x17        ///< Bajt wskazujacy na koniec ramki

//Zmienne dla transmisji danych z przeplywem energii
uint8_t dataFromRx_DL[RX_FRAME_LENGHT_DL];        ///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
uint16_t posInRxTab_DL=0;                 ///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
// uint8_t intRxCplt_DL;                   ///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)
// uint8_t dataToTx_DL[TX_FRAME_LENGHT_DL];             ///< Tablica w ktorej zawarta jest ramka danych do wyslania
// uint16_t posInTxTab_DL=0;                      ///< Aktualna pozycja w tabeli wykorzystywanej do wysylania danych
uint8_t rs485_flt_DL = RS485_NEW_DATA_TIMEOUT_DL;           ///< Zmienna przechowujaca aktualny kod bledu magistrali
uint32_t rejectedFramesInRow_DL=0;
// ******************************************************************************************************************************************************** //


 //Funkcja obliczajaca sume kontrolna dla RX
 

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
* @brief //Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void) dla kierownicy
*/
void rs485_init_DL(void)
{
  //HAL_UART_Receive_DMA(&UART_PORT_RS485_DL, &RS485_BUFF_DL.rx, 1); //----------- Funkcja inicjalizująca UART w ESP32 ############
  
   Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);
  //prepareNewDataToSend_DL();                //Przygotuj nowy pakiet danych
}
int endofRX_DL;
/**
* @fn rs485_step(void)
* @brief Funkcje obslugujace magistrale, umiescic wewnatrz hydrogreen_step(void) dla kierownicy i przeplywu energii
*/
void rs485_step_DL(void)
{

  if( endofRX_DL == 0 ){
   receiveData_DL();
  }

}


/**
* @fn receiveData(void)
* @brief Funkcja ktorej zadaniem jest obsluga linii RX, umiescic wewnatrz rs485_step() przeplywu energii
*/
 void receiveData_DL(void)
{
 //  uint32_t rejectedFramesInRow_EF;             //Zmienna przechowujaca liczbe straconych ramek z rzedu
   uint32_t cntEndOfRxTick_DL;              //Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

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
      //__disable_irq();

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
       Serial.println(RS485_RX_VERIFIED_DATA_DL.FCC_V.value);
   // Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.FCC_TMP.value);
    Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.CURRENT_SENSOR_FC_TO_SCC.value);
    Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.SCC_V.value);
    Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.fcFanRPMC.value);
//    Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.emergency);  
    numb+=RS485_RX_VERIFIED_DATA_DL.FCC_V.value;
    //appendFile(SD, "/hello.txt", numb);
//    file.print(numb);
  //  file.print("\n");
    //file.close();
     //readFile(SD, "/hello.txt");
  }
      else
  {
      //processReceivedData_EF();
      rejectedFramesInRow_DL++;
/*
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
      } */
  }

      //Wyczysc bufor odbiorczy
      for (uint8_t i = 0; i < RX_FRAME_LENGHT_DL; i++)
  {
    dataFromRx_DL[i] = 0x00;
  }

      //__enable_irq(); //Po odebraniu danych wlacz przerwania
    }
}

void HAL_UART_RxCpltCallback();

/**
*** @fn processReveivedData_EF()
* @brief Funkcja przypisujaca odebrane dane do zmiennych docelowych dla przeplywu energii
*/
 void processReceivedData_DL(void)
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
/*                //Telemetria powinna zapisywać dane niezaleznie od tego czy dotarla calosc czy cos przerwalo
 void resetActData_DL(void)
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
} */
