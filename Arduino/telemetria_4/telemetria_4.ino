//Obsluga polaczenia sieciowego
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>

//#include  <c_types.h>


#include "FS.h"
#include "SD.h"
#include "SPI.h"
int numb;

const char* ssid = "UPC7976702";
const char* password = "6ykfBfkmcy8z";

//Your IP address or domain name with URL path
const char* serverName = "192.168.0.171";  //"http://hydrogreen.waw.pl/esp-outputs-action.php?action=outputs_state&board=1";

// Update interval sd_time set to 50ms
const long interval = 50;
unsigned long previousMillis = 0;


File myFile;

long sd_time_old;
long sd_time_oldd;
int sd_min=0;
int sd_sec=0;
int sd_time;
String name = "dane_";

String outputsState;



//Kod odbiorczy i konwersyjny
#define RXp2 16
#define TXp2 17

//Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);

#include <cstdint>
#include <stdint-gcc.h>
///#include "rs485DLesp.h"

#define EOT_BYTE_DL 0x17       ///< Bajt wskazujacy na koniec ramki

#define RX_FRAME_LENGHT_DL 21

// ******************************************************************************************************************************************************** //

#define RS485_FLT_NONE_DL 0x00					///< Brak bledu
#define RS485_NEW_DATA_sd_timeOUT_DL 0x11				///< Nie otrzymano nowych dane (polaczenie zostalo zerwane)

extern uint8_t rs485_flt_DL; 					///< Zmienna przechowujaca aktualny kod bledu magistrali

extern uint8_t dataFromRx_DL[RX_FRAME_LENGHT_DL]; 				///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
extern uint16_t posInRxTab_DL;		///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
extern uint8_t crcSumOnMCU_DL;
//volatile static 
uint8_t intRxCplt_DL; 									///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)
uint8_t crc_calc_TX_DL(void);
uint8_t crc_calc_DL(void);


//Zmienne dla transmisji danych z przeplywem energii
uint8_t dataFromRx_DL[RX_FRAME_LENGHT_DL];  ///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
uint16_t posInRxTab_DL = 0;                 ///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
uint8_t rs485_flt_DL = RS485_NEW_DATA_sd_timeOUT_DL;  ///< Zmienna przechowujaca aktualny kod bledu magistrali
uint32_t rejectedFramesInRow_DL = 0;

uint8_t cntEndOfRxTick_DL = 0;
//uint32_t intRxCplt_DL = 0;

//static void sendData_DL(void);
static void receiveData_DL(void);
//static void prepareNewDataToSend_DL(void);
static void processReceivedData_DL(void);
//static void resetActData_DL(void);
uint32_t RS485_RX_VERIFIED_DATA_DL = 0;

//uint32_t RS485_RX_VERIFIED_DATA_DL;

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


// ******************************************************************************************************************************************************** //


void setup() {



  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.println("Hydrogreen Team 2022");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("Connecting");
  Serial.println("Connecting to ");
  Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  rs485_init_DL();


}




//Funkcja obliczajaca sume kontrolna dla RX


uint8_t crc_calc_DL() {
  uint8_t crcSumOnMCU_DL = 0xFF;  // 0x 255 hex
  uint8_t xbit, data1 = 1;
#define polynomial 0x7;

  for (uint8_t l = 0; l < RX_FRAME_LENGHT_DL - 1; l++) {
    uint8_t data = dataFromRx_DL[l];
    xbit = data1 << 7;
    for (uint8_t k = sizeof(RX_FRAME_LENGHT_DL - 1) * 8; k > 0; --k)  // obliczanie wartosci najbardziej znaczacego bitu
    {
      if (crcSumOnMCU_DL & 0x80)  //jesli najbardziej znaczacy bit = 1
      {
        crcSumOnMCU_DL = (crcSumOnMCU_DL << 1) ^ polynomial;  //XOR i leftshift
      } else {                                                //jesli = 0
        crcSumOnMCU_DL = (crcSumOnMCU_DL << 1);               //leftshift
      }
      if (data & xbit) {
        crcSumOnMCU_DL = crcSumOnMCU_DL ^ polynomial;
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
void rs485_init_DL() {
  //HAL_UART_Receive_DMA(&UART_PORT_RS485_DL, &RS485_BUFF_DL.rx, 1); //----------- Funkcja inicjalizująca UART w ESP32 ############

  Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);
  //prepareNewDataToSend_DL();                //Przygotuj nowy pakiet danych
}
int endofRX_DL;
/**
* @fn rs485_step(void)
* @brief Funkcje obslugujace magistrale, umiescic wewnatrz hydrogreen_step(void) dla kierownicy i przeplywu energii
*/
void rs485_step_DL() {

  if (endofRX_DL == 0) {
    receiveData_DL();
  }
}


/**
* @fn receiveData(void)
* @brief Funkcja ktorej zadaniem jest obsluga linii RX, umiescic wewnatrz rs485_step() przeplywu energii
*/
void receiveData_DL() {
  //  uint32_t rejectedFramesInRow_EF;             //Zmienna przechowujaca liczbe straconych ramek z rzedu
  // uint32_t cntEndOfRxTick_DL;              //Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

  //Sprawdz czy otrzymano nowe dane
  if (!intRxCplt_DL) {
    //Nie otrzymano nowych danych, zacznij odliczac czas
    cntEndOfRxTick_DL++;
  } else if (intRxCplt_DL) {
    //Nowe dane zostaly otrzymane, zeruj flage informujaca o zakonczeniu transmisji
    intRxCplt_DL = 0;
  }



  //Sprawdz czy minal juz czas wynoszacy RX_FRAME_LENGHT
  if (cntEndOfRxTick_DL > RX_FRAME_LENGHT_DL) {
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
    if ((dataFromRx_DL[RX_FRAME_LENGHT_DL - 2] == EOT_BYTE_DL) && (crc_calc_DL() == dataFromRx_DL[RX_FRAME_LENGHT_DL - 1])) {

      processReceivedData_DL();
      //rs485_flt_DL = RS485_FLT_NONE_DL;
      rejectedFramesInRow_DL = 0;
      //Serial.println(RS485_RX_VERIFIED_DATA_DL.FCC_V.value); //test zastapic 0 .
      Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.FCC_TMP.value);
      Serial.print("  ");
      Serial.print(RS485_RX_VERIFIED_DATA_DL.CURRENT_SENSOR_FC_TO_SCC.value);
      Serial.print("  ");
      Serial.print(RS485_RX_VERIFIED_DATA_DL.SCC_V.value);
      Serial.print("  ");
      Serial.print(RS485_RX_VERIFIED_DATA_DL.fcFanRPMC.value);
         Serial.print("  ");Serial.print(RS485_RX_VERIFIED_DATA_DL.emergency);
      numb += RS485_RX_VERIFIED_DATA_DL.FCC_V.value;
      
    } else {
      //processReceivedData_EF();
      rejectedFramesInRow_DL++;
      /*
    //Jezeli odrzucono wiecej niz 50 ramek z rzedu uznaj ze tranmisja zostala zerwana
    if (rejectedFramesInRow_DL > 100)
      {
        resetActData_DL();
        rs485_flt_DL = RS485_NEW_DATA_sd_timeOUT_DL;
      RS485_TX_DATA_SW.emergencyButton = 1;
      RS485_TX_DATA_EF.emergencyScenario = 1;
      HAL_GPIO_WritePin(GPIOC, Solenoid_Valve_GPIO_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, Emergency_Relay_GPIO_Pin, GPIO_PIN_RESET);
      RS485_TX_DATA_EF.motorPWM = 0;
      RS485_TX_DATA_SW.motorPWM = 0;
      } */
    }

    //Wyczysc bufor odbiorczy
    for (uint8_t i = 0; i < RX_FRAME_LENGHT_DL; i++) {
      dataFromRx_DL[i] = 0x00;
    }

    //__enable_irq(); //Po odebraniu danych wlacz przerwania
  }
}

//void HAL_UART_RxCpltCallback();


// @brief Funkcja przypisujaca odebrane dane do zmiennych docelowych dla przeplywu energii



void processReceivedData_DL() {
  uint8_t i = 0;

 RS485_RECEIVED_VERIFIED_DATA_DL RS485_RX_VERIFIED_DATA_DL = {1};
  
  RS485_RX_VERIFIED_DATA_DL.emergencyC = dataFromRx_DL[i];
  for (uint8_t k = 0; k < 4; k++) {
    RS485_RX_VERIFIED_DATA_DL.FCC_V.array[k] = dataFromRx_DL[++i];
  }
  for (uint8_t k = 0; k < 4; k++) {
    RS485_RX_VERIFIED_DATA_DL.FCC_TEMP.array[k] = dataFromRx_DL[++i];
  }
  for (uint8_t k = 0; k < 4; k++) {
    RS485_RX_VERIFIED_DATA_DL.CURRENT_SENSOR_FC_TO_SCC.array[k] = dataFromRx_DL[++i];
  }

  for (uint8_t k = 0; k < 4; k++) {
    RS485_RX_VERIFIED_DATA_DL.SCC_V.array[k] = dataFromRx_DL[++i];
  }

  for (uint8_t k = 0; k < 2; k++) {
    RS485_RX_VERIFIED_DATA_DL.fcFanRPMC.array[k] = dataFromRx_DL[++i];
  } 
}




void loop() {
  unsigned long currentMillis = millis();

  //Automatyczne
  if (currentMillis - previousMillis >= interval) {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED) {
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
    } else {
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

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;


 rs485_step_DL();
 


 //Zapis na karte sd

 if(!SD.begin()){
        Serial.println("Card Mount Failed");
    }




if (millis() - sd_time_oldd >= 1000) {
    sd_sec++;
    sd_time_oldd = millis();
  }

  if (millis() - sd_time_old >= 6000) {
    sd_time_old = millis();
    sd_min=+1;
  }

  sd_time = 0;
  sd_time =sd_min * 100 + sd_sec;

  name = name + sd_min;
  name = +".txt" ;
  myFile = SD.open(name, FILE_WRITE);

  if (myFile) {

  //Serial.println(FCC_V.value);

    myFile.print(sd_time);
    myFile.print(",");
    myFile.println(int(RS485_RX_VERIFIED_DATA_DL.emergencyC));
    myFile.print(",");
    myFile.println(float(RS485_RX_VERIFIED_DATA_DL.FCC_V));
    myFile.print(",");
    myFile.println(float(RRS485_RX_VERIFIED_DATA_DL.FCC_TEMP));
    myFile.print(",");
    myFile.println(float(RS485_RX_VERIFIED_DATA_DL.CURRENT_SENSOR_FC_TO_SCC));
    myFile.close();  // close the file
  }


  //save();

  //sd_control();
}









