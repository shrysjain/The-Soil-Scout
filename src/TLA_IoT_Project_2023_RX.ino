/*
The Bug
August 2023
Shreyas Jain, Minati Divakar
*/

// Install dependencies
#include <SPI.h>
#include <RH_RF95.h>

// Define pin configuration
#if defined (__AVR_ATmega32U4__)  // Feather 32u4 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   7
  #define RFM95_RST   4

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   3
  #define RFM95_RST   4

#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_RFM)  // Feather RP2040 w/Radio
  #define RFM95_CS   16
  #define RFM95_INT  21
  #define RFM95_RST  17

#elif defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM95_CS    4  //
  #define RFM95_INT   3  //
  #define RFM95_RST   2  // "A"

#elif defined(ESP8266)  // ESP8266 feather w/wing
  #define RFM95_CS    2  // "E"
  #define RFM95_INT  15  // "B"
  #define RFM95_RST  16  // "D"

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
  #define RFM95_CS   10  // "B"
  #define RFM95_INT   9  // "A"
  #define RFM95_RST  11  // "C"

#elif defined(ESP32)  // ESP32 feather w/wing
  #define RFM95_CS   33  // "B"
  #define RFM95_INT  27  // "A"
  #define RFM95_RST  13

#elif defined(ARDUINO_NRF52832_FEATHER)  // nRF52832 feather w/wing
  #define RFM95_CS   11  // "B"
  #define RFM95_INT  31  // "C"
  #define RFM95_RST   7  // "A"

#endif

// Set radio frequency (915mHz)
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  // I/O pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Open serial monitor
  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Radio initialization
  while (!rf95.init()) {
    while (1);
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    while (1);
  }

  // Set transmitter power to 23 dBm
  rf95.setTxPower(23, false);
}

void loop() {
  // Create packet buffers
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(500)) {
    // Receieve packet
    if (rf95.recv((uint8_t *)buf, &len)) {
      buf[len] = '\0'; // Null-terminate the data
      int voltVal, moistVal, humVal, tempVal, hiVal;

      // Decode packet to plaintext
      sscanf((char*)buf, "%04X%04X%04X%04X%04X", &voltVal, &moistVal, &humVal, &tempVal, &hiVal);

      // Convert values back to floating point
      float voltage = voltVal / 100.0;
      float moisture = moistVal / 100.0;
      float humidity = humVal / 100.0;
      float temperature = tempVal / 100.0;
      float humidityIndex = hiVal / 100.0;

      // Print packet to the Serial Monitor (to be interpreted by Python script)
      Serial.println(String(voltage) + "|" + String(moisture) + "|" + String(humidity) + "|" + String(temperature) + "|" + String(humidityIndex));

      // Send received acknowledgement back to the transmitter
      char ack = "ACK";
      rf95.send((uint8_t *)ack, strlen(ack));
      rf95.waitPacketSent();

      // Acknowledgement sent
    } else {
      // Receive failed
    }
  }
}
