/*
The Soil Scout
August 2023
Shreyas Jain, Minati Divakar
*/

// Install dependencies
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// Define pin configuration
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4

// Set radio frequency (915mHz)
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Define DHT configuration
#define DHTTYPE     DHT11
#define DHTPIN      12

// DHT instance
DHT dht(DHTPIN, DHTTYPE);

// Soil moisture thresholds
const int dry = 851;
const int saturated = 310;
int moisturePercentage;

// Soil pH placeholders
int pH;
float Voltage;

void setup() {
  // I/O pins
  pinMode(pH, INPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Open serial monitor
  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Radio initialization
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);

  // Intialize DHT11
  dht.begin();
}

// Packet counter, incremented per xmission
int16_t packetnum = 0;

void loop() {
  // Read soil moisture
  int moisture = analogRead(A1);
  if (isnan(moisture)) {
    Serial.println(F("Error reading moisture!"));
  } else {
    Serial.print(F("Moisture: "));
    Serial.println(moisture);

    // Convert moisture to mapped percentage
    moisturePercentage = map(moisture, saturated, dry, 100, 0);
    Serial.print(F("Moisture in Percentage: "));
    Serial.print(moisturePercentage);
    Serial.println(F("%"));
  }

  // Read soil pH
  pH = analogRead(A0);
  Voltage = pH * (5.0 / 1023.0);
  Serial.print("pH Voltage: ");
  Serial.println(Voltage);

  // Read ambient temperature + humidity
  float h = dht.readHumidity();
  float t = dht.readTemperature(true);
  float hi = dht.computeHeatIndex(t, h);

  Serial.print("Ambient humidity: "); 
  Serial.println(h);
  Serial.print("Ambient temperature: "); 
  Serial.println(t);
  Serial.print("Ambient heat index: ");
  Serial.println(hi);

  // Scale measurements
  int voltScaled = Voltage * 100;
  int moistScaled = moisturePercentage * 100;
  int humScaled = h * 100;
  int tempScaled = t * 100;
  int humIndScaled = hi * 100;

  // Packet transmission
  delay(1000);
  Serial.println("Transmitting...");

  // Define radio packet
  // String packet = String(Voltage) + "|" + String(moisturePercentage) + "|" + String(h) + "|" + String(t) + "|" + String(hi);
  char radiopacket[25];
  snprintf(radiopacket, sizeof(radiopacket), "%04X%04X%04X%04X%04X", voltScaled, moistScaled, humScaled, tempScaled, humIndScaled);
  // itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  // radiopacket[19] = 0;

  // Transmit radio packet
  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, strlen(radiopacket));

  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
  
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // Await reply from receiver
  // Serial.println("Waiting for reply...");
  // if (rf95.waitAvailableTimeout(1000)) {
  //   if (rf95.recv(buf, &len)) {
  //     Serial.print("Got reply: ");
  //     Serial.println((char*)buf);
  //     Serial.print("RSSI: ");
  //     Serial.println(rf95.lastRssi(), DEC);
  //   } else {
  //     Serial.println("Receive failed");
  //   }
  // } else {
  //   Serial.println("No reply, is there a listener around?");
  // }
}
