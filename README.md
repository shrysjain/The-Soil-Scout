# The Soil Scout - TLA IoT BYOD 2023 (Advanced Project) ![Arduino](https://img.shields.io/badge/-Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white) ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

<img src="./logo.png" align="right" alt="Logo" width="300" height="280">

The Soil Scout is a 2-part product to help farmers and individuals grow plants, both indoors and outdoors, properly. The device measures various parameters of soil, such as pH, moisture, ambient temperature, ambient humidity, and heat index. It uses these values to calculate the health of soil, and notify users via Discord on any issues with their soil that can be detremental to plant growth, and how to fix it.

## Device Replication

Here are step-by-step instructions on how to replicate the device.

### System Requirements
- **Operating Systems:** Windows 7 (or later), macOS, Linux
- USB Drivers
- 5 GB of RAM and 600 MB Available Disk Space
- Intel Pentium 4+
- A stable internet connection

### Installations
- Arduino IDE (Latest)
- Python (3.8+)
- Anaconda Distribution (Miniconda is untested but might work)

### Library and Board Installation
1. Install the following libraries in the Arduino IDE:
- [RadioHead](https://www.arduino.cc/reference/en/libraries/radiohead/)
- [DHT Sensor Library](https://www.arduino.cc/reference/en/libraries/dht-sensor-library/)

### Running the code

1. In the Arduino IDE, create a new sketch and paste the following code in: 
<details><summary>Expand to see code</summary>
<br>

```cpp
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
```
</details>

2. On another board, run the following sketch: 
<details><summary>Expand to see code</summary>
<br>

```cpp
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
```
</details>

The code can also be downloaded from within this GitHub repository in the `src/` folder.

Upload both samples to two RFM95 radio boards.

Once it is running, open the Anaconda Prompt and one by one, run the following lines. Once you run a line, wait for it to finish before running the next. These are the lines to run:

```cmd
conda create -y -n arduino python=3.8

conda activate arduino

conda install pyserial

pip install discord-webhook

python eventHandler.py
```

Before running the final line, make sure you are in the same directory as `eventHandler.py`. You can find this file in the `src/` folder, or below:

<details><summary>Expand to see code</summary>

```py
"""
Event Handler
August 2023
Shreyas Jain, Minati Divakar
"""

# Install dependencies
import code
from xml.etree.ElementTree import tostring
import serial
import time

# Open serial monitor
ser = serial.Serial('COM5', 115200)
time.sleep(2)

# Create array
values = []

# Test
while True:
  values = ser.readline().decode().split("|")

  try:
    values[4] = values[4][0:5]
  except:
    values = values;

  phVoltage = values[0]
  moisturePercentage = values[1]
  ambientHumidity = values[2]
  ambientTemperature = values[3]
  headIndex = values[4]

  # print(values)
```
</details>

(You may have to adjust the COM port in the code based off of what COM port your Arduino is running on)

**IMPORTANT:** You must create a webhook in a Discord channel of some server by going to `Channel Settings > Integrations > Webhooks` and create a webhook. Copy the webhook URL, and in the Python code on line 49, replace "Discord Webhook URL" with the webhook URL, which will look something similar to this URL: `https://discord.com/api/webhooks/1004349023488278548/W9ddoCTbzLq3EhHsab8ibk5ISfvj43XLm96dCmCixxw3BnXTb1VF6Q4T4CSBP6FtbD4E` 

Upon completing this, after running `python eventHandler.py` in the Anaconda Prompt, if the soil quality is poor, you will recieve notifications on Discord.

When installing `pyserial` with `conda install pyserial`, it will ask you if you want to install the packages and list them before installing. Simply type `y` on your keyboard and press enter to continue the installation.

With that, the device should be fully setup and ready to go!
