/*
* There is no specific copyright on this code as it has been
* copied from other generous developers.
* This is code that I have implemented on my lora send 
* module and is presented as a curiosity for review.
*/
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "SSD1306.h"
#include <ArduinoOTA.h> // from https://github.com/esp8266/Arduino/tree/master/libraries/ArduinoOTA
#include "Security.h"   // text file with Wifi username and password

#define SCK 5                         // GPIO5  -- SX1278's SCK
#define MISO 19                       // GPIO19 -- SX1278's MISnO
#define MOSI 27                       // GPIO27 -- SX1278's MOSI
#define SS 18                         // GPIO18 -- SX1278's CS
#define RST 14                        // GPIO14 -- SX1278's RESET
#define DIO0 26                       // GPIO26 -- SX1278's IRQ(Interrupt Request)
const unsigned long LoraBand = 915E6; // 915E6, 868E6, 433E6

// wifi
const IPAddress WiFiIP(192, 168, 0, 21);
const IPAddress WiFiGateway(192, 168, 0, 1);
const IPAddress WiFiSubnet(255, 255, 0, 0);
const IPAddress WiFiPrimaryDNS(192, 168, 0, 1); //must have
const IPAddress WiFiSecondaryDNS(8, 8, 4, 4);   //optional

// Pins
const byte WaterLevelTriggerPin = 13; // take high then read water level pin then take low to conserve battery
const byte WaterLevelPin = 21;        // on/off pin for water level
const byte DCVinPin = 36;             // pin for dc input
const byte OLEDResetPin = 16;         // reset pin for OLED display
const byte OLEDSDA = 4;
const byte OLEDSCL = 15;

// analogue input
const int DCVinCycles = 10;                                      // no of reads to average dc reading
const int DCLoopWait = 10;                                       // delay between reads when averaging
const float DCDivideRatio = 100000.0f / (230000.0f + 100000.0f); // resistors in voltage divider (230k top(R1), 100k bottom(R2))
// Delays
const int WaitForKeyDelay = 1000;                        // 1 second delay when starting up to see if the key has been pressed, if so then wakeup
const int WiFiWait = 1000;                               // 1 second delay for wifi to settle, probably not needed
const unsigned long DeepSleepTime = 10 * 60 * 1000000UL; // sleep for 10 minutes (time in microseconds)
const int SendPacketDelay = 2000;                        // delay to allow packet to send
const int KeyPressedDelay = 500;                         // delay to display key pressed message on oled
const int SendPacketLoopDelay = 5000;                    // delay between messages during wakeup loop

const byte LEDOn = HIGH;
const byte LEDOff = LOW;

// lora packet preamble, checked by receiver to see if it is a valid packet
const String LoraPacketPreAmble = "A1A";

// setup the display
SSD1306 display(0x3c, OLEDSDA, OLEDSCL);

void SerialConnect() // do nothing if we don't connect
{
  Serial.begin(115200);
}

void WiFiConnect() // do nothing if it doesn't connect, it means we are out of range
{
  WiFi.disconnect();
  WiFi.begin(WiFiSSID, WiFiPassword);
  WiFi.config(WiFiIP, WiFiGateway, WiFiSubnet, WiFiPrimaryDNS); // must be after begin else it won't connect
  vTaskDelay(pdMS_TO_TICKS(WiFiWait));                          // probably not needed
}

String GetWaterLevel() // check the water level which will either be on or off, i.e. full or not full
{
  String Level = "";
  digitalWrite(WaterLevelTriggerPin, HIGH);   // supply voltage to the water sensor loop
  vTaskDelay(pdMS_TO_TICKS(10));              // wait for filter capacitor to charge
  Level = String(digitalRead(WaterLevelPin)); // read the level (on or off)
  digitalWrite(WaterLevelTriggerPin, LOW);    // remove the supply to save battery
  return Level;
}

String GetVoltageLevel()
{
  // voltage reading
  float Vin = 0.0;
  for (int i = 0; i < DCVinCycles; i++) // taking X samples and averaging them
  {
    Vin += analogRead(DCVinPin);
    vTaskDelay(pdMS_TO_TICKS(DCLoopWait)); // pause between each reading to let it settle
  }
  Vin = (Vin / DCVinCycles); // Get average reading from the divider and manipulate to make more accurate
  // from: https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function/blob/master/ESP32_ADC_Read_Voltage_Accurate.ino
  // takes into account 3.3v max input and 4096 max width, no idea how it works but it improved the accuracy of my readings
  Vin = -0.000000000000016 * pow(Vin, 4) + 0.000000000118171 * pow(Vin, 3) - 0.000000301211691 * pow(Vin, 2) + 0.001109019271794 * Vin + 0.034143524634089;
  Vin = (Vin / DCDivideRatio);   // Get voltage at the top of the divider
  return String(int(Vin * 100)); // move 2 decimal places above the decimal point, make an integer then a string.  Divide by 100 at the receiving end
}
void SetUpOLED()
{
  // setup OLED display
  digitalWrite(OLEDResetPin, LOW); // set GPIO16 low to reset OLED
  vTaskDelay(pdMS_TO_TICKS(50));
  digitalWrite(OLEDResetPin, HIGH); // while OLED is running, must set GPIO16 in high
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

void SendPacket()
{
  LoRa.beginPacket();
  LoRa.print(LoraPacketPreAmble);
  LoRa.print(GetWaterLevel());
  LoRa.print(GetVoltageLevel());
  LoRa.endPacket();
}

void OLEDMessage(String OLEDText)
{
  display.clear();
  display.drawString(0, 0, OLEDText);
  display.display();
}

void setup()
{
  // don't need bluetooth
  btStop();

  // setup pins
  pinMode(OLEDResetPin, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(KEY_BUILTIN, INPUT);
  pinMode(WaterLevelTriggerPin, OUTPUT);
  pinMode(WaterLevelPin, INPUT_PULLUP);

  // setup and start lora
  SPI.begin(SCK, MISO, MOSI, SS);
  //vTaskDelay(pdMS_TO_TICKS(50));
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(LoraBand)) // must have lora to work, will only fail to begin if the lora module is the wrong type
  {
    SetUpOLED();
    display.clear();
    display.drawString(0, 0, "LoRa failed to start, wrong module type");
    display.drawString(0, 12, "Boot stopped");
    display.display();
    while (true)
    {
    }
  }
  LoRa.setSyncWord(0xA1); // ranges from 0-0xFF, default 0x34, see API docs - doesn't seem to work

  // Setup analogue DC input
  analogReadResolution(12);
  analogSetWidth(12);
  analogSetCycles(8);
  analogSetSamples(1);
  analogSetClockDiv(1);
  analogSetPinAttenuation(DCVinPin, ADC_11db); //VP pin

  // wait for key pressed to see if we need to wake up for OTA
  unsigned long Timing = millis();
  while (millis() - Timing < WaitForKeyDelay)
  {
    if (!digitalRead(KEY_BUILTIN)) // if on board key is pressed just after a reboot then go into WakeUpLoop otherwise just send and sleep
    {
      SetUpOLED();
      display.clear();
      display.drawString(0, 0, "Key pressed");
      display.display();
      vTaskDelay(pdMS_TO_TICKS(KeyPressedDelay)); // delay to display key pressed message on screen
      SerialConnect();                            // only need serial if in wakeup loop
      WiFiConnect();                              // only need wifi in wakeup loop
                                                  //OTA callbacks
      ArduinoOTA.onStart([]() {
        OLEDMessage("Starting OTA");
        Serial.println("Starting OTA");
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      });
      ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
      ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      });
      ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

      ArduinoOTA.begin(); // start OTA for wakeup loop
      while (true)        // stay in OTA mode until manual reboot
      {
        // OTA updates
        ArduinoOTA.handle();

        display.clear();
        display.drawString(0, 0, "Water level: " + GetWaterLevel());
        display.drawString(0, 12, "Voltage level: " + String(GetVoltageLevel().toFloat() / 100.00));
        display.display();

        digitalWrite(BUILTIN_LED, LEDOn);
        SendPacket();
        digitalWrite(BUILTIN_LED, LEDOff);
        vTaskDelay(pdMS_TO_TICKS(SendPacketLoopDelay)); // send packet faster than if in sleep mode
      }
    }
  }
  // not waking up so just send and sleep
  WiFi.mode(WIFI_OFF);                        // don't need wifi, shut it down to save battery
  SendPacket();                               // create and send a lora packet
  vTaskDelay(pdMS_TO_TICKS(SendPacketDelay)); // delay to make sure packet has gone b4 going to sleep

  esp_sleep_enable_timer_wakeup(DeepSleepTime);
  esp_deep_sleep_start();
}

void loop() // if sleep enabled then this will never execute, if wakeup is called then it will stay in WakeUpLoop and this will never be executed
{
  vTaskDelay(pdMS_TO_TICKS(100));
}