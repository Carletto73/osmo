/*
  release 08: release candidate
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <Arduino.h>
#include "AsyncTelegram.h"
#include <microDS3231.h>
#include <OneWire.h>
#include <DS18B20.h>
#include <AT24Cxx.h>

// EEPROM inside the DS3231 initialization
#define i2c_address 0x57
AT24Cxx eep(i2c_address, 32);

// OneWire and DS18B20 initialization
#define ONE_WIRE_BUS              2
OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);

// Created object for Telegram
AsyncTelegram myBot;

// Created object for RTC
MicroDS3231 rtc;

// Declaration
byte test;
byte state_releveling = 0;
byte pwm_pump = 255;
byte pwm_fan = 255;
byte pump_fault = 0;
byte last_second_start = 0;
byte last_minute_start = 0;
byte last_hour_start = 0;
byte last_day_start = 0;
byte last_month_start = 0;
byte last_year_start = 0;
byte last_dev_temperature_start = 0;
byte param_filt_max_level = 0;
byte param_filt_min_level = 0;
float param_temp_fan_activation = 0;
byte param_fan_hysteresis = 0; 
byte param_max_time_pump_on = 60;

int signal_percent[] = {0, 0, 0, 0, 0, 0, 4, 6, 8, 11, 13, 15, 17, 19, 21, 23, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45, 46, 48, 50, 52, 53, 55, 56, 58, 59, 61, 62, 64, 65, 67, 68, 69, 71, 72, 73, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 90, 91, 92, 93, 93, 94, 95, 95, 96, 96, 97, 97, 98, 98, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
String st;
float temperature_water;
unsigned long prev_overflow_time;
unsigned long prev_minimum_level_time;
unsigned long begin_time_timeout_pump;
unsigned long prev_state_releveling_time;
unsigned long prev_time_with_pump_stopped;
unsigned long duration_last_pumping;
bool water_overflow = true;
bool water_under_limit = false;
bool mosfet_closed = false;
bool relay_closed = false;
bool pump_timeout = false;
bool pump_stopped_in_previus_activation = true;
bool rtc_working = false;
bool pumping_ended_in_overflow = false;
bool telegram_unconnected;
bool connection_timeout;
bool request_change_account;
bool request_history;
bool request_parameter;
bool fan_switched_on = false;
bool request_preset_available = false;    

// E²PROM
// 0÷95 ID & PSW router
// 96÷159 Token telegram
// 160 pump_fault
// 161 param_filt_max_level
// 162 param_filt_min_level
// 163 param_temp_fan_activation (not float but 24°C + (value 163 * 0.1°C) )
// 164 param_fan_hysteresis
// 165 param_max_time_pump_on

#define OUTPUT_RELAY 14
#define OUTPUT_MOSFET 12
#define OUTPUT_FAN 0
#define ANALOG_INPUT_CHECK A0
#define INPUT_MAX_LEVEL 16
#define INPUT_MIN_LEVEL 13

// Create an instance of the server
// specify the port to listen on as an argument
ESP8266WebServer server(80);

// Main routine
void pump_manager()
{
  unsigned long current_time = millis();

  //Request if the temperature reading is available and launch a new reading
  if (sensor.isConversionComplete())
  {
    temperature_water = sensor.getTempC();  
    sensor.requestTemperatures();
  }

  // Filter on MAX level input
  if (digitalRead(INPUT_MAX_LEVEL))
  {
    if (current_time > (prev_overflow_time + (param_filt_max_level * 100))) // filter for max level
    {
      water_overflow = false;
    }
  }
  else
  {
    prev_overflow_time = current_time;
    water_overflow = true;
  }

  // Filter on MIN level input
  if (digitalRead(INPUT_MIN_LEVEL))
  {
    if (current_time > (prev_minimum_level_time + (param_filt_min_level * 100))) // filter for min level
    {
      water_under_limit = true;
    }
  }
  else
  {
    prev_minimum_level_time = current_time;
    water_under_limit = false;
  }

// These value are taken experimentally
  if (analogRead(ANALOG_INPUT_CHECK) < 175)
  {
    mosfet_closed = true;
    relay_closed = false;
  }
  else if (analogRead(ANALOG_INPUT_CHECK) > 525)
  {
    mosfet_closed = false;
    relay_closed = true;
  }
  else
  {
    mosfet_closed = false;
    relay_closed = false;
  }
  
// If is all right start the routine that feed check the hardware and starts the motor
  if(water_under_limit && !water_overflow && !pump_fault && pump_stopped_in_previus_activation)
  {
    switch (state_releveling)
    {
      case 0: // begin of pump phase
        if (rtc_working)
        {
          last_second_start = rtc.getSeconds();
          last_minute_start = rtc.getMinutes();
          last_hour_start = rtc.getHours();
          last_day_start = rtc.getDate();
          last_month_start = rtc.getMonth();
          last_year_start = rtc.getYear() % 100;
          last_dev_temperature_start = rtc.getTemperature();
          //stringHTML += String(rtc.getTemperatureFloat()).substring(0, 4);
        }
        else
        {
          last_second_start = 0;
          last_minute_start = 0;
          last_hour_start = 0;
          last_day_start = 1;
          last_month_start = 1;
          last_year_start = 20;
          last_dev_temperature_start = 0;
        }
        state_releveling = 1;
        prev_state_releveling_time = current_time;
        duration_last_pumping = 0;
        pumping_ended_in_overflow = false;
        break;
      case 1: // check if mosfet and relay are open
        if (current_time > (prev_state_releveling_time + 100))
        {
          prev_state_releveling_time = current_time;
          if (!mosfet_closed && !relay_closed)
          {
            state_releveling = 2;
          }
          else
          {
            if (relay_closed) 
            {
              pump_fault = 1;  // pump_fault = 1 >>> Relay stucked 
              EEPROM.write(160, pump_fault);
              EEPROM.commit();
            }
            if (mosfet_closed)
            {
              pump_fault = 2; // pump_fault = 2 >>> Mosfet in short-circuit
              EEPROM.write(160, pump_fault);
              EEPROM.commit();
            }
          }
        }
        break;
      case 2: // check if the mosfet works
        analogWrite(OUTPUT_MOSFET, 0); // Switch ON the mosfet
        if (current_time > (prev_state_releveling_time + 100))
        {
          prev_state_releveling_time = current_time;
          if (mosfet_closed)
          {
            analogWrite(OUTPUT_MOSFET, 255); // Switch OFF the mosfet
            state_releveling = 3;
          }
          else
          {
            pump_fault = 3; // pump_fault = 3 >>> Mosfet don't work
            EEPROM.write(160, pump_fault);
            EEPROM.commit();
          }
        }
        break;
      case 3: // delay
        if (current_time > (prev_state_releveling_time + 100))
        {
          prev_state_releveling_time = current_time;
          state_releveling = 4;
        }
        break;
      case 4: // check if the relay works
        digitalWrite(OUTPUT_RELAY, HIGH);
        if (current_time > (prev_state_releveling_time + 100))
        {
          prev_state_releveling_time = current_time;
          if (relay_closed)
          {
            state_releveling = 5;
          }
          else
          {
            pump_fault = 4; // pump_fault = 4 >>> Relay don't work
            EEPROM.write(160, pump_fault);
            EEPROM.commit();
          }
        }
        break;
      case 5: // soft-start pump
        prev_state_releveling_time = current_time;
        if (pwm_pump > 0)
        {
          pwm_pump--;
        }
        else
        {
          state_releveling = 6;
          begin_time_timeout_pump = current_time;
        }
        analogWrite(OUTPUT_MOSFET, pwm_pump); // 0=Switch ON / 255=Switch OFF
        break;
      case 6: // check_timeout
        prev_state_releveling_time = current_time;
        if (current_time > (begin_time_timeout_pump + (param_max_time_pump_on * 1000)))
        {
          pump_fault = 5; // pump_fault = 5 >>> Timeout (max time reach)
          EEPROM.write(160, pump_fault);
          EEPROM.commit();
        }
        break;
    }
  }
  else
  {
    if (pwm_pump < 255)
    {
      if (pwm_pump < 255) pwm_pump++;
      if (pwm_pump < 255) pwm_pump++;
      if (pwm_pump < 255) pwm_pump++;
      prev_state_releveling_time = current_time;
      pump_stopped_in_previus_activation = false;
      state_releveling = 7; // soft-stop pump
      if (water_overflow)
      {
        pwm_pump = 255;
        pumping_ended_in_overflow = true;
      }
    }
    else
    {
      if (current_time > (prev_state_releveling_time + 200))
      {
        digitalWrite(OUTPUT_RELAY, LOW);
        pump_stopped_in_previus_activation = true;
        if (state_releveling != 0)
        {
          state_releveling = 0;
          for (int i = 0 ; i <= 100 ; i++)
          {
            if (eep.read(i * 16) == 0)
            {
              Serial.print("Indice memoria: ");
              Serial.println(i);
              int temp = temperature_water * 100;
              eep.write((i * 16) + 0, last_year_start);                     // year
              eep.write((i * 16) + 1, last_month_start);                    // month
              eep.write((i * 16) + 2, last_day_start);                      // day
              eep.write((i * 16) + 3, last_hour_start);                     // hour
              eep.write((i * 16) + 4, last_minute_start);                   // minute
              eep.write((i * 16) + 5, last_second_start);                   // second
              eep.write((i * 16) + 6, duration_last_pumping / 1000);        // duration second
              eep.write((i * 16) + 7, (duration_last_pumping % 1000) / 10); // duration cent/second
              eep.write((i * 16) + 8, pumping_ended_in_overflow);           // overflow
              eep.write((i * 16) + 9, pump_fault);                          // fault
              eep.write((i * 16) + 10, last_dev_temperature_start);         // temperature device
              eep.write((i * 16) + 11, temp / 100);                         // temperature water
              eep.write((i * 16) + 12, temp % 100);                         // temperature water decimal
              eep.write((i * 16) + 13, 0);                                  // free
              eep.write((i * 16) + 14, 0);                                  // free
              eep.write((i * 16) + 15, 0);                                  // free
              if (i != 100)
              {
                for (int j = 0 ; j < 16 ; j++)
                {
                  eep.write(((i + 1) * 16) + j, 0); // cancel next slot
                }
              }
              else
              {
                for (int j = 0 ; j < 16 ; j++)
                {
                  eep.write(j, 0); // cancel next slot
                }
              }
              break;
            }
          }
        }
      }
    }
    analogWrite(OUTPUT_MOSFET, pwm_pump); // 0=Switch ON / 255=Switch OFF
  }
  if (pwm_pump != 255)
  {
    duration_last_pumping = current_time - prev_time_with_pump_stopped;
  }
  else
  {
    prev_time_with_pump_stopped = current_time;
  }

// FAN manager
  float temp_float = param_fan_hysteresis / 10.0;
  if (temperature_water >= param_temp_fan_activation)
  {
    digitalWrite(OUTPUT_FAN, LOW); // inverted
    fan_switched_on = true;
  }
  else if (temperature_water < param_temp_fan_activation - temp_float)
  {
    digitalWrite(OUTPUT_FAN, HIGH); // inverted
    fan_switched_on = false;
  }
}

void setup()
{

  // Set the output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(OUTPUT_RELAY, OUTPUT);
  pinMode(OUTPUT_MOSFET, OUTPUT);
  pinMode(OUTPUT_FAN, OUTPUT);
  pinMode(INPUT_MAX_LEVEL, INPUT);
  pinMode(INPUT_MIN_LEVEL, INPUT);
  pinMode(ANALOG_INPUT_CHECK, INPUT);
  digitalWrite(LED_BUILTIN, 0);
  digitalWrite(OUTPUT_RELAY, LOW);
  digitalWrite(OUTPUT_MOSFET, HIGH); // inverted
  digitalWrite(OUTPUT_FAN, HIGH); // inverted

  // Opening serial monitor
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println(F("Start Program"));

  // Preset RTC in case of lostPower
  if (rtc.lostPower())
  {
    rtc.setTime(BUILD_SEC, BUILD_MIN, BUILD_HOUR, BUILD_DAY, BUILD_MONTH, BUILD_YEAR);
    Serial.println("DS3231 presetted");
  }

  // Disconnect current wifi
  WiFi.disconnect();

  // Initialasing EEPROM
  EEPROM.begin(512);

  // Check the RTC connection
  if (!rtc.begin())
  {
    rtc_working = false;
    Serial.println("DS3231 not found");
  }
  else
  {
    rtc_working = true;
    Serial.print("DS3231 found ");
    Serial.print(" Date ");
    Serial.print(rtc.getDate());
    Serial.print("/");
    Serial.print(rtc.getMonth());
    Serial.print(" Hour ");
    Serial.print(rtc.getHours());
    Serial.print(":");
    Serial.print(rtc.getMinutes());
    Serial.print(":");
    Serial.println(rtc.getSeconds());
  }

  // Initialasing DS18B20
  sensor.begin();
  sensor.requestTemperatures();
  
  // Read SSID and Password from EEPROM
  Serial.print(F("    SSID stored in EEPROM: "));
  String esid;
  for (byte i = 0; i < 32; ++i)
  {
    esid += char(EEPROM.read(i));
  }
  esid = esid.c_str();
  Serial.println(esid);

  Serial.print(F("PASSWORD stored in EEPROM: "));
  String epass = "";
  for (int i = 32; i < 96; ++i)
  {
    epass += char(EEPROM.read(i));
  }
  epass = epass.c_str();
  //Serial.println(epass);
  Serial.println("******************************");

  Serial.print(F("   TOKEN stored in EEPROM: "));
  char* token = "1234567890:ABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHI";     // REPLACE myToken WITH YOUR TELEGRAM BOT TOKEN
  for (int i = 96; i < 160; ++i)
  {
    token[i - 96] = char(EEPROM.read(i));
  }
  Serial.println(token);

  // Read the parameters on EEPROM
  pump_fault = EEPROM.read(160);
  param_filt_max_level = EEPROM.read(161);
  param_filt_min_level = EEPROM.read(162);
  param_temp_fan_activation = 24 + (EEPROM.read(163) * 0.1);
  param_fan_hysteresis = EEPROM.read(164);
  param_max_time_pump_on = EEPROM.read (165);

  // Connect to WiFi network with 60 seconds of timeout
  Serial.print(F("Connecting to "));
  Serial.println(esid);
  WiFi.setAutoConnect(true);   
  WiFi.mode(WIFI_STA);
  WiFi.begin(esid, epass);

  unsigned long previousMillis = millis();
  while ((WiFi.status() != WL_CONNECTED) && (millis() < 60000))
  {
    if (millis() > previousMillis + 250)
    {
      Serial.print(F("."));
      previousMillis = millis();
    }
  }
  // connection router timeout
  if (millis() >= 60000)
  {
    Serial.println();
    Serial.println(F("Wifi timeout. WiFi NOT connected"));
    connection_timeout = true;

    // Start hotspot
    Serial.println("Server started as HotSpot after timeout (allow to change SSID and PASSWORD)");

    // Before to start the hotspot are checked the networks available
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    int n = WiFi.scanNetworks();
    Serial.println(F("Scan done"));
    if (n == 0)
    {
      Serial.println(F("No networks found"));
      st = "<ol><li>No networks found</li></ol>";
    }
    else
    {
      Serial.print(n);
      Serial.println(F(" networks found"));
      st = F("<ol>");
      for (int i = 0; i < n; ++i)
      {
        // Print SSID and RSSI for each network found
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.print(WiFi.SSID(i));
        Serial.print(F(" ("));
        Serial.print(WiFi.RSSI(i));
        Serial.print(F("dB) ("));
        Serial.print(signal_percent[100 + WiFi.RSSI(i)]);
        Serial.print(F("%)"));
        Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " Not protect" : " Protect");
        st += F("<li>");
        st += WiFi.SSID(i);
        st += F(" (");
        st += WiFi.RSSI(i);
        st += F("dB) (");
        st += signal_percent[100 + WiFi.RSSI(i)];
        st += F("%)");
        st += (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : " *";
        st += F("</li>");
        delay(10);
      }
      st += F("</ol>");
      st += F("(*) Protected</br>");
      Serial.println("");
    }

    // Opening of hotspot
    delay(200);
    WiFi.softAP("Osmo", "Osmolator");
    Serial.println("Initializing_softap_for_wifi credentials_modification");
    server.onNotFound(handleWebPage);
    
    // CreateWebServer();
    server.begin();

    // Preset timers
    prev_overflow_time = millis();
    prev_minimum_level_time = prev_overflow_time;
    begin_time_timeout_pump = prev_overflow_time;
    prev_state_releveling_time = prev_overflow_time;
    prev_time_with_pump_stopped = prev_overflow_time;
    prev_time_with_pump_stopped = duration_last_pumping;
  }
  // connection on time
  else
  {
    Serial.println();
    Serial.println(F("WiFi connected"));
    digitalWrite(LED_BUILTIN, 1);
    // set the local DNS name
    if (MDNS.begin("osmo"))
    {
      Serial.println("MDNS responder started.      HTTP://osmo.local/");
    }
    // for all web request it calls the handleWebPage function
    server.onNotFound(handleWebPage);
    // Start the server
    server.begin();
    Serial.println(F("Server started"));

    // Print the IP address
    Serial.println(WiFi.localIP());

    // To ensure certificate validation, WiFiClientSecure needs time updated
    // myBot.setInsecure(false);
    myBot.setClock("CET-1CEST,M3.5.0,M10.5.0/3");

    // Set the Telegram bot properies
    myBot.setUpdateTime(1000);
    myBot.setTelegramToken(token);

    // Check if all things are ok
    Serial.print("\nTest Telegram connection... ");
    if (myBot.begin())
    {
      Serial.println(F("Telegram connection OK"));
    }
    else
    {
      Serial.println(F("Telegram connection NOT OK"));
      telegram_unconnected = true;
    }

    // Preset timers
    prev_overflow_time = millis();
    prev_minimum_level_time = prev_overflow_time;
    begin_time_timeout_pump = prev_overflow_time;
    prev_state_releveling_time = prev_overflow_time;
    prev_time_with_pump_stopped = prev_overflow_time;
    prev_time_with_pump_stopped = duration_last_pumping;
  }
}

void loop()
{
  server.handleClient();
  if (connection_timeout == false) {MDNS.update();}
  if ((telegram_unconnected == false) && (connection_timeout == false)) {handleTelegram();}
  pump_manager();
  delay(5); // this delay blocking is realy need. removing it web and telegram not work
}

void handleWebPage()
{
  // Read the request
  String req = server.uri();
  int history_page;

  if (req == "/favicon.ico") return;
 
  test += 1;

  // check if was required a fault reset
  if (req.indexOf(F("/reset")) != -1)
  {
    pump_fault = 0;    
    EEPROM.write(160, pump_fault);
    EEPROM.commit();
  }

  // check if was required a credential network change
  if (req.indexOf(F("/account")) != -1)
  {
    request_change_account = true;    
  }
  else
  {
    request_change_account = false;    
  }
  
  // check if was required a history activation page
  if (req.indexOf(F("/history")) != -1)
  {
    request_history = true;
    history_page = server.arg(0).toInt();
  }
  else
  {
    request_history = false;    
  }

  // check if was required a preset parameters
  if (req.indexOf(F("/preset")) != -1)
  {
    if (request_preset_available)
    {
      EEPROM.write(161, 10);
      EEPROM.write(162, 10);
      EEPROM.write(163, 30);
      EEPROM.write(164, 10);
      EEPROM.write(165, 60);
      EEPROM.commit();
      param_filt_min_level = 10;
      param_filt_max_level = 10;
      param_temp_fan_activation = 24 + (30 * 0.1);
      param_fan_hysteresis = 10;
      param_max_time_pump_on = 60;
      for (int i = 0 ; i <= 1616 ; i++)
      {
        eep.write(i, 0);
      }
    }
    request_preset_available = false; 
  }
  else
  {
    request_preset_available = true;    
  }

  // check if was required a parameters change
  if (req.indexOf(F("/parameter")) != -1)
  {
    request_parameter = true;
    if (server.arg(0).length() > 0)
    {
      param_filt_min_level = server.arg(0).toInt();
      param_filt_max_level = server.arg(1).toInt();
      param_temp_fan_activation = 24 + (server.arg(2).toInt() * 0.1);
      param_fan_hysteresis = server.arg(3).toInt();
      param_max_time_pump_on = server.arg(4).toInt();
      EEPROM.write(161, param_filt_max_level);
      EEPROM.write(162, param_filt_min_level);
      EEPROM.write(163, server.arg(2).toInt());
      EEPROM.write(164, param_fan_hysteresis);
      EEPROM.write(165, param_max_time_pump_on);
      EEPROM.commit();
    }
  }
  else
  {
    request_parameter = false;    
  }

  // check if was required a data change
  if (req.indexOf(F("/date")) != -1)
  {
    rtc.setTime(0, server.arg(4).toInt(), server.arg(3).toInt(), server.arg(0).toInt(), server.arg(1).toInt(), server.arg(2).toInt());
    Serial.println("Changed hour");
  }

  // check if was required a setting change
  if (req.indexOf(F("/setting")) != -1)
  {
    String qsid = server.arg(0);
    String qpass = server.arg(1);
    String qtoken = server.arg(2);
    Serial.println("");
    if (qsid.length() > 0 && qpass.length() > 0)
    {
      Serial.println(F("Clearing eeprom"));
      for (int i = 0; i < 96; ++i)
      {
        EEPROM.write(i, 0);
      }
      Serial.println(qsid);
      Serial.println(qpass);
      Serial.println(F("Writing eeprom SSID:"));
      for (int i = 0; i < qsid.length(); ++i)
      {
        EEPROM.write(i, qsid[i]);
      }
      Serial.println(F("Writing eeprom PASSWORD:"));
      for (int i = 0; i < qpass.length(); ++i)
      {
        EEPROM.write(32 + i, qpass[i]);
      }
      EEPROM.commit();
    }
  
    if (qtoken.length() > 0)
    {
      Serial.println(F("Clearing eeprom"));
      for (int i = 96; i < 160; ++i)
      {
        EEPROM.write(i, 0);
      }
      Serial.println(qtoken);
      Serial.println(F("Writing eeprom TOKEN:"));
      for (int i = 0; i < qtoken.length(); ++i)
      {
        EEPROM.write(96 + i, qtoken[i]);
      }
      EEPROM.commit();
     }

    // Disconnect current wifi in case of parameter changed
    if ((qsid.length() > 0 && qpass.length() > 0) || (qtoken.length() > 0))
    {
      WiFi.disconnect();
      ESP.reset();
    }
  }

  // Main web page
  String stringHTML = F("");
  if (!connection_timeout && !request_change_account && !request_history && !request_parameter)
  {
    stringHTML += F("<html>");
    stringHTML += F("<head>");
    if (req == "/") stringHTML += F("<meta http-equiv=\"refresh\" content=\"3\">");
    stringHTML += F("</head>");
    stringHTML += F("<body>");
    stringHTML += F("<table style=\"border-collapse: collapse; width: 100%;\" border=\"1\">");
    stringHTML += F("<tbody>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 36pt;\"><strong><font color=\"red\"><a title=\"refresh\" style=\"text-decoration: none\" href=\"http:/\">OSMOLATOR</a></font></strong></td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    if (pump_fault) {stringHTML += F("<td style=\"width: 100%; text-align: center;\"><img src=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAATIAAADoCAYAAABsFGg4AAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAD0FSURBVHhe7Z0HmBRV1obNCkgOkkZyTpKDJMlBsogiIEqQIFlAFgXBkaioiKwEUZAoUYEFySACCyuwgOCKAsoKIiqo67rq7n/++k5X9VT33M5V1dU953ue75mZquqqWzV1377h3HtvIJFIJEpweUH2zTff0IULF8RisTghbJYXZKtWraLVq1fT9u3bxWKx2NVesGAB/f3vf9fpZQLZX/7yF/rnP/+p/yUSiUTu1V//+lcBmUgkSmwJyEQiUcJLQCYSiRJeAjKRSJTwEpCJRKKEl4BMJBIlvARkIpEo4WUbyP7973/T66+/Ts8995xYLA5gBHLu3r1bzzWiaGUbyHDSG264QSwWh/CMGTP0XCOKVraBDN8yqn+aWCz2NUpm8dL//d//sVX63//+l24f/sZ2lVTHQ4GOD3btSOUIyKToLBKlV+PGjeMKsuvXr9OePXt4XPWXX36pb/Xo4MGDtHDhQjp//ry+hei///0v/43q8LFjx3wghO0rVqyg/fv30++//65vJfrPf/5Db7zxBm3evJl++uknfSvR1atXacuWLTye+8qVK/rW6CUgE4nipHiD7JNPPqH27dtT+fLladOmTfpWj3r06EHZs2en9957T99C9Msvv9DixYspZ86clJqaymAztG7dOqpQoQL179+frl27pm8l+v777+m2226junXr0hdffKFvJTp69Ci1aNGCypUrR7t27dK3Ri8BmUgUJ8UbZCh1IQ1lypShDRs26Fs9atWqFWXKlMkHZCjBzZ07l3LkyEETJ070Adm7775LZcuWpV69etHXX3+tb/WA7JZbbqGSJUvSP/7xD30r0ZEjR6hp06ZUvHhxWrlypb41egnIRKI4SUAmIBOJEl4CMgGZSJTwEpAJyNIJ18A/RSx2g3v37q2/mYGF45wGGRrWBw8eTNWqVaOUlBTKkiUL3X777VSgQAEGmmFsv+mmm6hgwYLebYDRXXfdRTfffDPlzp2bSpcu7d2XP39+Pg86CAAn82duvPFGbvAvWrSodzuufeedd/J2fBYdDp06dWIgApiRKmKQ/f3ECZoxcyZNmvw8PfPsBBr79Dj2uD+N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\"></td>");}
    else if (pwm_pump < 255) {stringHTML += F("<td style=\"width: 100%; text-align: center;\"><img src=\"data:image/png;base64,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\"></td>");}
    else if (water_overflow) {stringHTML += F("<td style=\"width: 100%; text-align: center;\"><img src=\"data:image/png;base64,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\"></td>");}
    else if (fan_switched_on) {stringHTML += F("<td style=\"width: 100%; text-align: center;\"><img src=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAATEAAADnCAYAAAB2dWHuAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAD/XSURBVHhe7Z0HmBTFtscNFxWRICgSJSMZSRIuApIlBzGgBCUKShQQURFYiUqGSxIk56DAE8kgAhdUuIBiBLw8RUUvqFd96n3vvPqf6d7tma2ZndDd0zN7/t/3/3a3u6fD7NRvTlWdqrqGRCKRKIHFEPvmm2/owoULYrFYnBC2iiG2du1aWr9+Pe3atUssFos97YULF9I//vEPBhjEEPuv//ov+u///m/eIBKJRF7W3//+d4GYSCRKXAnERCJRQksgJhKJEloCMZFIlNASiIlEooSWQEwkEiW0BGIikSih5QzEfv2VaM4copdeEovFOu/bZxQWUaxyBmI44TXqVGKxWO/Jk43CIopVzkAM3zK6f5xYLPYZ0Vic9X//939snf73f/833T78je066Y6Hgh0f6tqRynmISdgsEqWpQQNPQOzq1au0f/9+Hif95ZdfGlt9OnLkCC1atIjOnz9vbCH6z3/+w39jnOKJEyf8AITtq1evpkOHDtEff/xhbCX6n//5H5o3bx5t376dfvrpJ2Mr0eXLl+mtt97i8dnffvutsTV6CcREIjflEYh9+OGH1KZNGypXrhxt27bN2OrTY489Rjlz5qQ33njD2EL0yy+/0NKlS+nWW2+llJQUhpqpTZs2Ufny5al379505coVYyvRDz/8QDfccAPVrl2bvvjiC2Mr0QcffEBNmzalsmXL0t69e42t0UsgJhK5KY9ADNFWA3Uvd911F23ZssXY6lPz5s0pa9asfhBD5DZ37lzKlSsXjR492g9i69atozJlylDXrl3pq6++Mrb6IPaXv/yFSpYsSZ988omxlej48ePUqFEjKl68OK1Zs8bYGr0EYiKRmxKICcREooSWQEwgJhIltARiAjGd9qlr4B8iFnvB3bt3Nz6ZGqn98YIYGtH79+9PVatWpcKFC1O2bNnoxhtvpPz58zPMTGP7ddddRwUKFEjdBhDdcccddP3111OePHmodOnSqfvy5cvH50FnAMBkfc21117LjftFixZN3Y5r33LLLbwdr0XnQvv27RmGgGWkighi/zh1iiZPmUJjxo6j5194kUY8O5I98rlR9KKi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\"></td>");}
    else {stringHTML += F("<td style=\"width: 100%; text-align: center;\"><img src=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAATIAAADnCAYAAACdQtrtAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAD3kSURBVHhe7Z0HmBRVuoYNiwkJgiJpJCMZSRIkSRYkCGJARRQQBCWKyKIoOAKCoiCyEkRBSSqgAhckowgurMAChjUALldRUUFd16vuvf+t7+86M9U9p7q7uquqq3v+73m+Z2aqqqtO1dR5+4T/nHMGmfr666/p2LFjYrFYnBa2KgdkK1asoFdffZU2bdokFovFgfb8+fPp73//u0kvC8j+67/+i/77v//b/EskEomCq7/+9a8CMpFIlN4SkIlEorSXgEwkEqW9BGQikSjtJSATiURpLwGZSCRKewnIRCJR2sszkP373/+mZ599lh555BGxWGxjBHJu27bNzDWiROUZyHDSM844QywWx/C0adPMXCNKVJ6BDN8yun+aWCwON0pmqdL//d//sXX63//93zz78De266Q7HrI7Ptq1ncoXkEnRWSTKq9atW6cUZKdPn6bt27fzuOovvvjC3BrS7t27acGCBXT06FFzC9F//vMf/hvV4f3794dBCNuXLVtGO3fupN9//93cSvQ///M/9Nxzz9G6devop59+MrcSnTx5ktavX8/jub/55htza+ISkIlEKVKqQfbBBx9Qt27dqEaNGrR27Vpza0i33norFSlShN544w1zC9Evv/xCixYtoosuuoiys7MZbEqrVq2imjVr0l133UWnTp0ytxJ9//33dM4551DTpk3p888/N7cS7du3jzp06EDVq1enrVu3mlsTl4BMJEqRUg0ylLqQhssvv5xef/11c2tInTp1ovPPPz8MZCjBzZkzh4oWLUoPP/xwGMheeeUVqlatGvXt25e+/PJLc2sIZH/605+ocuXK9I9//MPcSrR3715q27YtVaxYkZYvX25uTVwCMpEoRRKQCchEorSXgExAJhKlvQRkArI8wjXwTxGLg+B+/fqZb6a9cJzfIEPD+tChQ6l+/fqUlZVFBQsWpHPPPZdKlSrFQFPG9rPOOotKly6dsw0wuvTSS+nss8+m4sWLU9WqVXP2lSxZks+DDgLAyfqZM888kxv8y5cvn7Md177wwgt5Oz6LDofrrruOgQhgOpVjkP394EGaNn06TZz0KD340AQa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\"></td>");}
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\">Temperature of the water: <strong><font color=\"blue\">");
    stringHTML += temperature_water;
    stringHTML += char(176);
    stringHTML += F("C</font></strong></td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\">Status of the osmolator: ");
    if (pump_fault == 0)
    {
      if (pwm_pump < 255) stringHTML += F("<font color=\"brown\"><strong>Releveling</strong></font>");
      else if (water_overflow) stringHTML += F("<font color=\"orange\"><strong>Overflow</strong></font>");
      else if (fan_switched_on) stringHTML += F("<font color=\"green\"><strong>Cooling fan on</strong></font>");
      else {stringHTML += F("<font color=\"green\"><strong>GOOD!</strong></font>");}
    }
    else if (pump_fault == 1) {stringHTML += F("<font color=\"red\"><strong>Hard fault</br>Relay stucked</br>Press <a title=\"refresh\" style=\"text-decoration: none\" href=\"http:/reset\">HERE</a> to reset</strong></font>");}
    else if (pump_fault == 2) {stringHTML += F("<font color=\"red\"><strong>Hard fault</br>Mosfet in short-circuit</br>Press <a title=\"refresh\" style=\"text-decoration: none\" href=\"http:/reset\">HERE</a> to reset</strong></font>");}
    else if (pump_fault == 3) {stringHTML += F("<font color=\"red\"><strong>Hard fault</br>Mosfet don't work</br>Press <a title=\"refresh\" style=\"text-decoration: none\" href=\"http:/reset\">HERE</a> to reset</strong></font>");}
    else if (pump_fault == 4) {stringHTML += F("<font color=\"red\"><strong>Hard fault</br>Relay don't work</br>Press <a title=\"refresh\" style=\"text-decoration: none\" href=\"http:/reset\">HERE</a> to reset</strong></font>");}
    else if (pump_fault == 5) {stringHTML += F("<font color=\"red\"><strong>Hard fault</br>Maximum pump time reached</br>Press <a title=\"refresh\" style=\"text-decoration: none\" href=\"http:/reset\">HERE</a> to reset</strong></font>");}
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\">Temperature of the osmolator controller: <strong>");
    if (rtc_working)
    {
      stringHTML += F("<font color=\"blue\">");
      stringHTML += rtc.getTemperature();
      stringHTML += char(176);
      stringHTML += "C";
   }
    else
    {
      stringHTML += F("<font color=\"red\">");
      stringHTML += "na (Check RTC battery)";
    }
    stringHTML += F("</font></strong></td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\">Status of the minimum water level sensor: ");
    if (water_under_limit) {stringHTML += F("<font color=\"orange\"><strong>Water below the minimum level</strong></font>");}
    else {stringHTML += F("<font color=\"green\"><strong>Water level normal</strong></font>");}
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\">Status of the overflow level sensor: ");
    if (water_overflow) {stringHTML += F("<font color=\"red\"><strong>Water above the maximum level</strong></font>");}
    else {stringHTML += F("<font color=\"green\"><strong>Water level normal</strong></font>");}
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\">Last activation of the pump: <strong>");
    if (last_day_start < 10 )stringHTML += F("0");
    stringHTML += last_day_start;
    stringHTML += F("/");
    if (last_month_start < 10 )stringHTML += F("0");
    stringHTML += last_month_start;
    stringHTML += F("/");
    if (last_year_start < 10 )stringHTML += F("0");
    stringHTML += last_year_start;
    stringHTML += F(" ");
    if (last_hour_start < 10 )stringHTML += F("0");
    stringHTML += last_hour_start;
    stringHTML += F(":");
    if (last_minute_start < 10 )stringHTML += F("0");
    stringHTML += last_minute_start;
    stringHTML += F(":");
    if (last_second_start < 10 )stringHTML += F("0");
    stringHTML += last_second_start;
    stringHTML += F("</strong></td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\">Last running time of the pump: <strong>");
    stringHTML += duration_last_pumping / 1000;
    stringHTML += F(",");
    if (((duration_last_pumping % 1000) / 10) < 10 )stringHTML += F("0");
    stringHTML += (duration_last_pumping % 1000) / 10;
    stringHTML += F(" s</strong></td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\"><a title=\"See operating history\" style=\"text-decoration: none\"  href=\"http:/history?page=1\">See operating history</a></td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\"><a title=\"Change parametrization\" style=\"text-decoration: none\"  href=\"http:/parameter\">Change parametrization</a></td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\"><a title=\"Change router credential or Telegram token\" style=\"text-decoration: none\"  href=\"http:/account\">Change router credential or Telegram token</a></td>");
    stringHTML += F("</tr>");
    stringHTML += F("</tbody>");
    stringHTML += F("</table>");
    stringHTML += F("</body>");
    stringHTML += F("</html>");
    server.send(200, "text/html", stringHTML);
  }

// Web page for credential network change
  if (connection_timeout  || request_change_account)
  {
    stringHTML = F("<!DOCTYPE HTML>\r\n<html>Welcome to OSMO Wifi Credentials Update<p>");
    stringHTML += st;
    stringHTML += F("</p><form method='post' enctype='application/x-www-form-urlencoded' action='setting'><label>SSID: </label><input name='ssid' length=32> PASSWORD: <input type=\"password\" name='pass' length=64> TELEGRAM TOKEN: <input name='token' length=64><input type='submit'></form>");
    if (request_change_account == 1)
    {
      stringHTML += F("</br><a title=\"Home\" style=\"text-decoration: none\"  href=\"http:/\">Home</a>");
    }
    stringHTML += F("</html>");
    server.send(200, "text/html", stringHTML);
  }

// web page for list of activation
  if (request_history)
  {
    stringHTML += F("<html>");
    stringHTML += F("<head>");
    stringHTML += F("</head>");
    stringHTML += F("<body>");
    stringHTML += F("<table style=\"border-collapse: collapse; width: 100%;\" border=\"1\">");
    stringHTML += F("<tbody>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 36pt;\"><strong><font color=\"red\"><a title=\"refresh\" style=\"text-decoration: none\" href=\"http:/\">OSMOLATOR</a></font></strong></td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center;\">");
    stringHTML += F("<table style=\"border-collapse: collapse; width: 100%;\" border=\"1\">");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 9%; text-align: center; font-size: 12pt;\"><strong>Day</strong></td>");
    stringHTML += F("<td style=\"width: 9%; text-align: center; font-size: 12pt;\"><strong>Hour</strong></td>");
    stringHTML += F("<td style=\"width: 9%; text-align: center; font-size: 12pt;\"><strong>Duration</strong></td>");
    stringHTML += F("<td style=\"width: 16%; text-align: center; font-size: 12pt;\"><strong>Temperature water</strong></td>");
    stringHTML += F("<td style=\"width: 14%; text-align: center; font-size: 12pt;\"><strong>Temperature osmolator</strong></td>");
    stringHTML += F("<td style=\"width: 29%; text-align: center; font-size: 12pt;\"><strong>Fault</strong></td>");
    stringHTML += F("<td style=\"width: 14%; text-align: center; font-size: 12pt;\"><strong>Overflow</strong></td>");
    stringHTML += F("</tr>");
    int i;
    for (i = 0 ; i <= 100 ; i++)
    {
      if (eep.read(i * 16) == 0)
      {
        i--;
        if (i < 0) i = 100;
        break;
      }
    }
    i = i - ((history_page - 1) * 10);
    if (i < 0) i = 100 + i;
    for (int j = 0 ; j < 10 ; j++)
    {
      stringHTML += F("<tr>");
      stringHTML += F("<td style=\"width: 9%; text-align: center; font-size: 12pt;\">");
      if (eep.read((i * 16) + 2) < 10) stringHTML += "0";
      stringHTML += eep.read((i * 16) + 2); // day
      stringHTML += F("/");
      if (eep.read((i * 16) + 1) < 10) stringHTML += "0";
      stringHTML += eep.read((i * 16) + 1); // month
      stringHTML += F("/");
      if (eep.read((i * 16) + 0) < 10) stringHTML += "0";
      stringHTML += eep.read((i * 16) + 0); // year
      stringHTML += F("</td>");
      stringHTML += F("<td style=\"width: 9%; text-align: center; font-size: 12pt;\">");
      if (eep.read((i * 16) + 3) < 10) stringHTML += "0";
      stringHTML += eep.read((i * 16) + 3); // hour
      stringHTML += F(":");
      if (eep.read((i * 16) + 4) < 10) stringHTML += "0";
      stringHTML += eep.read((i * 16) + 4); // minutes
      stringHTML += F(":");
      if (eep.read((i * 16) + 5) < 10) stringHTML += "0";
      stringHTML += eep.read((i * 16) + 5); // seconds
      stringHTML += F("</td>");
      stringHTML += F("<td style=\"width: 9%; text-align: center; font-size: 12pt;\">");
      stringHTML += eep.read((i * 16) + 6); // duration in second
      stringHTML += F(",");
      if (eep.read((i * 16) + 7) < 10) stringHTML += "0";
      stringHTML += eep.read((i * 16) + 7); // duration cent/second
      stringHTML += F(" s</td>");
      stringHTML += F("<td style=\"width: 16%; text-align: center; font-size: 12pt;\">");
      stringHTML += eep.read((i * 16) + 11); // temperature water
      stringHTML += F(",");
      if (eep.read((i * 16) + 12) < 10) stringHTML += "0";
      stringHTML += eep.read((i * 16) + 12); // temperature water decimal
      stringHTML += char(176);
      stringHTML += F("C</td>");
      stringHTML += F("<td style=\"width: 14%; text-align: center; font-size: 12pt;\">");
      stringHTML += eep.read((i * 16) + 10); // temperature osmo
      stringHTML += char(176);
      stringHTML += F("C</td>");
      stringHTML += F("<td style=\"width: 29%; text-align: center; font-size: 12pt;\">");
      char temp_fault = eep.read((i * 16) + 9); // fault
      if (temp_fault == 0) {stringHTML += F("None");}
      else if (temp_fault == 1) {stringHTML += F("Relay stucked");}
      else if (temp_fault == 2) {stringHTML += F("Mosfet in short-circuit");}
      else if (temp_fault == 3) {stringHTML += F("Mosfet don't work");}
      else if (temp_fault == 4) {stringHTML += F("Relay don't work");}
      else if (temp_fault == 5) {stringHTML += F("Maximum pump time reached");}
      stringHTML += F("</td>");
      stringHTML += F("<td style=\"width: 14%; text-align: center; font-size: 12pt;\">");
      if (eep.read((i * 16) + 8) == 1) {stringHTML += F("YES");}
      else {stringHTML += F("NO");}
      stringHTML += F("</td>");
      stringHTML += F("</tr>");
      i--;
      if (i < 0) i = 100;
    }
    stringHTML += F("</table>");
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\">Sheet ");
    for (int a = 1 ; a <= 10 ; a++)
    {
      if (a != history_page)
      {
        stringHTML += F("<a title=\"Page ");
        stringHTML += a;
        stringHTML += F("\" style=\"text-decoration: none\"  href=\"http:/history?page=");
        stringHTML += a;
        stringHTML += F("\">");
        stringHTML += a;
        stringHTML += F(" </a>");
      }
      else
      {
        stringHTML += F("<strong>");
        stringHTML += a;
        stringHTML += F(" </strong>");
      }
    }
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\"><a title=\"Home\" style=\"text-decoration: none\"  href=\"http:/\">Home</a></td>");
    stringHTML += F("</tr>");
    stringHTML += F("</tbody>");
    stringHTML += F("</table>");
    stringHTML += F("</body>");
    stringHTML += F("</html>");
    server.send(200, "text/html", stringHTML);
  }

// web page for setting parametrization
  if (request_parameter)
  {
    stringHTML += F("<html>");
    stringHTML += F("<head>");
    stringHTML += F("</head>");
    stringHTML += F("<body>");
    stringHTML += F("<table style=\"border-collapse: collapse; width: 100%;\" border=\"1\">");
    stringHTML += F("<tbody>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 36pt;\"><strong><font color=\"red\"><a title=\"refresh\" style=\"text-decoration: none\" href=\"http:/\">OSMOLATOR</a></font></strong></td>");
    stringHTML += F("</tr>");
    stringHTML += F("<form action=\"/parameter\">");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 14pt;\">Minimum water level sensor rebound filter: ");
    stringHTML += F("<select name=\"min_filter\" id=\"min_filter\">");
    for (byte i = 5; i < 51; i++)
    {
      stringHTML += F("<option value=\"");
      stringHTML += i;
      stringHTML += F("\"");
      if (i == param_filt_min_level)
      {
        stringHTML += F(" selected");
      }
      stringHTML += F(">");

      stringHTML += i * 100;
      stringHTML += F(" ms</option>");
    }
    stringHTML += F("</select> ");
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 14pt;\">Overflow sensor rebound filter: ");
    stringHTML += F("<select name=\"max_filter\" id=\"max_filter\">");
    for (byte i = 5; i < 51; i++)
    {
      stringHTML += F("<option value=\"");
      stringHTML += i;
      stringHTML += F("\"");
      if (i == param_filt_max_level)
      {
        stringHTML += F(" selected");
      }
      stringHTML += F(">");
      stringHTML += i * 100;
      stringHTML += F(" ms</option>");
    }
    stringHTML += F("</select> ");
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 14pt;\">Activation temperature of the aquarium cooling fan: ");
    stringHTML += F("<select name=\"t_fan\" id=\"t_fan\">");
    int temp_int;
    for (int i = 0; i < 61; i++)
    {
      stringHTML += F("<option value=\"");
      stringHTML += i;
      stringHTML += F("\"");
      temp_int = (param_temp_fan_activation - 24) / 0.1;
      if (i == temp_int)
      {
        stringHTML += F(" selected");
      }
      stringHTML += F(">");
      stringHTML += 24 + (i * 0.1);
      stringHTML += char(176);
      stringHTML += F("C</option>");
    }
    stringHTML += F("</select> ");
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 14pt;\">Fan hysteresis: ");
    stringHTML += F("<select name=\"h_fan\" id=\"h_fan\">");
    for (int i = 0; i < 21; i++)
    {
      stringHTML += F("<option value=\"");
      stringHTML += i;
      stringHTML += F("\"");
      if (i == param_fan_hysteresis)
      {
        stringHTML += F(" selected");
      }
      stringHTML += F(">");
      stringHTML += i * 0.1;
      stringHTML += char(176);
      stringHTML += F("C</option>");
    }
    stringHTML += F("</select> ");
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 14pt;\">Maximum pump activation time (after a fault occurs): ");
    stringHTML += F("<select name=\"t_max\" id=\"t_max\">");
    for (int i = 10; i < 241; i++)
    {
      stringHTML += F("<option value=\"");
      stringHTML += i;
      stringHTML += F("\"");
      if (i == param_max_time_pump_on)
      {
        stringHTML += F(" selected");
      }
      stringHTML += F(">");
      stringHTML += i;
      stringHTML += F(" s</option>");
    }
    stringHTML += F("</select> ");
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 14pt;\">Save the data ");
    stringHTML += F("<input type=\"submit\" value=\" SAVE \">");
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("</form>");
    stringHTML += F("<form action=\"/date\">");
    byte current_minute = 0;
    byte current_hour = 0;
    byte current_day = 1;
    byte current_month = 1;
    int current_year = 2020;
    if (rtc_working)
    {
      current_minute = rtc.getMinutes();
      current_hour = rtc.getHours();
      current_day = rtc.getDate();
      current_month = rtc.getMonth();
      current_year = rtc.getYear();
    }
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 14pt;\">");
    stringHTML += F("Date ");
    stringHTML += F("<select name=\"day\" id=\"day\">");
    for (int i = 1; i < 32; i++)
    {
      stringHTML += F("<option value=\"");
      stringHTML += i;
      stringHTML += F("\"");
      if (i == current_day)
      {
        stringHTML += F(" selected");
      }
      stringHTML += F(">");
      stringHTML += i;
      stringHTML += F("</option>");
    }
    stringHTML += F("</select> ");
    stringHTML += F("/");
    stringHTML += F("<select name=\"month\" id=\"month\">");
    for (int i = 1; i < 13; i++)
    {
      stringHTML += F("<option value=\"");
      stringHTML += i;
      stringHTML += F("\"");
      if (i == current_month)
      {
        stringHTML += F(" selected");
      }
      stringHTML += F(">");
      stringHTML += i;
      stringHTML += F("</option>");
    }
    stringHTML += F("</select> ");
    stringHTML += F("/");
    stringHTML += F("<select name=\"year\" id=\"year\">");
    for (int i = 2020; i < 2041; i++)
    {
      stringHTML += F("<option value=\"");
      stringHTML += i;
      stringHTML += F("\"");
      if (i == current_year)
      {
        stringHTML += F(" selected");
      }
      stringHTML += F(">");
      stringHTML += i;
      stringHTML += F("</option>");
    }
    stringHTML += F("</select> ");
    stringHTML += F("Hour ");
    stringHTML += F("<select name=\"hour\" id=\"hour\">");
    for (int i = 0; i < 24; i++)
    {
      stringHTML += F("<option value=\"");
      stringHTML += i;
      stringHTML += F("\"");
      if (i == current_hour)
      {
        stringHTML += F(" selected");
      }
      stringHTML += F(">");
      stringHTML += i;
      stringHTML += F("</option>");
    }
    stringHTML += F("</select> ");
    stringHTML += F(":");
    stringHTML += F("<select name=\"minute\" id=\"minute\">");
    for (int i = 0; i < 60; i++)
    {
      stringHTML += F("<option value=\"");
      stringHTML += i;
      stringHTML += F("\"");
      if (i == current_minute)
      {
        stringHTML += F(" selected");
      }
      stringHTML += F(">");
      stringHTML += i;
      stringHTML += F("</option>");
    }
    stringHTML += F("</select> ");
    stringHTML += F("<input type=\"submit\" value=\" CHANGE DATE/HOUR \">");
    stringHTML += F("</td>");
    stringHTML += F("</tr>");
    stringHTML += F("</form>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\"><a title=\"Preset\" style=\"text-decoration: none\"  href=\"http:/preset\">Preset parameter and delete history activations</a></td>");
    stringHTML += F("</tr>");
    stringHTML += F("<tr>");
    stringHTML += F("<td style=\"width: 100%; text-align: center; font-size: 24pt;\"><a title=\"Home\" style=\"text-decoration: none\"  href=\"http:/\">Home</a></td>");
    stringHTML += F("</tr>");
    stringHTML += F("</tbody>");
    stringHTML += F("</table>");
    stringHTML += F("</body>");
    stringHTML += F("</html>");
    server.send(200, "text/html", stringHTML);
  }
}

void handleTelegram()
{
  // a variable to store telegram message data
  TBMessage msg;
  String msg_telegram = "";

  // if there is an incoming message...
  if (myBot.getNewMessage(msg))
  {
    if (msg.text.equalsIgnoreCase("/Last"))
    { 
      int i;
      msg_telegram = "Latest activations\n";
  
      for (i = 0 ; i <= 100 ; i++)
      {
        if (eep.read(i * 16) == 0)
        {
          i--;
          if (i < 0) i = 100;
          break;
        }
      }
      for (int j = 0 ; j < 4 ; j++)
      {
        msg_telegram += String(j + 1);
        msg_telegram += ") Date ";
        if (eep.read((i * 16) + 2) < 10) msg_telegram += "0";
        msg_telegram += eep.read((i * 16) + 2); // day
        msg_telegram += F("/");
        if (eep.read((i * 16) + 1) < 10) msg_telegram += "0";
        msg_telegram += eep.read((i * 16) + 1); // month
        msg_telegram += F("/");
        if (eep.read((i * 16) + 0) < 10) msg_telegram += "0";
        msg_telegram += eep.read((i * 16) + 0); // year
        msg_telegram += F(" Hour ");
        if (eep.read((i * 16) + 3) < 10) msg_telegram += "0";
        msg_telegram += eep.read((i * 16) + 3); // hour
        msg_telegram += F(":");
        if (eep.read((i * 16) + 4) < 10) msg_telegram += "0";
        msg_telegram += eep.read((i * 16) + 4); // minutes
        msg_telegram += F(":");
        if (eep.read((i * 16) + 5) < 10) msg_telegram += "0";
        msg_telegram += eep.read((i * 16) + 5); // seconds
        msg_telegram += F("\nDuration ");
        msg_telegram += eep.read((i * 16) + 6); // duration in second
        msg_telegram += F(",");
        if (eep.read((i * 16) + 7) < 10) msg_telegram += "0";
        msg_telegram += eep.read((i * 16) + 7); // duration cent/second
        msg_telegram += F("s Temp ");
        msg_telegram += eep.read((i * 16) + 11); // temperature water
        msg_telegram += F(",");
        if (eep.read((i * 16) + 12) < 10) msg_telegram += "0";
        msg_telegram += eep.read((i * 16) + 12); // temperature water decimal
        msg_telegram += F("C\n");
        char temp_fault = eep.read((i * 16) + 9); // fault
        if (temp_fault == 0) {msg_telegram += F("Successfull\n");}
        else {msg_telegram += F("***** FAULT *****\n");}
        i--;
        if (i < 0) i = 100;
      }
      Serial.println(msg_telegram);
      myBot.sendMessage(msg, msg_telegram);
    }
    else if (msg.text.equalsIgnoreCase("/Reset"))
    { 
      pump_fault = 0;    
      EEPROM.write(160, pump_fault);
      EEPROM.commit();
      myBot.sendMessage(msg, "Reset fault done !");
    }
    else if (msg.text.equalsIgnoreCase("/Status"))
    { 
    if (pump_fault == 0)
    {
      if (pwm_pump < 255) msg_telegram += F("Status: Releveling\n");
      else if (water_overflow) msg_telegram += F("Status: Overflow\n");
      else if (fan_switched_on) msg_telegram += F("Status: Fan active\n");
      else msg_telegram += F("Status: GOOD !\n");
    }
    else if (pump_fault == 1) msg_telegram += F("Status: FAULT Relay stucked\n");
    else if (pump_fault == 2) msg_telegram += F("Status: FAULT Mosfet in short-circuit\n");
    else if (pump_fault == 3) msg_telegram += F("Status: FAULT Mosfet don't work\n");
    else if (pump_fault == 4) msg_telegram += F("Status: FAULT Relay don't work\n");
    else if (pump_fault == 5) msg_telegram += F("Status: FAULT Maximum pump time reached\n");
    msg_telegram += F("Temperature of the water: ");
    msg_telegram += temperature_water;
    msg_telegram += F("C\n");
    msg_telegram += F("Temperature of the controller: ");
    if (rtc_working)
    {
      msg_telegram += rtc.getTemperature();
      msg_telegram += "C\n";
    }
    else
    {
      msg_telegram += "na\n";
      msg_telegram += "(Check RTC battery)\n";
    }
      myBot.sendMessage(msg, msg_telegram);
    }
    else
    {
      // generate the message for the sender
      msg_telegram = F("Welcome ");
      msg_telegram += msg.sender.username;
      if (pump_fault != 0) msg_telegram += F("\nOsmolator on *** FAULT ***");
      else if (water_overflow) msg_telegram += F("\nOsmolator *** OVERFLOW ***");
      else msg_telegram += F("\nOsmolator working properly");
      msg_telegram += F("\n/Last to see last activations");
      msg_telegram += F("\n/Command to see the commands");
      msg_telegram += F("\n/Reset to reset the fault");
      msg_telegram += F("\n/Status to see the status");
      myBot.sendMessage(msg, msg_telegram);
    }
  } 
}
