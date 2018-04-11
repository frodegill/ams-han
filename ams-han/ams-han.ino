#include <DNSServer.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

static const char* SETUP_SSID = "sensor-setup";
static const byte EEPROM_INITIALIZED_MARKER = 0xF1; //Just a magic number


static const uint8_t SETUP_MODE_PIN = D7;
static const uint8_t HAN_RX_PIN     = D2;

ESP8266WebServer server(80);

DNSServer dnsServer;
static const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);

#define MAX_SSID_LENGTH        (32)
#define MAX_PASSWORD_LENGTH    (64)

char ssid_param[MAX_SSID_LENGTH+1];
char password_param[MAX_PASSWORD_LENGTH+1];
boolean in_setup_mode;

static const int DATABUFFER_LENGTH = 256;
uint8_t data_buffer[DATABUFFER_LENGTH];
volatile int databuffer_pos;
volatile bool buffer_overflow;
volatile unsigned long databuffer_receive_time;
static unsigned long MINIMUM_TIME_BETWEEN_PACKETS = 1000L;


void read_persistent_string(char* s, int max_length, int& adr)
{
  int i = 0;
  byte c;
  do {
    c = EEPROM.read(adr++);
    if (i<max_length)
    {
      s[i++] = static_cast<char>(c);
    }
  } while (c!=0);
  s[i] = 0;
}

void read_persistent_params()
{
  int adr = 0;
  if (EEPROM_INITIALIZED_MARKER != EEPROM.read(adr++))
  {
    ssid_param[0] = 0;
    password_param[0] = 0;
  } else {
    read_persistent_string(ssid_param, MAX_SSID_LENGTH, adr);
    read_persistent_string(password_param, MAX_PASSWORD_LENGTH, adr);
  }
}

void write_persistent_string(const char* s, size_t max_length, int& adr)
{
  for (int i=0; i<min(strlen(s), max_length); i++)
  {
    EEPROM.write(adr++, s[i]);
  }
  EEPROM.write(adr++, 0);
}

void write_persistent_params(const char* ssid, const char* password)
{
  int adr = 0;
  EEPROM.write(adr++, EEPROM_INITIALIZED_MARKER);
  write_persistent_string(ssid, MAX_SSID_LENGTH, adr);
  write_persistent_string(password, MAX_PASSWORD_LENGTH, adr);
  EEPROM.commit();
}

void handleSetupRoot() {
  if (server.hasArg("ssid") || server.hasArg("password"))
  {
    if (server.hasArg("ssid"))
    {
      strncpy(ssid_param, server.arg("ssid").c_str(), MAX_SSID_LENGTH);
      ssid_param[MAX_SSID_LENGTH] = 0;
    }
    
    if (server.hasArg("password") && !server.arg("password").equals(F("password")))
    {
      strncpy(password_param, server.arg("password").c_str(), MAX_PASSWORD_LENGTH);
      password_param[MAX_PASSWORD_LENGTH] = 0;
    }
    
    write_persistent_params(ssid_param, password_param);

    server.send(200, F("text/plain"), F("Settings saved"));
  }
  else
  {
    read_persistent_params();

    String body = F("<!doctype html>"\
                    "<html lang=\"en\">"\
                    "<head>"\
                     "<meta charset=\"utf-8\">"\
                     "<title>Setup</title>"\
                     "<style>"\
                      "form {margin: 0.5em;}"\
                      "input {margin: 0.2em;}"\
                     "</style>"\
                    "</head>"\
                    "<body>"\
                     "<form method=\"post\">"\
                      "SSID:<input type=\"text\" name=\"ssid\" required maxlength=\"");
    body += String(MAX_SSID_LENGTH);
    body += F("\" autofocus value=\"");
    body += ssid_param;
    body += F("\"/><br/>"\
              "Password:<input type=\"password\" name=\"password\" maxlength=\"");
    body += String(MAX_PASSWORD_LENGTH);
    body += F("\" value=\"password\"/><br/>"\
              "<input type=\"submit\" value=\"Submit\"/>"\
              "</form>"\
             "</body>"\
             "</html>");
    server.send(200, F("text/html"), body);
  }
}

void serialEvent() {
  unsigned long now = millis();
  if (now<databuffer_receive_time || (now-databuffer_receive_time)>MINIMUM_TIME_BETWEEN_PACKETS)
  {
    databuffer_pos = 0;
    buffer_overflow = false;
    databuffer_receive_time = now;
  }

  uint8_t b;
  while (Serial.available()) {
    b = Serial.read();
    if ((databuffer_pos+1)<DATABUFFER_LENGTH)
    {
      data_buffer[databuffer_pos++] = b;
    }
    else
    {
      buffer_overflow = true;
    }
  }
}

void handleRequest() {
  String response = "";
  int i;
  for (i=0; i<databuffer_pos; i++)
  {
    switch(i%8)
    {
      case 0: if(i!=0)
              {
                response += "\n";
              }
              response += String(i, 16);
              response += ":";
              break;
      case 4: response += " ";
              break;
      default: break;      
    }
    response += " ";
    response += String(data_buffer[i], 16);
  }

  if (buffer_overflow)
  {
    response += "\n\n(Buffer overflowed)";
  }
  
  server.send(200, F("text/plain"), response);
}

void handleNotFound() {
  server.send(404, F("text/plain"), F("Page Not Found\n"));
}

void setup()
{
  EEPROM.begin(1 + MAX_SSID_LENGTH + 1 + MAX_PASSWORD_LENGTH + 1);

  pinMode(SETUP_MODE_PIN, INPUT_PULLUP);
  if (LOW == digitalRead(SETUP_MODE_PIN))
  {
    in_setup_mode = true;

    WiFi.softAP(SETUP_SSID);
    dnsServer.start(DNS_PORT, "*", apIP);

    server.on("/", handleSetupRoot);
  }
  else
  {
    in_setup_mode = false;
    
    databuffer_pos = 0;
    buffer_overflow = false;
    databuffer_receive_time = 0L;

    Serial.begin(9600, SERIAL_8N1);
    Serial.setDebugOutput(false);

    read_persistent_params();
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid_param, password_param);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }

    server.on("/", handleRequest);
    server.onNotFound(handleNotFound);
  }

  server.begin();
}

void loop()
{
  if (in_setup_mode) {
      dnsServer.processNextRequest();
  }

  server.handleClient();
  delay(100);
}
