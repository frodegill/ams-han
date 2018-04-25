#include <DNSServer.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

static const char* SETUP_SSID = "sensor-setup";
static const byte EEPROM_INITIALIZED_MARKER = 0xF1; //Just a magic number

static const uint8_t SETUP_MODE_PIN = D7;
static const uint8_t HAN_RX_PIN     = D9;

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

static const uint8_t SLIP_END     = 0xC0;
static const uint8_t SLIP_ESC     = 0xDB;
static const uint8_t SLIP_ESC_END = 0xDC;
static const uint8_t SLIP_ESC_ESC = 0xDD;

static const unsigned CRC16_XMODEM_POLY = 0x1021;

static const SerialConfig SERIAL_MODE = SERIAL_8N1;



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
    if (0==databuffer_pos && SLIP_END==b) //FRAME_END can be sent to force reset of buffer
    {
      continue;
    }
    
    if ((databuffer_pos+1) < DATABUFFER_LENGTH) //Room for one more byte?
    {
      //ESC ESC_END => END  (0xDB 0xDC => 0xC0)
      //ESC ESC_ESC => ESC  (0xDB 0xDD => 0xDB)
      if (databuffer_pos>0 && SLIP_ESC==data_buffer[databuffer_pos-1])
      {
        if (SLIP_ESC_END==b)
        {
          data_buffer[databuffer_pos-1] = SLIP_END;
          continue;
        }
        else if (SLIP_ESC_ESC==b)
        {
          //data_buffer[databuffer_pos-1] = SLIP_ESC; //Already has this value
          continue;
        }
      }

      data_buffer[databuffer_pos++] = b;
    }
    else
    {
      buffer_overflow = true;
    }
  }
}

char* toHex(char* buffer, unsigned int value)
{
  buffer[0]='0';
  buffer[1]='x';
  uint8_t length = value<256?2:4;
  for (uint8_t i=length; i>0; i--)
  {
    buffer[i+1] = ((value&0x000F)<=9?'0':'A'-10)+(value&0x000F);
    value = value >> 4;
  }
  buffer[length+2] = 0;
  return buffer;
}

void dumpHex(const uint8_t* data_buffer, int databuffer_pos, String& response, uint8_t start_pos, uint8_t length)
{
  char tmp_buffer[7];
  if(start_pos!=0)
  {
    response += "\n";
  }
  response.concat(toHex(tmp_buffer, start_pos));
  response += ":";
  for (int i=0; i<length && (start_pos+i)<databuffer_pos; i++)
  {
    response += " ";
    response.concat(toHex(tmp_buffer, data_buffer[start_pos+i]));
  }
}

unsigned int hexToInt(const uint8_t* data_buffer, int databuffer_pos, uint8_t start_pos)
{
  return ((start_pos+4)>databuffer_pos) ? 0 : data_buffer[start_pos+3]<<24 | data_buffer[start_pos+2]<<16 | data_buffer[start_pos+1]<<8 | data_buffer[start_pos];
}

unsigned short hexToShort(const uint8_t* data_buffer, int databuffer_pos, uint8_t start_pos)
{
  return ((start_pos+2)>databuffer_pos) ? 0 : data_buffer[start_pos+1]<<8 | data_buffer[start_pos];
}

boolean validCrc16(const uint8_t* data_buffer, int databuffer_pos, uint8_t content_start_pos, uint8_t content_length, uint8_t crc_start_pos)
{
  if ((content_start_pos+content_length)>databuffer_pos || (crc_start_pos+2)>databuffer_pos)
  {
    return false;
  }
  
  unsigned crc = 0;
  for (uint8_t i=0; i<content_length; i++)
  {
      crc ^= ((unsigned)data_buffer[content_start_pos+i]) << 8;
      for (uint8_t j=0; j<8; j++)
      {
        crc = crc&0x8000 ? (crc<<1)^CRC16_XMODEM_POLY : crc<<1;
      }
  }
  unsigned short actual_crc = hexToShort(data_buffer, databuffer_pos, crc_start_pos);
  return crc==actual_crc;
}

void handleRequest() {

  String response;
  dumpHex(data_buffer, databuffer_pos, response,  0, 16);
  response.concat(F("\tMålernummer: \""));
  for (uint8_t i=0; i<16; i++)
  {
    response.concat((char)data_buffer[i]);
  }
  response.concat(F("\""));

  dumpHex(data_buffer, databuffer_pos, response, 16,  4);
  response.concat(F("\tAkkumulert forbruk: "));
  response.concat(hexToInt(data_buffer, databuffer_pos, 16)/1000.0);
  response.concat(F("MWh"));
 
  dumpHex(data_buffer, databuffer_pos, response, 20, 28);
  
  dumpHex(data_buffer, databuffer_pos, response, 48,  4);
  response.concat(F("\tForbruk: "));
  response.concat(hexToInt(data_buffer, databuffer_pos, 48));
  response.concat(F("W"));

  dumpHex(data_buffer, databuffer_pos, response, 52, 12);

  dumpHex(data_buffer, databuffer_pos, response, 64,  2);
  response.concat(F("\tStrøm fase 1: "));
  response.concat(hexToShort(data_buffer, databuffer_pos, 64));
  response.concat(F("mA"));

  dumpHex(data_buffer, databuffer_pos, response, 66,  4);

  dumpHex(data_buffer, databuffer_pos, response, 70,  2);
  response.concat(F("\tStrøm fase 2: "));
  response.concat(hexToShort(data_buffer, databuffer_pos, 70));
  response.concat(F("mA"));

  dumpHex(data_buffer, databuffer_pos, response, 72,  6);

  dumpHex(data_buffer, databuffer_pos, response, 78,  2);
  response.concat(F("\tStrøm fase 3: "));
  response.concat(hexToShort(data_buffer, databuffer_pos, 78));
  response.concat(F("mA"));

  dumpHex(data_buffer, databuffer_pos, response, 80,  2);

  dumpHex(data_buffer, databuffer_pos, response, 82,  2);
  response.concat(F("\tSpenning fase 1: "));
  response.concat(hexToShort(data_buffer, databuffer_pos, 82)/10.0);
  response.concat(F("V"));

  dumpHex(data_buffer, databuffer_pos, response, 84,  2);
  response.concat(F("\tSpenning fase 2: "));
  response.concat(hexToShort(data_buffer, databuffer_pos, 84)/10.0);
  response.concat(F("V"));

  dumpHex(data_buffer, databuffer_pos, response, 86,  2);
  response.concat(F("\tSpenning fase 3: "));
  response.concat(hexToShort(data_buffer, databuffer_pos, 86)/10.0);
  response.concat(F("V"));

  dumpHex(data_buffer, databuffer_pos, response, 88,  6);

  dumpHex(data_buffer, databuffer_pos, response, 94,  2);
  response.concat(F("\tFrekvens: "));
  response.concat(hexToShort(data_buffer, databuffer_pos, 94)/100.0);
  response.concat(F("Hz"));

  dumpHex(data_buffer, databuffer_pos, response, 96,  1);

  dumpHex(data_buffer, databuffer_pos, response, 97,  2);
  response.concat(F("\tSjekksum er "));
  response.concat(validCrc16(data_buffer, databuffer_pos, 0, 97, 97) ? F("OK") : F("IKKE OK"));
  
  dumpHex(data_buffer, databuffer_pos, response, 99,  1);

  if (buffer_overflow)
  {
    response.concat(F("\n\n(Buffer overflowed)"));
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

    Serial.begin(9600, SERIAL_MODE);
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

  if (Serial.available())
  {
    serialEvent();
  }
  
  delay(100);
}
