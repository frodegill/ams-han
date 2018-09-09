static const uint8_t HAN_RX_PIN     = 3; //RX for Serial. Seems to be GPIO3 on most boards

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

static const SerialConfig SERIAL_MODE = SERIAL_8N1;



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

unsigned int hexToInt(const uint8_t* data_buffer, int databuffer_pos, uint8_t start_pos)
{
  return ((start_pos+4)>databuffer_pos) ? 0 : data_buffer[start_pos+3]<<24 | data_buffer[start_pos+2]<<16 | data_buffer[start_pos+1]<<8 | data_buffer[start_pos];
}

unsigned short hexToShort(const uint8_t* data_buffer, int databuffer_pos, uint8_t start_pos)
{
  return ((start_pos+2)>databuffer_pos) ? 0 : data_buffer[start_pos+1]<<8 | data_buffer[start_pos];
}

void handleRequest() {
  int akkumulert_forbruk = hexToInt(data_buffer, databuffer_pos, 16)/1000.0; //MWh
  int forbruk = hexToInt(data_buffer, databuffer_pos, 48); //W
  short strom_fase1 = hexToShort(data_buffer, databuffer_pos, 64); //mA
  short strom_fase2 = hexToShort(data_buffer, databuffer_pos, 70); //mA
  short strom_fase3 = hexToShort(data_buffer, databuffer_pos, 78); //mA
  float spenning_fase1 = hexToShort(data_buffer, databuffer_pos, 82)/10.0; //V
  float spenning_fase2 = hexToShort(data_buffer, databuffer_pos, 84)/10.0; //V
  float spenning_fase3 = hexToShort(data_buffer, databuffer_pos, 86)/10.0; //V
  float frekvens = hexToShort(data_buffer, databuffer_pos, 94)/100.0; //Hz
}

void setup()
{
  databuffer_pos = 0;
  buffer_overflow = false;
  databuffer_receive_time = 0L;

  Serial.begin(9600, SERIAL_MODE);
  Serial.setDebugOutput(false);
}

void loop()
{
  if (Serial.available())
  {
    serialEvent();
  }
  
  delay(100);
}
