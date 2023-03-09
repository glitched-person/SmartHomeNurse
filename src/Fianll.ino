/*************************Monitor*************************/
#include <SPI.h>
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

#include "flash_chinese.h" // chinese display
/*************************Math*************************/
#include <Average.h>
Average<float> tempArray(100);
Average<float> spo2Array(100);
Average<float> bpmArray(100);
/*************************Pressure*************************/
#include <Robojax_L298N_DC_motor.h>
#include "HX711.h"
/*************************I2C*************************/
#include <Wire.h>
/*************************Temp*************************/
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
/*************************LoRa*************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
/*************************SPO2*************************/
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate;

#define HomeBtn 36
#define YesBtn 39
int YesBtnState = 0, HomeBtnState = 0, Stage = 0;
bool MeasureTempBool = false, MeasureSPO2Bool = false, MeasurePressureBool = false;
int tempMode = 0;
long startTime = 0;
int t = 0, s = 0, intervalSpo2 = 15000;
float up = 0.0 , down = 0.0, reading;

#define CHA 0
#define ENA 13
#define IN1 41
#define IN2 12

/*************************motor2setting*************************/
#define IN3 40
#define IN4 14
#define ENB 27
#define CHB 1

const int CCW = 2; // do not change
const int CW  = 1; // do not change

#define motor1 1 // do not change
#define motor2 2 // do not change

const int LOADCELL_DOUT_PIN = 5;
const int LOADCELL_SCK_PIN = 3;

Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA,  IN3, IN4, ENB, CHB);


HX711 scale;


/*************************LoRa*************************/
static const PROGMEM u1_t NWKSKEY[16] = { 0xAA, 0xE5, 0x18, 0xF4, 0xB3, 0xCD, 0x16, 0x94, 0x8E, 0x30, 0x7B, 0x26, 0xA3, 0xA4, 0x75, 0x72 };
static const u1_t PROGMEM APPSKEY[16] = { 0xCA, 0x25, 0x3E, 0x43, 0x0C, 0x20, 0x16, 0xC6, 0xC3, 0xCE, 0x37, 0x6A, 0xAA, 0xC1, 0x65, 0x6B };
static const u4_t DEVADDR = 0x26041C6A ;
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
static osjob_t sendjob;
const unsigned TX_INTERVAL = 30;
const lmic_pinmap lmic_pins = {
  .nss = 26,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 25,
  .dio = {33, 32, LMIC_UNUSED_PIN},
};
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}
void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    /*uint32_t ID = 3748;
      uint32_t Temp = mlx.readObjectTempC();
      uint32_t BP = 78;
      uint32_t Oxygen = 100;
      uint32_t SysPres = 128;
      uint32_t DiaPres = 101;*/

    uint32_t ID = 3748;
    uint32_t Temp = tempMode;
    uint32_t BP = heartRate;
    uint32_t Oxygen = spo2;
    uint32_t SysPres = up;
    uint32_t DiaPres = down;

    Serial.println("ID: " + String(ID));
    Serial.println("Temperature: " + String(Temp));
    Serial.println("Heart Beat: " + String(BP));
    Serial.println("Blood Oxygen: " + String(Oxygen));
    Serial.println("Sys Pressure: " + String(SysPres));
    Serial.println("Dia Pressure: " + String(DiaPres));

    byte payload[12];
    payload[0] = highByte(ID);
    payload[1] = lowByte(ID);
    payload[2] = highByte(Temp);
    payload[3] = lowByte(Temp);
    payload[4] = highByte(BP);
    payload[5] = lowByte(BP);
    payload[6] = highByte(Oxygen);
    payload[7] = lowByte(Oxygen);
    payload[8] = highByte(SysPres);
    payload[9] = lowByte(SysPres);
    payload[10] = highByte(DiaPres);
    payload[11] = lowByte(DiaPres);
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println(F("Packet queued"));
  }
}

/***************************************************************************************
** Function name:           setup
** Descriptions:            To set things up
***************************************************************************************/
void setup() {
  Serial.begin(115200);
  /*************************mon&btn*************************/
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 10);
  tft.setTextWrap(true);
  tft.setRotation(1);
  tft.setTextSize(5);
  tft.setTextColor(TFT_WHITE);
  //tft.print("HOME NURSE");
  tft.drawBitmap(70, 20, tx_start, 179 , 84, TFT_WHITE);
  tft.drawBitmap(67, 110, tx_measure, 186 , 94, TFT_WHITE);
  pinMode(HomeBtn, INPUT_PULLUP);
  pinMode(YesBtn, INPUT_PULLUP);
  Stage = 0;
  /************************Temp*************************/
  pinMode(22, INPUT_PULLUP);       // Clock goes from master to peripherical
  pinMode(21, INPUT_PULLUP); // Data should be internally pulled up (or 4.7K pullup to 3.3v)
  Wire.begin(21, 22, 10000);
  mlx.begin();
}

/***************************************************************************************
** Function name:           loop
** Descriptions:            Infinite loop
***************************************************************************************/
void loop() {
  YesBtnState = digitalRead(YesBtn);
  HomeBtnState = digitalRead(HomeBtn);
  //Serial.println(YesBtnState);

  if (YesBtnState == 0) {
    if (Stage == 0) {
      MeasureTemp();
    }
    else if (Stage == 1) {
      MeasureSPO2();
    }
    else if (Stage == 2) {
      MeasurePressure();
    }
    else if (Stage == 3) {
      Lora();
    }
    else if (Stage == 4) {
      Home();
    }
  }
  if (HomeBtnState == 0) {
    Home();
  }
}

/***************************************************************************************
** Function name:           MeasureTemp
** Descriptions:            Measure temperature
***************************************************************************************/
void MeasureTemp() {
  Serial.print("Stage:");
  Serial.println(Stage);
  tft.fillScreen(TFT_NAVY);

  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  for (int a = 0; a < 100; a++) {
    tft.setCursor(70, 100);
    Serial.println(mlx.readObjectTempC());
    tft.print(mlx.readObjectTempC(), 1);
    tft.println(F("C"));
    tempArray.push(mlx.readObjectTempC());
    delay(100);
  }

  tempMode = tempArray.mode();
  Serial.print("Mode:   "); Serial.println(tempArray.mode());

  tft.fillScreen(TFT_NAVY);
  tft.drawBitmap(10, 89, tx_temp, 131 , 62, TFT_WHITE);
  tft.drawBitmap(146, 100, tx_colon, 25 , 39, TFT_WHITE);
  tft.setCursor(175, 105);
  tft.print(tempMode);
  delay(1000);
  Stage = 1;
}

/***************************************************************************************
** Function name:           MeasureSPO2
** Descriptions:            Measure SPO2 and Heart rate
***************************************************************************************/
void MeasureSPO2() {
  particleSensor.begin(Wire, I2C_SPEED_FAST);
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  Serial.print("Stage:");
  Serial.println(Stage);
  tft.fillScreen(TFT_DARKCYAN);

  tft.setTextColor(TFT_WHITE, TFT_DARKCYAN);
  tft.setCursor(50, 100);
  tft.drawBitmap(10, 20, tx_measuring, 266 , 88, TFT_WHITE);

  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  // while (1)
  // {
  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data


    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    //send samples and calculation result to terminal program through UART
    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.print(irBuffer[i], DEC);

    Serial.print(F(", HR="));
    Serial.print(heartRate, DEC);

    Serial.print(F(", HRvalid="));
    Serial.print(validHeartRate, DEC);

    Serial.print(F(", SPO2="));
    Serial.print(spo2, DEC);

    Serial.print(F(", SPO2Valid="));
    Serial.println(validSPO2, DEC);
    if (i = 99) {
      i = 100;

      tft.fillScreen(TFT_DARKCYAN);
      tft.drawBitmap(10, 10, tx_average, 129 , 62, TFT_WHITE);
      tft.drawBitmap(144, 10, tx_hr, 126 , 62, TFT_WHITE);
      tft.setCursor(10, 77);
      tft.print(heartRate, DEC);
      tft.drawBitmap(10, 117, tx_average, 129 , 62, TFT_WHITE);
      tft.drawBitmap(144, 117, tx_spo2, 128 , 64, TFT_WHITE);
      tft.setCursor(20, 186);
      tft.print(spo2, DEC);
      //    }
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    delay(1000);
    Stage = 2;
    Serial.print("Stage:");
    Serial.println(Stage);
  }
}

/***************************************************************************************
** Function name:           MeasurePressure
** Descriptions:            Measure blood pressure
***************************************************************************************/
void MeasurePressure() {
  //bloodPres
  robot.begin();
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(2280.f);
  Serial.print("Stage:");
  Serial.println(Stage);
  Serial.print("asdasdsad"); Serial.println(reading);
  reading = scale.read();
  tft.fillScreen(TFT_MAROON);
  tft.setTextColor(TFT_WHITE, TFT_MAROON);
  robot.rotate(motor1, 80, CW);
  robot.rotate(motor2, 70, CW);
  tft.setCursor(50, 100);
  tft.drawBitmap(10, 20, tx_measuring, 266 , 88, TFT_WHITE);

  for (int i = 0; i < 2; i--) {
    float v1 = scale.read() / 5000.0;
    float v2 = v1 - 0.04;
    float kpa = v2 / 0.018;
    float mmhg = 7.5 * kpa;
    float bp_test = mmhg * 0.005;
    float asd = bp_test / random(20, 25);
    up = asd * 0.82;
    down = asd * 0.55;
    if (scale.read() > 7000000) {
      Serial.println("BREAK");
      robot.brake(2);
      delay(20000);
      Serial.print("up:");
      Serial.println(up);

      tft.fillScreen(TFT_MAROON);
      //tft.print(F("Systolic:"));
      tft.drawBitmap(10, 10, tx_sys, 131 , 62, TFT_WHITE);
      tft.setCursor(20, 77);
      tft.println(up);
      Serial.println("down:");
      Serial.println(down);
      //tft.print(F("diastolic:"));
      tft.drawBitmap(10, 117, tx_dia, 129 , 62, TFT_WHITE);
      tft.setCursor(20, 186);
      tft.println(down);
      robot.brake(1);
      delay(5000);
      i = 3;
    }
    else {
      Serial.print("read: \t\t");
      Serial.println(scale.read());
    }
  }
  delay(1000);
  Stage = 3;
  Serial.print("Stage:");
  Serial.println(Stage);
}

/***************************************************************************************
** Function name:           Lora
** Descriptions:            send data to server
***************************************************************************************/
void Lora() {

  tft.fillScreen(TFT_DARKGREEN);
  tft.drawBitmap(74, 10, tx_data, 172 , 88, TFT_WHITE);
  tft.drawBitmap(27, 108, tx_transmitting, 265 , 84, TFT_WHITE);
#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif
  os_init();
  LMIC_reset();
#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  LMIC_setupChannel(0, 923587000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 923587000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 923587000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 923587000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 923587000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 923587000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 923587000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 923587000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 923587000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
#elif defined(CFG_us915)
  LMIC_selectSubBand(1);
#endif
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7, 14);
  //tft.fillScreen(TFT_BLACK);
  //tft.setCursor(70, 100);
  //tft.setTextColor(TFT_WHITE);
  // tft.print(F("LoRa Sending..."));
  Serial.println("LoRa Send");
  do_send(&sendjob);
  delay(1500);
  tft.fillScreen(TFT_DARKGREEN);
  tft.drawBitmap(22, 10, tx_pressto, 276 , 88, TFT_WHITE);
  tft.drawBitmap(70, 103, tx_cont, 181 , 88, TFT_WHITE);
  Stage = 4;
}


/***************************************************************************************
** Function name:           Home
** Descriptions:            back to home page
***************************************************************************************/
void Home() {

  tft.fillScreen(TFT_WHITE);
  tft.drawBitmap(10, 10, tx_finish, 267 , 90, TFT_BLACK);
  tft.drawBitmap(10, 105, tx_reset, 262 , 88, TFT_BLACK);
  delay(2000);
  tft.fillScreen(TFT_BLACK);

  tft.setCursor(10, 100);
  tft.setTextColor(TFT_WHITE);
  //tft.println("HOME NURSE");

  tft.drawBitmap(70, 20, tx_start, 179 , 84, TFT_WHITE);
  tft.drawBitmap(67, 110, tx_measure, 186 , 94, TFT_WHITE);
  Stage = 0;
  delay(1000);
}
