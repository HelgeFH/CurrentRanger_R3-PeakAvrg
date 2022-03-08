// CurrentRanger(TM) stock firmware
// https://lowpowerlab.com/CurrentRanger
// CurrentRanger is a *high-side* precision current meter featuring:
//   - fast autoranging
//   - uni/bi-directional modes (ie. DC/AC measurements)
//   - ultra low burden voltage
//   - 1mV per nA/uA/mA measurements with DMM/scope
//   - OLED standalone readings
//   - serial data logging option via 3.3v/RX/TX header or USB (must use isolation, read guide!)
//   - full digital control for power/switching
//   - LiPo powered with auto power-off feature (0.6uA quiescent current)
// *************************************************************************************************************
#ifndef CURRENT_RANGER
#error CurrentRanger target board required, see guide on how to add it to the IDE: lowpowerlab.com/currentranger
#endif
//***********************************************************************************************************
#include "src/flstore/FlashStorage.h"      //for emulated EEPROM - https://github.com/cmaglie/FlashStorage
#include "src/ftouch/Adafruit_FreeTouch.h" //https://github.com/adafruit/Adafruit_FreeTouch
#include "src/u8g2/U8g2lib.h"              //https://github.com/olikraus/u8g2/wiki/u8g2reference fonts:https://github.com/olikraus/u8g2/wiki/fntlistall
//#include <ATSAMD21_ADC.h>

// CurrentRanger Firmware Version
#define FW_VERSION "1.3.0"

//***********************************************************************************************************
#define BIAS_LED 11
#define LPFPIN   4
#define LPFLED   LED_BUILTIN
#define AUTOFF   PIN_AUTO_OFF
//***********************************************************************************************************
#define MA_PIN           PIN_PA13 //#define MA  38
#define UA_PIN           PIN_PA14 //#define UA  2
#define NA_PIN           PIN_PA15 //#define NA  5
#define MA_GPIO_PIN      PIN_PB11
#define UA_GPIO_PIN      PIN_PA12
#define NA_GPIO_PIN      PIN_PB10
#define PINOP(pin, OP)   (PORT->Group[(pin) / 32].OP.reg = (1 << ((pin) % 32)))
#define PIN_OFF(THE_PIN) PINOP(THE_PIN, OUTCLR)
#define PIN_ON(THE_PIN)  PINOP(THE_PIN, OUTSET)
#define PIN_TGL(THE_PIN) PINOP(THE_PIN, OUTTGL)
//***********************************************************************************************************
#define SENSE_IOGND            0
#define SENSE_OUTPUT           A3
#define SENSE_GNDISO           A2
#define SENSE_VIN              A5
#define ADC_PRESCALER          ADC_CTRLB_PRESCALER_DIV16
#define ADC_SAMPCTRL           0b111 //sample timing [fast 0..0b111 slow]
#define ADCFULLRANGE           32767.0
#define VBAT_REFRESH_INTERVAL  1000 //ms
#define LOBAT_THRESHOLD        3.40 //volts
#define DAC_GND_ISO_OFFSET     10
#define DAC_HALF_SUPPLY_OFFSET 512
#define DAC_GND_VDD_OFFSET     1014
#define OUTPUT_CALIB_FACTOR    1.00 //calibrate final VOUT value
//***********************************************************************************************************
#define ADC_CALIBRATE_FORCED_OFFSET 0
#define ADC_CALIBRATE_FORCED_GAIN   1.0
#define LDO_DEFAULT                 3.300 //volts, change to actual LDO output (measure GND-3V on OLED header)
//***********************************************************************************************************
#define BUZZER    1       // BUZZER pin
#define NOTE_OFF  0       // this works for tune()
#define NOTE_C7   2093    // 2093.00 ->  0.0%
#define NOTE_Cs7  2217    // 2217.46 -> −0.0207%
#define NOTE_D7   2349    // 2349.32 -> −0.0136%
#define NOTE_Ds7  2489    // 2489.02 -> −0.0008%
#define NOTE_E7   2637    // 2637.02 -> −0.0008%
#define NOTE_F7   2794    // 2793.83 ->  0.0061%
#define NOTE_Fs7  2960    // 2959.96 ->  0.0014%
#define NOTE_G7   3136    // 3135.96 ->  0.0013%
#define NOTE_Gs7  3322    // 3322.44 -> −0.0132%
#define NOTE_A7   3520    // 3520.00 ->  0.0%
#define NOTE_As7  3729    // 3729.31 -> −0.0083%
#define NOTE_B7   3951    // 3951.07 -> −0.0018%
#define NOTE_C8   4186    // 4186.01 -> −0.0002%
#define NOTE_Cs8  4435    // 4434.92 ->  0.0018%
#define NOTE_D8   4699    // 4698.63 ->  0.0079%
#define NOTE_Ds8  4978    // 4978.03 -> −0.0006%
#define NOTE_E8   5274    // 5274.04 -> −0.0008%
#define NOTE_F8   5588    // 5587.65 ->  0.0063%
#define NOTE_Fs8  5920    // 5919.91 ->  0.0015%
#define NOTE_G8   6272    // 6271.93 ->  0.0011%
#define NOTE_Gs8  6645    // 6644.88 ->  0.0018%
#define NOTE_A8   7040    // 7040.00 ->  0.0%
#define NOTE_As8  7459    // 7458.62 ->  0.0051%
#define NOTE_B8   7902    // 7902.13 -> −0.0016%
#define NOTE_C9   8372    // 8372.02 -> −0.0002%
#define NOTE_Cs9  8870    // 8869.84 ->  0.0018%
#define NOTE_D9   9397    // 9397.26 -> −0.0028%
#define TONE_BEEP NOTE_C8 // default buzz frequency
//***********************************************************************************************************
#define MODE_MANUAL    0
#define MODE_AUTORANGE 1
#define STARTUP_MODE   MODE_MANUAL //or: MODE_AUTORANGE
//***********************************************************************************************************
#include <Wire.h>                     //i2c scanner: https://playground.arduino.cc/Main/I2cScanner
#define OLED_BAUD             1600000 //fast i2c clock
#define OLED_ADDRESS          0x3C    //i2c address on most small OLEDs
#define OLED_REFRESH_INTERVAL 500     //ms
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
//***********************************************************************************************************
#define TOUCH_N 8
#define TOUCH_U 9
#define TOUCH_M A4
Adafruit_FreeTouch qt[3] = {
  Adafruit_FreeTouch(TOUCH_N, OVERSAMPLE_1, RESISTOR_50K, FREQ_MODE_NONE),
  Adafruit_FreeTouch(TOUCH_U, OVERSAMPLE_1, RESISTOR_50K, FREQ_MODE_NONE),
  Adafruit_FreeTouch(TOUCH_M, OVERSAMPLE_1, RESISTOR_50K, FREQ_MODE_NONE),
};
#define TOUCH_HIGH_THRESHOLD  400 //range is 0..1023
#define TOUCH_SAMPLE_INTERVAL 50  //ms
//***********************************************************************************************************
#define SERIAL_UART_BAUD 230400 //Serial baud for HC-06/bluetooth output
#define BT_SERIAL_EN
#define BT_REFRESH_INTERVAL 200 //ms
//***********************************************************************************************************
#define AUTOOFF_BUZZ_DELAY 500    //ms
#define AUTOOFF_DEFAULT    600    //seconds, turn unit off after 10min of inactivity
#define AUTOOFF_DISABLED   0xFFFF // do not turn off
#define AUTOOFF_SMART      0xFFFE // turn off only if there is no BT or USB data logging
//***********************************************************************************************************
#define LOGGING_FORMAT_EXPONENT 0 //ex: 123E-3 = 123mA
#define LOGGING_FORMAT_NANOS    1 //ex: 1234 = 1.234uA = 0.001234mA
#define LOGGING_FORMAT_MICROS   2 //ex: 1234 = 1.234mA = 1234000nA
#define LOGGING_FORMAT_MILLIS   3 //ex: 1234 = 1.234A = 1234000uA = 1234000000nA
#define LOGGING_FORMAT_ADC      4 //raw register output vAdc and corrected vCor
#define LOGGING_FORMAT_DISPLAY  5 //range, oledVal, voutVal
//***********************************************************************************************************
#define ADC_SAMPLING_SPEED_AVG  0
#define ADC_SAMPLING_SPEED_FAST 1
#define ADC_SAMPLING_SPEED_SLOW 2
//***********************************************************************************************************

int ofsUniCorrectionValue = 0;
int ofsBiaCorrectionValue = 0;
float gainCorrectionValue = 0;
float ldoValue = 0, ldoOptimized = 0;
uint16_t autooff_interval   = 0;
uint8_t USB_LOGGING_ENABLED = false;
uint8_t TOUCH_DEBUG_ENABLED = false;
uint8_t GPIO_HEADER_RANGING = false;
uint8_t BT_LOGGING_ENABLED  = true;
uint8_t LOGGING_FORMAT      = LOGGING_FORMAT_EXPONENT;
uint16_t ADC_SAMPLING_SPEED = ADC_SAMPLING_SPEED_SLOW;
uint32_t ADC_AVGCTRL;
uint8_t OLED_found    = false;
uint8_t autoffWarning = false;
uint8_t autoffBuzz    = 0;
int8_t tpAll          = 0;

#ifdef BT_SERIAL_EN
uint8_t BT_found = false;
#endif
FlashStorage(eeprom_ADCofsUni, int);
FlashStorage(eeprom_ADCofsBia, int);
FlashStorage(eeprom_ADCgain, float);
FlashStorage(eeprom_LDO, float);
//FlashStorage(eeprom_AUTOFF, uint16_t);
FlashStorage(eeprom_LOGGINGFORMAT, uint8_t);
//FlashStorage(eeprom_ADCSAMPLINGSPEED, uint8_t);

//***********************************************************************************************************
void setup() {
  /*
    //some buzz
    tone(BUZZER, NOTE_C7); delay(100);
    tone(BUZZER, NOTE_E7); delay(100);
    tone(BUZZER, NOTE_G7); delay(100);
    tone(BUZZER, NOTE_C8); delay(200);
    noTone(BUZZER);        delay(50);
    tone(BUZZER, NOTE_G7); delay(100);
    tone(BUZZER, NOTE_C8); delay(400);
    noTone(BUZZER);

    // range-test
    for (float f = 2200; f < 8800; f *= 1.059463094) {
      tone(BUZZER, f + 0.5);
      delay(200);
    }
    noTone(BUZZER);
  */

  delay(50); //Wire apparently needs this
  Wire.begin();
  Wire.beginTransmission(OLED_ADDRESS);
  uint8_t error = Wire.endTransmission();
  if (error == 0) {
    Serial.print("OLED FOUND at 0x");
    Serial.println(OLED_ADDRESS);
    u8g2.begin();
    //u8g2.setDisplayRotation(U8G2_R2); //if required (inside/custom mount?)
    u8g2.setBusClock(OLED_BAUD);
    OLED_found = true;
  } else
    Serial.println("NO OLED found...");

  pinMode(A0, OUTPUT); //DAC/GNDISO
  //DAC->CTRLB.bit.EOEN = 0x00; //enable high drive strength - already done in wiring.c

  pinMode(SENSE_OUTPUT, INPUT);
  pinMode(SENSE_GNDISO, INPUT); //GND-ISO
  pinMode(SENSE_VIN, INPUT);    //VIN > 1MEG > SENSE_VIN > 2MEG > GND
  pinMode(AUTOFF, INPUT_PULLUP);
  pinMode(BIAS_LED, OUTPUT);
  pinMode(LPFLED, OUTPUT); //STATUS/LPF-LED
  pinMode(LPFPIN, OUTPUT); //LPF control pin
  pinMode(BUZZER, OUTPUT);
  PINOP(MA_PIN, DIRSET);
  PINOP(UA_PIN, DIRSET);
  PINOP(NA_PIN, DIRSET);
  PINOP(MA_GPIO_PIN, DIRSET);
  PINOP(UA_GPIO_PIN, DIRSET);
  PINOP(NA_GPIO_PIN, DIRSET);

  qt[0].begin();
  qt[1].begin();
  qt[2].begin();             //touch pads
  analogWriteResolution(10); //DAC resolution

  //DAC->CTRLA.bit.RUNSTDBY = 0x01;delay(1);
  //DAC->CTRLB.bit.REFSEL=0;//pick internal reference, skip SYNCDAC (done by analogWrite)
  analogWrite(A0, DAC_GND_ISO_OFFSET); // Initialize Dac to OFFSET

  //autooff_interval = eeprom_AUTOFF.read();
  if (autooff_interval == 0) {
    autooff_interval = AUTOOFF_SMART;
    //eeprom_AUTOFF.write(autooff_interval);
  }

  LOGGING_FORMAT = eeprom_LOGGINGFORMAT.read();

  ofsUniCorrectionValue = eeprom_ADCofsUni.read();
  ofsBiaCorrectionValue = eeprom_ADCofsBia.read();
  gainCorrectionValue   = eeprom_ADCgain.read();
  ldoValue              = eeprom_LDO.read();

  //ADC_SAMPLING_SPEED = eeprom_ADCSAMPLINGSPEED.read();
  refreshADCSamplingSpeed(); //load correct value into ADC_AVGCTRL

  if (ldoValue == 0) { //check if anything saved in EEPROM
    saveLDO(LDO_DEFAULT);
    ofsUniCorrectionValue = ADC_CALIBRATE_FORCED_OFFSET;
    ofsBiaCorrectionValue = ADC_CALIBRATE_FORCED_OFFSET;
    gainCorrectionValue   = ADC_CALIBRATE_FORCED_GAIN;
    eeprom_ADCofsUni.write(ofsUniCorrectionValue);
    eeprom_ADCofsBia.write(ofsBiaCorrectionValue);
    eeprom_ADCgain.write(gainCorrectionValue);
  }
  ldoOptimizeRefresh();

  // check all touched for entering bootloader
  for (int i = 0; i < 10; i++) {
    tpAll = 0;
    while (!tpAll)
      handleTouchPads();
  }
  if (tpAll == 1)
    rebootIntoBootloader();

  if (OLED_found) {
    bootScreen();
#if 1
    // buzzbadinerie
    const uint16_t notes[] = {
      NOTE_B8, 8, NOTE_D9, 4, NOTE_B8, 4,
      NOTE_Fs8, 8, NOTE_B8, 4, NOTE_Fs8, 4, NOTE_D8, 8, NOTE_Fs8, 4, NOTE_D8, 4,
      NOTE_B7, 16, NOTE_Fs7, 4, NOTE_B7, 4, NOTE_D8, 4, NOTE_B7, 4,
      NOTE_Cs8, 4, NOTE_B7, 4, NOTE_Cs8, 4, NOTE_B7, 4, NOTE_As7, 4, NOTE_Cs8, 4, NOTE_E8, 4, NOTE_Cs8, 4,
      NOTE_D8, 8, NOTE_B7, 8,
      NOTE_OFF, 0
    };

    for (unsigned int i = 0; i < (sizeof(notes) / sizeof(notes[0])); i += 2) {
      float trsp = 0.5;
      float frq  = notes[0 + i] * trsp;
      int len    = notes[1 + i];
      tone(BUZZER, frq + 0.5, len * 23);
      delay(len * 32);
    }
#else
    delay(2000);
#endif
  }

#ifdef BT_SERIAL_EN
  //BT check
  Serial.print("Bluetooth AT check @");
  Serial.print(SERIAL_UART_BAUD);
  Serial.print("baud...");
  delay(600);
  SerialBT.begin(SERIAL_UART_BAUD);
  SerialBT.print("AT"); //assuming HC-06, no line ending required
  uint32_t timer = millis();
  while (millis() - timer < 1000) //about 1s to respond
  {
    if (SerialBT.available() == 2 && SerialBT.read() == 'O' && SerialBT.read() == 'K') {
      BT_found = true;
      break;
    }
  }

  Serial.print(BT_found ? "OK!" : "No HC-06 response.\r\nChecking for BT v3.0...");

  if (!BT_found) {
    SerialBT.print("\r\n"); //assuming HC-06 version 3.0 that requires line ending
    uint32_t timer = millis();
    while (millis() - timer < 50) //about 50ms to respond
    {
      if (SerialBT.available() == 4 && SerialBT.read() == 'O' && SerialBT.read() == 'K' && SerialBT.read() == '\r' && SerialBT.read() == '\n') {
        BT_found = true;
        break;
      }
    }

    Serial.println(BT_found ? "OK!" : "No response.");
  }

  BT_LOGGING_ENABLED = BT_found;
#endif

  printSerialMenu();
  WDTset();
  rangeMA();
  if (STARTUP_MODE == MODE_AUTORANGE)
    toggleAUTORANGE();
  toggleLPF();
}

uint32_t lpfInterval = 0, offsetInterval = 0, autorangeInterval = 0, btInterval = 0,
         autoOffBuzzInterval = 0, autoOffRemain = AUTOOFF_DEFAULT,
         touchSampleInterval = 0, lastKeepAlive = 0, vBatInterval = 0, oledInterval = 0;
bool LPF = 0, BIAS = 0, AUTORANGE = 0;

uint8_t extraMode = 0; // AUTO extra mode, 0=normal, 1=PEAK, 2=AVRG-I, 3=AVRG-C

uint8_t adcRange; // 0=nA, 1=uA, 2=mA
float vBat = 0, vAdc, vCor;

float voutVal = 0, oledVal = 0, oledSum = 0; // total in nA
float voutSum[3], voutCnt[3];                // cumulated by adcRange

uint32_t SampleTime = 0;

float voutPkhEx1 = 0, voutSumEx2 = 0, voutCntEx2 = 0;
uint32_t ARmillis = 0, Ex2millis = 0;

uint8_t avgdiscard = 0;
bool showBT        = false;
#define IS_RANGE_MA (adcRange == 2)
#define IS_RANGE_UA (adcRange == 1)
#define IS_RANGE_NA (adcRange == 0)

void loop() {
  // static uint32_t timestamp = 0, oldstamp;
  // oldstamp  = timestamp;
  // timestamp = micros();

  static int voutcnt = 0, oledcnt = 0;

  // collect VOUT once
  vAdc     = adcRead(SENSE_OUTPUT, SENSE_GNDISO);
  vCor     = vAdc + (BIAS ? ofsBiaCorrectionValue : ofsUniCorrectionValue) / 10.;
  float fv = ldoOptimized * vCor * gainCorrectionValue, voutval;
  SampleTime = millis();
  // do some averaging for readability
  voutSum[adcRange] += fv;
  voutCnt[adcRange]++;

  if (++voutcnt < 8)
    return;

  voutcnt = 0;
  voutval = 0;

  for (int i = 2; i >= 0; i--) {
    voutval *= 1000;
    if (voutCnt[i])
      voutval += voutSum[i] / voutCnt[i];
    voutSum[i] = 0;
    voutCnt[i] = 0;
  }

  if (avgdiscard) {
    avgdiscard--;
    return;
  }

  // arriving here approx. every 8..12ms depending on logging etc.

  voutVal = voutval;
  oledSum += voutVal;
  oledcnt++;

  // extramode-1 PEAK hold
  if (abs(voutPkhEx1) < abs(voutVal)) {
    voutPkhEx1 = voutVal;
  }

  // battery voltage
  if (millis() >= vBatInterval) {
    vBatInterval = millis() + VBAT_REFRESH_INTERVAL;

    vBat = adcRead(SENSE_VIN, SENSE_IOGND);
    vBat = ((vBat / ADCFULLRANGE) * ldoValue) * 1.5;
    //1.5 given by vBat->A5 resistor ratio (1 / (2M * 1/(1M+2M)))
  }

  // serial line input
  while (Serial.available() > 0) {
    char inByte = Serial.read();
    handleSerialIn(inByte);
  }

  //serial line logging output (consumes time)
  if (USB_LOGGING_ENABLED) {
    String logline;
    if (handleLog(&logline))
      Serial.println(logline.c_str());
  }

  // BT serial logging (consumes time)
  showBT = false;
#ifdef BT_SERIAL_EN
  if (BT_LOGGING_ENABLED) {
    showBT = true;
    String logline;
    if (handleLog(&logline))
      SerialBT.println(logline.c_str());
  }
#endif

  // adjust ADC range, this is separate from autorange-display
  float sv = voutVal;
  for (int i = adcRange; i; i--)
    sv *= 0.001;
  if (AUTORANGE && (ARmillis < millis())) {
    if (BIAS) {
      if (sv < -1400. || sv > 1400.) {
        // range up
        if (IS_RANGE_NA) {
          rangeUA();
        } else if (IS_RANGE_UA) {
          rangeMA();
        }
      } else if (sv > -1.3 && sv < 1.3) {
        // range down
        if (IS_RANGE_MA) {
          rangeUA();
        } else if (IS_RANGE_UA) {
          rangeNA();
        }
      }
    } else {
      if (sv > 2800.) {
        // range up
        if (IS_RANGE_NA) {
          rangeUA();
        } else if (IS_RANGE_UA) {
          rangeMA();
        }
      } else if (sv < 2.6) {
        // range down
        if (IS_RANGE_MA) {
          rangeUA();
        } else if (IS_RANGE_UA) {
          rangeNA();
        }
      }
    }
  }

  // prepare OLED values
  if (millis() >= oledInterval) {
    oledInterval = millis() + OLED_REFRESH_INTERVAL;

    oledVal = oledSum / oledcnt;
    oledSum = 0;
    oledcnt = 0;

    // extramode-2+3 AVRG long term averaging
    voutSumEx2 += oledVal;
    voutCntEx2++;

    handleOLED();
  }

  WDTclear();
  handleTouchPads(); //~112uS
  handleAutoOff();

  // String dbug = "T=" + String(timestamp - oldstamp) + " t=" + String(micros() - timestamp);
  // Serial.println(dbug.c_str());
} //loop()

void handleSerialIn(char inByte) {
  static String inLine = "";
  String txt;
  int nxt, idx, ofs = 0;

  // tickle the AUTOOFF function so it doesn't shut down when there are commands coming over serial
  lastKeepAlive = millis();
  //    avgCnt        = 0;
  //    voutAvg       = 0;

  switch (inByte) {
    case '[':
      gainCorrectionValue -= 0.002;
    // fall through
    case ']':
      gainCorrectionValue += 0.001;
      eeprom_ADCgain.write(gainCorrectionValue);
      Serial.print("new gainCorrectionValue = ");
      Serial.println(gainCorrectionValue, 3);
      break;
    case '{':
      ofs = -2;
    // fall through
    case '}':
      ofs++;
      if (BIAS) {
        ofsBiaCorrectionValue += ofs;
        eeprom_ADCofsBia.write(ofsBiaCorrectionValue);
        Serial.print("new ofsBiaCorrectionValue = ");
        Serial.println(ofsBiaCorrectionValue);
      } else {
        ofsUniCorrectionValue += ofs;
        eeprom_ADCofsUni.write(ofsUniCorrectionValue);
        Serial.print("new ofsUniCorrectionValue = ");
        Serial.println(ofsUniCorrectionValue);
      }
      break;
    case '<':
      ldoValue -= 0.002;
    // fall through
    case '>':
      ldoValue += 0.001;
      saveLDO(ldoValue);
      Serial.print("new LDO_Value = ");
      Serial.println(ldoValue, 3);
      break;
    case 'r': //reboot to bootloader
      Serial.print("\nRebooting to bootloader.");
      for (uint8_t i = 0; i++ < 30;) {
        delay(10);
        Serial.print('.');
      }
      rebootIntoBootloader();
      break;
    case 'u': //toggle USB logging
      USB_LOGGING_ENABLED = !USB_LOGGING_ENABLED;
      Serial.println(USB_LOGGING_ENABLED ? "USB_LOGGING_ENABLED" : "USB_LOGGING_DISABLED");
      break;
    case 't': //toggle touchpad serial output debug info
      TOUCH_DEBUG_ENABLED = !TOUCH_DEBUG_ENABLED;
      Serial.println(TOUCH_DEBUG_ENABLED ? "TOUCH_DEBUG_ENABLED" : "TOUCH_DEBUG_DISABLED");
      break;
    case 'g': //toggle GPIOs indicating ranging
      GPIO_HEADER_RANGING = !GPIO_HEADER_RANGING;
      if (GPIO_HEADER_RANGING) {
        if (IS_RANGE_MA)
          PIN_ON(MA_GPIO_PIN);
        else
          PIN_OFF(MA_GPIO_PIN);
        if (IS_RANGE_UA)
          PIN_ON(UA_GPIO_PIN);
        else
          PIN_OFF(UA_GPIO_PIN);
        if (IS_RANGE_NA)
          PIN_ON(NA_GPIO_PIN);
        else
          PIN_OFF(NA_GPIO_PIN);
      }
      Serial.println(GPIO_HEADER_RANGING ? "GPIO_HEADER_RANGING_ENABLED" : "GPIO_HEADER_RANGING_DISABLED");
      break;
    case 'i': // toggle BIAS setting
      toggleBIAS();
      Serial.println(BIAS ? "BIAS On" : "BIAS Off");
      break;
    case 'b': //toggle BT/serial logging
#ifdef BT_SERIAL_EN
      if (BT_found) {
        BT_LOGGING_ENABLED = !BT_LOGGING_ENABLED;
        Serial.println(BT_LOGGING_ENABLED ? "BT_LOGGING_ENABLED" : "BT_LOGGING_DISABLED");
      } else {
        BT_LOGGING_ENABLED = false;
        Serial.println("BT Module not found: cannot enable logging");
      }
#else
      Serial.println("BT_LOGGING Not Enabled");
#endif
      break;
    case 'f': //cycle through output logging formats
      if (++LOGGING_FORMAT > LOGGING_FORMAT_DISPLAY)
        LOGGING_FORMAT = LOGGING_FORMAT_EXPONENT;
      eeprom_LOGGINGFORMAT.write(LOGGING_FORMAT);
      if (LOGGING_FORMAT == LOGGING_FORMAT_EXPONENT)
        Serial.println("LOGGING_FORMAT_EXPONENT");
      else if (LOGGING_FORMAT == LOGGING_FORMAT_NANOS)
        Serial.println("LOGGING_FORMAT_NANOS");
      else if (LOGGING_FORMAT == LOGGING_FORMAT_MICROS)
        Serial.println("LOGGING_FORMAT_MICROS");
      else if (LOGGING_FORMAT == LOGGING_FORMAT_MILLIS)
        Serial.println("LOGGING_FORMAT_MILLIS");
      else if (LOGGING_FORMAT == LOGGING_FORMAT_ADC)
        Serial.println("LOGGING_FORMAT_ADC");
      else if (LOGGING_FORMAT == LOGGING_FORMAT_DISPLAY)
        Serial.println("LOGGING_FORMAT_DISPLAY");
      break;
    case 's':
      if (++ADC_SAMPLING_SPEED > ADC_SAMPLING_SPEED_SLOW)
        ADC_SAMPLING_SPEED = ADC_SAMPLING_SPEED_AVG;
      if (ADC_SAMPLING_SPEED == ADC_SAMPLING_SPEED_AVG)
        Serial.println("ADC_SAMPLING_SPEED_AVG");
      else if (ADC_SAMPLING_SPEED == ADC_SAMPLING_SPEED_FAST)
        Serial.println("ADC_SAMPLING_SPEED_FAST");
      else if (ADC_SAMPLING_SPEED == ADC_SAMPLING_SPEED_SLOW)
        Serial.println("ADC_SAMPLING_SPEED_SLOW");
      //eeprom_ADCSAMPLINGSPEED.write(ADC_SAMPLING_SPEED);
      refreshADCSamplingSpeed();
      break;
    case 'a': //toggle autoOff function
      if (autooff_interval == AUTOOFF_DEFAULT) {
        Serial.println("AUTOOFF_DISABLED");
        autooff_interval = AUTOOFF_DISABLED;
      } else if (autooff_interval == AUTOOFF_SMART) {
        Serial.println("AUTOOFF_DEFAULT");
        autooff_interval = AUTOOFF_DEFAULT;
      } else {
        // turn off only when there is no serial or BT data logging
        Serial.println("AUTOOFF_SMART");
        autooff_interval = AUTOOFF_SMART;
      }
      //eeprom_AUTOFF.write(autooff_interval);
      break;
    case '?':
      printSerialMenu();
      break;
    case 'v': // sample raw ADC voltages
      uint32_t mus, atm;
      int16_t adc;
      mus = micros();
      adc = adcRead(SENSE_OUTPUT, SENSE_GNDISO);
      atm = micros() - mus;
      txt = "adcR " + String(atm) + "us " + String(adc);
      Serial.println(txt.c_str());
      mus = micros();
      adc = adcRead(SENSE_VIN, SENSE_IOGND);
      atm = micros() - mus;
      txt = "adcB " + String(atm) + "us " + String(adc);
      Serial.println(txt.c_str());
      txt = "vBat " + String(vBat, 2);
      Serial.println(txt.c_str());
      break;
    case 'c': // print replayable configuration data
      txt = ":" + String(ofsUniCorrectionValue) +
            ":" + String(ofsBiaCorrectionValue) +
            ":" + String(gainCorrectionValue, 3) +
            ":" + String(ldoValue, 3) +
            ":0:0"; // two spares
      Serial.println(txt.c_str());
      break;
    case '\r':
    case '\n': // cr|lf check for calibration data string
      Serial.println(inLine.c_str());
      nxt = idx = 0;
      while (1) {
        int res;
        float stf;

        res = inLine.indexOf(':', nxt);
        if (res < 0)
          break;
        nxt = res + 1;
        stf = inLine.substring(nxt).toFloat();
        switch (idx++) {
          case 0: // offset uni
            ofsUniCorrectionValue = (int)stf;
            eeprom_ADCofsUni.write(ofsUniCorrectionValue);
            break;
          case 1: // offset bias
            ofsBiaCorrectionValue = (int)stf;
            eeprom_ADCofsBia.write(ofsBiaCorrectionValue);
            break;
          case 2: // gain
            gainCorrectionValue = stf;
            eeprom_ADCgain.write(gainCorrectionValue);
            break;
          case 3: // LDO-voltage
            saveLDO(stf);
            break;
        }
      }
      if (idx)
        printCalibInfo();
      inLine = "";
      break;

    default:
      inLine += String(inByte);
      if (inLine.length() > 80)
        inLine = "";
      break;
  }
}

bool handleLog(String *logline) {
  bool retflg = true;

  if (LOGGING_FORMAT == LOGGING_FORMAT_EXPONENT) {
    *logline = String(voutVal);
    *logline += "e";
    *logline += String(IS_RANGE_NA   ? -9
                       : IS_RANGE_UA ? -6
                       : -3);
  } else if (LOGGING_FORMAT == LOGGING_FORMAT_NANOS)
    *logline = String(voutVal);
  else if (LOGGING_FORMAT == LOGGING_FORMAT_MICROS) {
    *logline = String(SampleTime);
    *logline += ",";
    *logline += String(adcRange);
    *logline += ",";
    *logline += String(voutVal * 0.001);
  } else if (LOGGING_FORMAT == LOGGING_FORMAT_MILLIS) {
    *logline = String(SampleTime);
    *logline += ",";
    *logline += String(voutVal * 0.000001);
  } else if (LOGGING_FORMAT == LOGGING_FORMAT_ADC)
    *logline += String(vAdc, 0) + "\t" + String(vCor, 0);
  else if (LOGGING_FORMAT == LOGGING_FORMAT_DISPLAY) {
    String olv, vov, space = "               ";
    char warn = ')';
    float sv  = voutVal;
    for (int i = adcRange; i; i--)
      sv *= 0.001;
    if (BIAS) {
      if (sv > 1400. || sv < -1400.)
        warn = '!';
      else if (sv > -1.4 && sv < 1.4)
        warn = '~';
    } else {
      if (sv > 2800. || sv < -28.)
        warn = '!';
      else if (sv < 2.8)
        warn = '~';
    }

    olv = String(oledVal);
    if (oledVal >= 0.)
      olv = String(' ') + olv;
    vov = String(voutVal);
    if (voutVal >= 0.)
      vov = String(' ') + vov;
    *logline = "Range " + String("num"[adcRange]) + " :" + String(warn) + " ";
    *logline += space.substring(0, space.length() - olv.length()) + olv;
    *logline += space.substring(0, space.length() - vov.length()) + vov;
  }

  return retflg;
}

void handleOLED(void) {

  if (!OLED_found)
    return;

  String txt;
  uint8_t showRange = adcRange;
  char showChrge    = 0;
  float showVal     = oledVal;
  bool showTwo      = false;

  u8g2.clearBuffer(); //175us

  // BT and battery symbol
  u8g2.setFont(u8g2_font_siji_t_6x10);
  if (showBT)
    u8g2.drawGlyph(104, 10, 0xE00B); //BT icon
  if (vBat > 4.3)
    u8g2.drawGlyph(115, 10, 0xE23A); //charging!
  else if (vBat > 4.1)
    u8g2.drawGlyph(115, 10, 0xE24B); //100%
  else if (vBat > 3.95)
    u8g2.drawGlyph(115, 10, 0xE249); //80%
  else if (vBat > 3.85)
    u8g2.drawGlyph(115, 10, 0xE247); //60%
  else if (vBat > 3.75)
    u8g2.drawGlyph(115, 10, 0xE245); //40%
  else if (vBat > 3.65)
    u8g2.drawGlyph(115, 10, 0xE244); //20%
  else if (vBat > LOBAT_THRESHOLD)
    u8g2.drawGlyph(115, 10, 0xE243); //5%!
  else
    u8g2.drawGlyph(115, 10, 0xE242); //u8g2.drawStr(88,12,"LoBat!");

  // headline
  u8g2.setFont(u8g2_font_6x12_tf); //7us

  // auto-range mode, hold/peak mode, auto-off remaining and warning
  if (AUTORANGE) {
    u8g2.drawStr(0, 10, "AUTO");
    if (extraMode == 1) {
      u8g2.drawStr(34, 10, "PEAK");
      showVal = voutPkhEx1;
    } else if (extraMode == 2 || extraMode == 3) {
      u8g2.drawStr(34, 10, "AVRG");
      showVal = voutCntEx2 ? (voutSumEx2 / voutCntEx2) : 0.;
    }

    if (abs(showVal) > 10000000.) {
      showRange = 2;
    } else if (abs(showVal) > 1000000.) {
      showTwo   = true;
      showRange = 2;
    } else if (abs(showVal) > 10000.) {
      showRange = 1;
    } else if (abs(showVal) > 1000.) {
      showTwo   = true;
      showRange = 1;
    } else {
      showRange = 0;
    }

  } else {
    u8g2.drawStr(0, 10, "MANU");
  }

  // normalise value to according range
  for (int i = showRange; i; i--)
    showVal *= 0.001;

#if 0
  // ADC debug val
  txt = String((int16_t)adcRead(SENSE_OUTPUT, SENSE_GNDISO));
  u8g2.drawStr(10, 25, txt.c_str());
#else
  bool ovload = false;
  if (BIAS) {
    if (showVal > 1500.) {
      showVal = 1500.;
      ovload = true;
    } else if (showVal < -1500.) {
      showVal = -1500.;
      ovload = true;
    }
  } else {
    if (showVal > 3000.) {
      showVal = 3000.;
      ovload = true;
    } else if (showVal < -10.) {
      showVal = -10.;
      ovload = true;
    }
  }

  txt = (autoOffRemain != AUTOOFF_DISABLED)
        ? String(autoOffRemain) + "s"
        : "";
  if (autoffBuzz)
    txt = "<" + txt + ">";
  else
    txt = " " + txt + " ";
  u8g2.drawStr(72 + 6 * (5 - txt.length()), 10, txt.c_str());

  u8g2.setFont(u8g2_font_9x15B_tf);
  if (AUTORANGE) {
    // extramode-2+3 time
    if (extraMode == 2 || extraMode == 3) {
      txt = String((millis() - Ex2millis) / 1000) + "s";
      u8g2.drawStr(10 + 8 * (8 - txt.length()), 25, txt.c_str());
    }

    // for extramode-3 adjust showval and additional unit
    if (extraMode == 3) {
      showVal *= (millis() - Ex2millis) / 1000;
      if (abs(showVal) >= 3600) {
        showVal /= 3600;
        showChrge = 'h';
        if (showVal > 9999.) {
          showVal = 9999.;
          ovload = true;
        }
      } else {
        showChrge = 's';
      }
    }
  }

  // range unit
  txt = String("n\xb5m"[showRange]) + "A" + String(showChrge);
  u8g2.drawStr(101 + 9 * (3 - txt.length()), 25, txt.c_str());

  if (ovload) {
    u8g2.drawStr(10, 25, "OVERLOAD!");
    tone(BUZZER, NOTE_A7, 30);
  }

#endif
  // value
  u8g2.setFont(u8g2_font_logisoso32_tr);
  if (!BIAS) {
    // on !BIAS keep value around zero positive except
    // when too low, show -0.0
    if (showVal < -0.09)
      showVal = -0.01;
    else if (showVal < 0)
      showVal = 0;
  }
  if (abs(showVal) >= 1000)
    txt = String(showVal, 0) + ".";
  else
    txt = String(showVal, showTwo ? 2 : 1);
  u8g2.drawStr(5 + 20 * (6 - txt.length()), 64, txt.c_str());

  u8g2.sendBuffer();
}

uint16_t valM = 0, valU = 0, valN = 0;
uint32_t tpAlive = 0;
void handleTouchPads() {
  static bool lock = false;

  if (millis() - touchSampleInterval < TOUCH_SAMPLE_INTERVAL)
    return;

  if (TOUCH_DEBUG_ENABLED) {
    Serial.print(qt[0].measure());
    Serial.print('\t');
    Serial.print(qt[1].measure());
    Serial.print('\t');
    Serial.println(qt[2].measure());
  }

  bool MA_PRESSED = qt[2].measure() > TOUCH_HIGH_THRESHOLD;
  bool UA_PRESSED = qt[1].measure() > TOUCH_HIGH_THRESHOLD;
  bool NA_PRESSED = qt[0].measure() > TOUCH_HIGH_THRESHOLD;

  touchSampleInterval = millis();
  if (MA_PRESSED && UA_PRESSED && NA_PRESSED) {
    lastKeepAlive = millis();
    tpAll         = 1;
    return;
  } else if (MA_PRESSED || UA_PRESSED || NA_PRESSED) {
    lastKeepAlive = millis();
  } else {
    tpAlive = 0;
    lock    = false;
  }
  tpAll = -1; // signal tp measured

  //range switching
  if (MA_PRESSED && !UA_PRESSED && !NA_PRESSED) {
    if (!tpAlive)
      tpAlive = millis();
    if (!IS_RANGE_MA && !AUTORANGE) {
      rangeMA();
      rangeBeep(20);
    } else if (extraMode != 0 && AUTORANGE) {
      extraMode = 0;
      rangeBeep(20);
    } else if (tpAlive && millis() - tpAlive > 2000) {
      if (autooff_interval != AUTOOFF_DISABLED)
        autooff_interval = AUTOOFF_DISABLED;
      else
        autooff_interval = AUTOOFF_DEFAULT;
      rangeBeep(20);
      tpAlive = 0;
    }
  }
  if (UA_PRESSED && !MA_PRESSED && !NA_PRESSED) {
    if (!tpAlive)
      tpAlive = millis();
    if (!IS_RANGE_UA && !AUTORANGE) {
      rangeUA();
      rangeBeep(20);
    } else if (extraMode != 1 && AUTORANGE) {
      extraMode = 1;
      rangeBeep(20);
    } else if (tpAlive && millis() - tpAlive > 2000) {
      if (AUTORANGE) {
        voutPkhEx1 = 0;
        rangeBeep(20);
      }
      tpAlive = 0;
    }
  }
  if (NA_PRESSED && !UA_PRESSED && !MA_PRESSED) {
    if (!tpAlive)
      tpAlive = millis();
    if (!IS_RANGE_NA && !AUTORANGE) {
      rangeNA();
      rangeBeep(20);
    } else if (extraMode != 2 && AUTORANGE && !lock) {
      extraMode = 2;
      rangeBeep(20);
      lock = true;
    } else if (extraMode == 2 && AUTORANGE && !lock) {
      extraMode = 3;
      rangeBeep(20);
      lock = true;
    } else if (tpAlive && millis() - tpAlive > 2000) {
      if (AUTORANGE) {
        Ex2millis  = millis();
        voutSumEx2 = 0;
        voutCntEx2 = 0;
        rangeBeep(20);
      }
      tpAlive = 0;
    }
  }

  //LPF activation --- [MA+UA]
  //  -> both rightmost pads activate right-located feature LPF
  if (UA_PRESSED && !NA_PRESSED && MA_PRESSED && millis() - lpfInterval > 1000) {
    toggleLPF();
    Beep(3, false);
  }

  //BIAS toggling (GNDISO to half supply) --- [NA+UA]
  //  -> both leftmost pads activate left-located feature BIAS
  if (!MA_PRESSED && UA_PRESSED && NA_PRESSED && millis() - offsetInterval > 1000) {
    toggleBIAS();
    Beep(3, false);
  }

  //AUTORANGE toggling
  if (MA_PRESSED && NA_PRESSED && !UA_PRESSED && millis() - autorangeInterval > 1000) {
    toggleAUTORANGE();
    Beep(20, false);
    delay(50);
    Beep(20, false);
    // reset peak and avrg, TODO: maybe put in a fn() or MACRO()
    extraMode  = 0;
    Ex2millis  = millis();
    voutSumEx2 = 0;
    voutCntEx2 = 0;
    voutPkhEx1 = 0;
    if (AUTORANGE) {
      rangeNA();
    }
  }
}

void rangeMA() {
  adcRange = 2;
  PIN_ON(MA_PIN);
  PIN_OFF(UA_PIN);
  PIN_OFF(NA_PIN);
  if (GPIO_HEADER_RANGING) {
    PIN_ON(MA_GPIO_PIN);
    PIN_OFF(UA_GPIO_PIN);
    PIN_OFF(NA_GPIO_PIN);
  }
  avgdiscard = 2;
#ifdef BT_OUTPUT_ADC
  if (BT_found)
    SerialBT.println("RANGE: MA");
#endif
}

void rangeUA() {
  adcRange = 1;
  PIN_OFF(MA_PIN);
  PIN_ON(UA_PIN);
  PIN_OFF(NA_PIN);
  if (GPIO_HEADER_RANGING) {
    PIN_OFF(MA_GPIO_PIN);
    PIN_ON(UA_GPIO_PIN);
    PIN_OFF(NA_GPIO_PIN);
  }
  avgdiscard = 2;
#ifdef BT_OUTPUT_ADC
  if (BT_found)
    SerialBT.println("RANGE: UA");
#endif
}

void rangeNA() {
  adcRange = 0;
  PIN_OFF(MA_PIN);
  PIN_OFF(UA_PIN);
  PIN_ON(NA_PIN);
  if (GPIO_HEADER_RANGING) {
    PIN_OFF(MA_GPIO_PIN);
    PIN_OFF(UA_GPIO_PIN);
    PIN_ON(NA_GPIO_PIN);
  }
  avgdiscard = 2;
#ifdef BT_OUTPUT_ADC
  if (BT_found)
    SerialBT.println("RANGE: NA");
#endif
}

void handleAutoOff() {
  uint32_t autooff_deadline = AUTOOFF_DEFAULT * 1000;

  autoOffRemain = (500 + autooff_deadline - (millis() - lastKeepAlive)) / 1000;

  if (autooff_interval == AUTOOFF_DISABLED ||
      (autooff_interval == AUTOOFF_SMART &&
       (USB_LOGGING_ENABLED || BT_LOGGING_ENABLED))) {
    lastKeepAlive = millis();
    autoOffRemain = AUTOOFF_DISABLED;
  }

  if (millis() - lastKeepAlive > autooff_deadline - 5 * 1000) {
    autoffWarning = true;

    if (millis() - autoOffBuzzInterval > AUTOOFF_BUZZ_DELAY) {
      autoOffBuzzInterval = millis();
      autoffBuzz          = !autoffBuzz;

      if (autoffBuzz)
        tone(BUZZER, NOTE_B7);
      else
        noTone(BUZZER);
    }

    if (millis() - lastKeepAlive > autooff_deadline) {
      pinMode(AUTOFF, OUTPUT);
      digitalWrite(AUTOFF, LOW);
      while (1)
        WDTclear();
    }
  } else if (autoffWarning) {
    autoffWarning = autoffBuzz = false;
    digitalWrite(AUTOFF, HIGH);
    noTone(BUZZER);
  }
}

void toggleLPF() {
  LPF         = !LPF;
  lpfInterval = millis();
  digitalWrite(LPFPIN, LPF);
  digitalWrite(LPFLED, LPF);
  // if (AUTORANGE && !LPF)
  //     toggleAUTORANGE(); //turn off AUTORANGEn
}

void toggleBIAS() {
  BIAS           = !BIAS;
  offsetInterval = millis();
  analogWrite(A0, (BIAS ? DAC_HALF_SUPPLY_OFFSET : DAC_GND_ISO_OFFSET));
  digitalWrite(BIAS_LED, BIAS);
  // if (AUTORANGE && BIAS)
  //     toggleAUTORANGE(); //turn off AUTORANGE
}

void toggleAUTORANGE() {
  autorangeInterval = millis();
  AUTORANGE         = !AUTORANGE;
  // if (AUTORANGE && BIAS)
  //     toggleBIAS(); //turn off BIAS
  // if (AUTORANGE && !LPF)
  //     toggleLPF(); //turn on BIAS
}

void Beep(uint8_t theDelay, boolean twoSounds) {
  tone(BUZZER, TONE_BEEP, theDelay);
  if (twoSounds) {
    delay(10);
    tone(BUZZER, 4500, theDelay);
  }
}

static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}

void setupADC() {
  ADC->CTRLA.bit.ENABLE = 0; // disable ADC
  analogReference(AR_DEFAULT);
  syncADC();
  ADC->REFCTRL.bit.REFCOMP = 1;

  ADC->CTRLB.reg    = ADC_PRESCALER | ADC_CTRLB_RESSEL_16BIT | ADC_CTRLB_DIFFMODE;
  ADC->AVGCTRL.reg  = ADC_AVGCTRL;
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL;

  ADC->CTRLA.bit.ENABLE = 1; // enable ADC
  syncADC();
  //  // ADC Linearity/Bias Calibration from NVM (should already be done done in core)
  //  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  //  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  //  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
  //  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
}

int16_t adcRead(uint8_t ADCpinPOS, uint8_t ADCpinNEG) {

  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ADCpinPOS].ulADCChannelNumber;
  ADC->INPUTCTRL.bit.MUXNEG = (ADCpinNEG != SENSE_IOGND)
                              ? g_APinDescription[ADCpinNEG].ulADCChannelNumber
                              : 0x19;
  syncADC();
  ADC->SWTRIG.bit.START = 1;
  while (ADC->INTFLAG.bit.RESRDY == 0)
    ;
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
  syncADC();
  return ADC->RESULT.reg;
}

void WDTset() {
  // Generic clock generator 2, divisor = 32 (2^(DIV+1))
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  // Enable clock generator 2 using low-power 32KHz oscillator. With /32 divisor above, this yields 1024Hz(ish) clock.
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
  // WDT clock = clock gen 2
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2;

  WDT->CTRL.reg = 0; //disable WDT
  while (WDT->STATUS.bit.SYNCBUSY)
    ;
  WDT->INTENCLR.bit.EW = 1;   //disable early warning
  WDT->CONFIG.bit.PER  = 0xA; //period ~8s
  WDT->CTRL.bit.WEN    = 0;   //disable window mode
  while (WDT->STATUS.bit.SYNCBUSY)
    ;
  WDTclear();
  WDT->CTRL.bit.ENABLE = 1; //enable WDT
  while (WDT->STATUS.bit.SYNCBUSY)
    ;
}

uint32_t WDTInterval = 0;
void WDTclear() {
  if (millis() - WDTInterval > 6999) //pet the dog every 7s
  {
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
    //while(WDT->STATUS.bit.SYNCBUSY);
    WDTInterval = millis();
  }
}

void ldoOptimizeRefresh() {
  ldoOptimized = (ldoValue * 1000) / ADCFULLRANGE;
}

void saveLDO(float newLdoValue) {
  ldoValue = newLdoValue;
  eeprom_LDO.write(newLdoValue);
  ldoOptimizeRefresh();
}

void refreshADCSamplingSpeed() {
  if (ADC_SAMPLING_SPEED == ADC_SAMPLING_SPEED_AVG)
    ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM_64 | ADC_AVGCTRL_ADJRES(0);
  else if (ADC_SAMPLING_SPEED == ADC_SAMPLING_SPEED_FAST)
    ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(0);
  else if (ADC_SAMPLING_SPEED == ADC_SAMPLING_SPEED_SLOW)
    ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM_256 | ADC_AVGCTRL_ADJRES(0);
  setupADC();
  //other combinations:
  //ADC_AVGCTRL_SAMPLENUM_128 | ADC_AVGCTRL_ADJRES(0x4ul)
  //ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x00ul);  // take 1 sample, adjusting result by 0
  //ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(0x4ul); //take 16 samples adjust by 4
  //ADC_AVGCTRL_SAMPLENUM_256 | ADC_AVGCTRL_ADJRES(0x4ul); //take 256 samples adjust by 4
  //ADC_AVGCTRL_SAMPLENUM_512 | ADC_AVGCTRL_ADJRES(0x4ul); //take 512 samples adjust by 4
  //ADC_AVGCTRL_SAMPLENUM_1024 | ADC_AVGCTRL_ADJRES(0x4ul); //take 1024 samples adjust by 4
}

void printCalibInfo() {
  Serial.println("\r\nADC calibration values:");
  Serial.print("OfsUni=");
  Serial.println(ofsUniCorrectionValue);
  Serial.print("OfsBias=");
  Serial.println(ofsBiaCorrectionValue);
  Serial.print("Gain=");
  Serial.println(gainCorrectionValue, 3);
  Serial.print("LDO=");
  Serial.println(ldoValue, 3);
}

void printConfigInfo() {
  Serial.println("\r\nEEPROM Settings:");
  Serial.print("LoggingFormat=");
  if (LOGGING_FORMAT == LOGGING_FORMAT_EXPONENT)
    Serial.println("EXPONENT");
  else if (LOGGING_FORMAT == LOGGING_FORMAT_NANOS)
    Serial.println("NANOS");
  else if (LOGGING_FORMAT == LOGGING_FORMAT_MICROS)
    Serial.println("MICROS");
  else if (LOGGING_FORMAT == LOGGING_FORMAT_MILLIS)
    Serial.println("MILLIS");
  else if (LOGGING_FORMAT == LOGGING_FORMAT_ADC)
    Serial.println("ADC");
  else
    Serial.println("unknown");
  Serial.print("ADCSamplingSpeed=");
  if (ADC_SAMPLING_SPEED == ADC_SAMPLING_SPEED_AVG)
    Serial.println("AVG");
  else if (ADC_SAMPLING_SPEED == ADC_SAMPLING_SPEED_FAST)
    Serial.println("FAST");
  else if (ADC_SAMPLING_SPEED == ADC_SAMPLING_SPEED_SLOW)
    Serial.println("SLOW");
  else
    Serial.println("unknown");
  Serial.print("AutoOff=");
  if (autooff_interval == AUTOOFF_DISABLED) {
    Serial.println("DISABLED");
  } else if (autooff_interval == AUTOOFF_SMART) {
    Serial.println("SMART");
  } else {
    Serial.println(autooff_interval);
  }

  Serial.println("");
}

void printSerialMenu() {
  // Print device name, firmware version and state for interop on PC side
  Serial.println("\r\nCurrentRanger R3");
  Serial.print("Firmware version: ");
  Serial.println(FW_VERSION);
  Serial.print("BT Logging: ");
  Serial.println(BT_LOGGING_ENABLED);
  Serial.print("USB Logging: ");
  Serial.println(USB_LOGGING_ENABLED);

  printCalibInfo();
  printConfigInfo();

  Serial.println("a = cycle Auto-Off function");
  Serial.print("b = toggle BT/serial logging (");
  Serial.print(SERIAL_UART_BAUD);
  Serial.println("baud)");
  Serial.println("c = show replayable calibration data string");
  Serial.println(": = calibration string");
  Serial.println("f = cycle serial logging formats (exponent, nA, uA, mA, ADC, Display)");
  Serial.println("g = toggle GPIO range indication (SCK=mA,MISO=uA,MOSI=nA)");
  Serial.println("i = toggle BIAS setting");
  Serial.println("r = reboot into bootloader");
  Serial.println("s = cycle ADC sampling speeds (0=average,faster,slower)");
  Serial.println("t = toggle touchpad serial output debug info");
  Serial.println("u = toggle USB/serial logging");
  Serial.println("v = show raw ADC values VOUT, VBAT");
  Serial.println("< = Calibrate LDO value (-1mV)");
  Serial.println("> = Calibrate LDO value (+1mV)");
  Serial.println("[ = Calibrate GAIN value (-0.01)");
  Serial.println("] = Calibrate GAIN value (+0.01)");
  Serial.println("{ = Calibrate OFFSET values (-1)");
  Serial.println("} = Calibrate OFFSET values (+1)");
  Serial.println("? = Print this menu and calib info");
  Serial.println();
}

void rangeBeep(uint16_t switch_delay) {
  uint16_t freq = NOTE_C7;
  if (IS_RANGE_UA)
    freq = NOTE_D7;
  if (IS_RANGE_MA)
    freq = NOTE_E7;
  tone(BUZZER, freq, switch_delay ? switch_delay : 20);
  handleOLED();
}

void bootScreen(void) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_8x13B_tf);
  u8g2.setCursor(15, 10);
  u8g2.print("CurrentRanger");
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.setCursor(0, 20);
  u8g2.print("Offsets:");
  u8g2.setCursor(64, 20);
  u8g2.print(ofsUniCorrectionValue);
  u8g2.setCursor(96, 20);
  u8g2.print(ofsBiaCorrectionValue);
  u8g2.setCursor(0, 32);
  u8g2.print("Gain  :");
  u8g2.setCursor(64, 32);
  u8g2.print(gainCorrectionValue, 3);
  u8g2.setCursor(0, 44);
  u8g2.print("LDO   :");
  u8g2.setCursor(64, 44);
  u8g2.print(ldoValue, 3);
  u8g2.setCursor(0, 56);
  u8g2.print("Firmware:");
  u8g2.setCursor(64, 56);
  u8g2.print(FW_VERSION);
  u8g2.sendBuffer();
}

#define REBOOT_TOKEN 0xf01669ef //special token in RAM, picked up by the bootloader
void rebootIntoBootloader() {

  if (OLED_found)
    bootScreen();

  *((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4)) = REBOOT_TOKEN; //Entering bootloader from application: https://github.com/microsoft/uf2-samdx1/issues/41
  NVIC_SystemReset();
}
