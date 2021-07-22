#include <SPI.h>
#include <stdint.h>
#include <BLEPeripheral.h>
#include <nrf_nvic.h>//interrupt controller stuff
#include <nrf_sdm.h>
#include <nrf_soc.h>
#include <WInterrupts.h>
#include "Adafruit_GFX.h"
#include "SSD1306.h"
#include <TimeLib.h>
#include <nrf.h>
#include "i2csoft.h"

#define wdt_reset() NRF_WDT->RR[0] = WDT_RR_RR_Reload
#define wdt_enable(timeout) \
  NRF_WDT->CONFIG = NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos); \
  NRF_WDT->CRV = (32768*timeout)/1000; \
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  \
  NRF_WDT->TASKS_START = 1

Adafruit_SSD1306 display(128, 32, &SPI, 28, 4, 29);

boolean debug = true;

// TODO: bring it back to smaller value later (3000ms?)
#define sleepDelay ((unsigned long)-1)
#define BUTTON_PIN              30
#define refreshRate 100

int menu;
volatile bool buttonPressed = false;
long startbutton;
unsigned long sleepTime, displayRefreshTime;
volatile bool sleeping = false;
boolean gotoBootloader = false;
boolean vibrationMode;
int contrast;
uint32_t        prevTime   = 0;      // Used for frames-per-second throttle

#ifdef __cplusplus
extern "C" {
#endif

#define LF_FREQUENCY 32768UL
#define SECONDS(x) ((uint32_t)((LF_FREQUENCY * x) + 0.5))
#define wakeUpSeconds 120
void RTC2_IRQHandler(void)
{
  volatile uint32_t dummy;
  if (NRF_RTC2->EVENTS_COMPARE[0] == 1)
  {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    NRF_RTC2->CC[0] = NRF_RTC2->COUNTER +  SECONDS(wakeUpSeconds);
    dummy = NRF_RTC2->EVENTS_COMPARE[0];
    dummy;
    //powerUp();
  }
}

void initRTC2() {

  NVIC_SetPriority(RTC2_IRQn, 15);
  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NVIC_EnableIRQ(RTC2_IRQn);

  NRF_RTC2->PRESCALER = 0;
  NRF_RTC2->CC[0] = SECONDS(wakeUpSeconds);
  NRF_RTC2->INTENSET = RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos;
  NRF_RTC2->EVTENSET = RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos;
  NRF_RTC2->TASKS_START = 1;
}
#ifdef __cplusplus
}
#endif

void powerUp() {
  if (sleeping) {
    sleeping = false;
    display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
    display.display();
    if (debug)Serial.begin(115200);
    delay(5);
  }
  sleepTime = millis();
}

void powerDown() {
  if (!sleeping) {
    if (debug)NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Disabled;
    sleeping = true;

  digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(29, LOW);
    digitalWrite(4, LOW);
    NRF_SAADC ->ENABLE = 0; //disable ADC
    NRF_PWM0  ->ENABLE = 0; //disable all pwm instance
    NRF_PWM1  ->ENABLE = 0;
    NRF_PWM2  ->ENABLE = 0;
  }
}


void buttonHandler() {
  if (!sleeping) buttonPressed = true;
  else menu = 0;
  powerUp();
}

void acclHandler() {
  ReadRegister(0x17);
  if (sleeping) {
    menu = 77;
    powerUp();
  }
}


void setup() {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(3, INPUT);
  if (digitalRead(BUTTON_PIN) == LOW) {
    NRF_POWER->GPREGRET = 0x01;
    sd_nvic_SystemReset();
  }
  wdt_enable(10000);
  pinMode(2, INPUT);
  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(15, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(15), acclHandler, RISING);
  NRF_GPIO->PIN_CNF[15] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);
  NRF_GPIO->PIN_CNF[15] |= ((uint32_t)GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
//  attachInterrupt(digitalPinToInterrupt(2), charge, RISING);
  NRF_GPIO->PIN_CNF[2] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);
  NRF_GPIO->PIN_CNF[2] |= ((uint32_t)GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
  display.begin(SSD1306_SWITCHCAPVCC);
  delay(100);
  display.clearDisplay();
  // display.setFont(&FreeSerifItalic9pt7b);
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.println("D6 Emulator");
  display.display();
  digitalWrite(25, LOW);
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  initRTC2();
  initi2c();
  initkx023();

}
void loop() {
  if (digitalRead(BUTTON_PIN) ) wdt_reset();
  if (sleeping) {
    sd_app_evt_wait();
    sd_nvic_ClearPendingIRQ(SD_EVT_IRQn);
  } else {
 displayMenu0();
  }
}
void displayMenu0() {
  display.setRotation(0);

  uint8_t res[6];
  softbeginTransmission(0x1F);
  softwrite(0x06);
  softendTransmission();
  softrequestFrom(0x1F , 6);
  for (int i = 0; i < 6; i++) {
      res[i] = softread();
  }

  float acc[] = { int16_t((res[1] << 8) | res[0]) / 128,
                  int16_t((res[3] << 8) | res[2]) / 128,
                  int16_t((res[5] << 8) | res[4]) / 128 };

  // pitch, roll - source:
  // http://www.hobbytronics.co.uk/accelerometer-info
  float tilt[] = {(atan(acc[0] / sqrt(acc[1]*acc[1] + acc[2]*acc[2]))*180) / M_PI,
                  (atan(acc[1] / sqrt(acc[0]*acc[0] + acc[2]*acc[2]))*180) / M_PI };

  bool risk = tilt[1] < -30;

  // vibrate if we raise hand (simple WIP test)
  if (risk) {
    digitalWrite(25, HIGH); // vibrate
    display_warning1();
    delay(100);

    digitalWrite(25, LOW);  // stop vibration
    display_warning2();
    delay(100);
  } else {
      display.clearDisplay();
      display.display();
  }
}

void display_warning1() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("      X   X   X      ");
  display.println("                     ");
  display.println("      X   X   X      ");
  display.println("      X   X   X      ");
  display.display();
}

void display_warning2() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("XXXXXX XXX XXX XXXXXX");
  display.println("XXXXXXXXXXXXXXXXXXXXX");
  display.println("XXXXXX XXX XXX XXXXXX");
  display.println("XXXXXX XXX XXX XXXXXX");
  display.display();
}


void writeRegister(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(0x6b);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t addr)
{
  Wire.beginTransmission(0x6b);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(0x6b, 1);
  return Wire.read();
}
