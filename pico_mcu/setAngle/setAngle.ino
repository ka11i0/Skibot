#include <Wire.h>
#include "RP2040_PWM.h"

// Pin GP10 to GP13
#define pin10   10    // PWM channel 5A
#define pin11   11    // PWM channel 5B
#define pin12   12    // PWM channel 6A
#define pin13   13    // PWM channel 6B

uint32_t PWM_Pins[]       = { pin10, pin11, pin12, pin13 };
#define NUM_OF_PINS       ( sizeof(PWM_Pins) / sizeof(uint32_t) )
float freq = 330.0f;
RP2040_PWM* PWM_Instance[NUM_OF_PINS];

bool ledState = 0;
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Set i2c address to 0x53
  Wire.begin(0x08);
  Wire.setClock(100000);
  // Set interupt function on 
  Wire.onReceive(setAngle);

  // Init PWM pins
  float mid_pos = 1500.0*0.0001*freq;
  for(uint8_t index = 0; index<NUM_OF_PINS; index++){
    PWM_Instance[index] = new RP2040_PWM(PWM_Pins[index], freq, mid_pos);
    PWM_Instance[index]->setPWM();
  }

  // For debugging
  //Serial.begin(9600);
}

void loop() {
  delay(100000);
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);
}

void setAngle(int numBytes) {
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);
  // Set PWM to requested via I2C
  int i = 0;
  while (Wire.available()) {  
    uint16_t pw;
    Wire.readBytes((uint8_t*)&pw, 2);
    PWM_Instance[i]->setPWM(PWM_Pins[i], freq, calc_duty((float)pw), true);
    i++;
  } 
}

float calc_duty(float pw) {
  float duty = pw*0.0001*freq;
  return duty;
}
