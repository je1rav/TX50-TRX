/*
 * Copyright (C) 2023- Hitoshi Kawaji <je1rav@gmail.com>
 * 
 * TX27_TRX_XIAO.ino
 * 
 * TX27_TRX_XIAO.ino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * 
 * TX27_TRX_XIAO.ino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#define Si5351_CAL -22000 // Calibrate for your Si5351 module.
#define FREQ_IF 450000 // IF frequency in Hz: Calibrate for your IF filter.  

#define N_FREQ 8 // number of using RF frequencies
#define FREQ_0 26968000 // RF frequency in Hz
#define FREQ_1 26976000 // in Hz
#define FREQ_2 27040000 // in Hz
#define FREQ_3 27080000 // in Hz
#define FREQ_4 27088000 // in Hz
#define FREQ_5 27112000 // in Hz
#define FREQ_6 27120000 // in Hz
#define FREQ_7 27144000 // in Hz
/*
#define FREQ_0 28285000 // RF frequency in Hz
#define FREQ_1 28295000 // in Hz
#define FREQ_2 28305000 // in Hz
#define FREQ_3 28315000 // in Hz
#define FREQ_4 28325000 // in Hz
#define FREQ_5 28335000 // in Hz
#define FREQ_6 28345000 // in Hz
#define FREQ_7 28355000 // in Hz
*/
uint64_t Freq_table[N_FREQ]={FREQ_0,FREQ_1,FREQ_2,FREQ_3,FREQ_4,FREQ_5,FREQ_6,FREQ_7}; // Freq_table[N_FREQ]={FREQ_0,FREQ_1, ...}

#define pin_SW1 2 //pin for freq change switch (input)
#define pin_TX_STATUS 29 //pin to detect transmittion (input)
#define pin_RED 17 //pin for onboard Red LED (output)
#define pin_GREEN 16 //pin for onboard GREEN LED (output)
#define pin_BLUE 25 //pin for onboard BLUE LED (output) 
#define pin_LED_POWER 11 //pin for onboard NEOPIXEL LED power (output)
#define pin_LED 12 //pin for onboard NEOPIXEL LED (output)

#include <Wire.h>
#include <si5351.h>  //"Etherkit Si5351"
Si5351 si5351;

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(1, pin_LED);
#define BRIGHTNESS 4  //max 255
uint32_t colors[] = {pixels.Color(BRIGHTNESS, 0, 0), pixels.Color(0, BRIGHTNESS, 0), pixels.Color(0, 0, BRIGHTNESS), pixels.Color(BRIGHTNESS, BRIGHTNESS, 0), pixels.Color(BRIGHTNESS, 0, BRIGHTNESS), pixels.Color(0, BRIGHTNESS, BRIGHTNESS), pixels.Color(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS), pixels.Color(0, 0, 0)};

uint64_t RF_freq;   // RF frequency (Hz)
int64_t IF_freq;   // IF frequency (Hz)
int local_osc = 1;  //1 -> local oscilation frequency is lower than that of receiving frequency , -1 -> local oscilation frequency is higher than that of receiving frequency
int C_freq = 0;  //FREQ_x: In this case, FREQ_0 is selected as the initial frequency. 
int Tx_Status =0;

#include "pico/stdlib.h"  //for use set_sys_clock_khz(): change the system clock frequency

void setup() {

  //for low energy mode 
  //After using the next function, you should enter the programming mode to upload a new program by holding the BOOTSEL button when powering on the Pico.
  //set_sys_clock_khz(18000, true);   //CPU clock is down to 18000kHz (18MHz) to reduce the power consumption.

  //Digital pin setting ----- 
  pinMode(pin_SW1, INPUT_PULLUP); //SW (freq. change)
  pinMode(pin_TX_STATUS, INPUT); //SW (freq. change)

  pinMode(pin_RED, OUTPUT); //On →　LOW (for RED LED)
  pinMode(pin_GREEN, OUTPUT); //On →　LOW (for GREEN LED)
  pinMode(pin_BLUE, OUTPUT); //On →　LOW (for BLUE LED)
  pinMode(pin_LED_POWER, OUTPUT); //NEOPIXEL LED

  //I2c initialization-----  
  Wire.begin();

  //si5351 initialization-----  
  int32_t cal_factor = Si5351_CAL;
  RF_freq = Freq_table[C_freq];
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); 
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.set_freq(RF_freq*100ULL, SI5351_CLK0);  //for TX
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK0, 0);
  IF_freq = FREQ_IF * local_osc;
  si5351.set_freq((RF_freq-IF_freq)*100ULL, SI5351_CLK1);  //for RX
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  si5351.output_enable(SI5351_CLK1, 0);
  
  //NEOPIXEL LED initialization-----
  digitalWrite(pin_LED_POWER, HIGH);  //NEOPIXEL LED ON
  pixels.begin();  // initialize the NEOPIXEL
  pixels.setPixelColor(0, colors[C_freq]);
  pixels.show();

  //transceiver initialization-----
  si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
  si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on

  //Onboard LED initialization-----
  digitalWrite(pin_RED, HIGH);
  digitalWrite(pin_GREEN, LOW); //Green LED ON
  digitalWrite(pin_BLUE, HIGH);
}

void loop() {
  if (digitalRead(pin_TX_STATUS) == 1 ) transmit();
  else {
    receive();
    freqChange();
  }
}

void transmit(){
  if (Tx_Status==0){
    si5351.output_enable(SI5351_CLK1, 0);   //RX osc. off
    si5351.output_enable(SI5351_CLK0, 1);   //TX osc. on
    Tx_Status=1;
    digitalWrite(pin_RED, 0);
    digitalWrite(pin_GREEN, 1);
    //digitalWrite(pin_BLUE, 1);
  }
}

void receive(){
  if (Tx_Status==1){
    si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
    si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on
    Tx_Status=0;
    digitalWrite(pin_RED, 1);
    digitalWrite(pin_GREEN, 0);
    //digitalWrite(pin_BLUE, 1);
  }
}

void freqChange(){
  if (digitalRead(pin_SW1)==LOW){
    delay(100);
    if  (digitalRead(pin_SW1)==LOW){
      C_freq++;
      if  (C_freq >= N_FREQ){
        C_freq = 0;
      }
    }
    RF_freq = Freq_table[C_freq];
    si5351.set_freq(RF_freq*100ULL, SI5351_CLK0);
    si5351.set_freq((RF_freq-IF_freq)*100ULL, SI5351_CLK1);
    pixels.setPixelColor(0, colors[C_freq]);
    pixels.show();
    delay(100);
  }
}
