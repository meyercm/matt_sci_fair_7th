#include <Arduino.h>
#include "Adafruit_GFX.h"
#include "Adafruit_FeatherOLED.h"
#include "Adafruit_SSD1306.h"
#include <stdint.h>

#define MIC_PIN A5
#define SPEAKER_PIN 11
#define LED_PIN 13

#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5

#define TONE_DELAY_US 50000
#define TONE_DURATION_US 50000
#define MAX_MEASURE_US 150000
#define MAX_MEASURE 1000

void update_led();
void update_measure_state();
void measure();

typedef enum {
  NOT_STARTED,
  START,
  TONE,
  WAITING
} MEASUREMENT_STATE;

struct MEASUREMENT {
  // populate at measure time
  int32_t time;
  int16_t value;
  uint32_t ad1_sum;
  // populate after run
  float pedro;
} ;


Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

void button_a_isr();
void button_b_isr();
void button_c_isr();

void setup() {
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(MIC_PIN, INPUT);

  oled.init();
  oled.clearDisplay();
  oled.println("");
  oled.display();
  attachInterrupt(digitalPinToInterrupt(BUTTON_A), button_a_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_B), button_b_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_C), button_c_isr, FALLING);
  Serial.begin(115200);
}

auto state = HIGH;

MEASUREMENT_STATE measurement_state = NOT_STARTED;
long start_time;
long tone_start;
long tone_stop;
int measurement_count;

MEASUREMENT measurements[MAX_MEASURE];

void loop() {
  update_led();
  update_measure_state();
  measure();
}


void button_a_isr(){
  if(NOT_STARTED == measurement_state) {
    measurement_state = START;
    measurement_count = 0;
    measurements[0].ad1_sum = 0;
    start_time = micros();
  } else{ // we are already measuring, so bail
    return;
  }
}
void button_b_isr(){

}
void button_c_isr(){

}

void update_led(){
  switch(measurement_state) {
    case NOT_STARTED:
      digitalWrite(LED_PIN, LOW);
      break;
    default:
      digitalWrite(LED_PIN, HIGH);
      break;
  }
}

void update_measure_state(){
  switch(measurement_state){
    case NOT_STARTED:
      break;
    case START: {
      if(start_time + TONE_DELAY_US < micros()){
        tone_start = micros() - start_time;
        analogWrite(SPEAKER_PIN, 127);
        measurement_state = TONE;
      }
      break;
    }
    case TONE: {

      if (start_time + TONE_DELAY_US + TONE_DURATION_US < micros()) {
        tone_stop = micros() - start_time;
        analogWrite(SPEAKER_PIN, 0);
        measurement_state = WAITING;
      }
      break;
    }
    case WAITING: {
      if (start_time + MAX_MEASURE_US < micros()) {
        // Serial.print("Tone Start: ");
        // Serial.println(tone_start);
        // Serial.print("Tone Stop: ");
        // Serial.println(tone_stop);
        MEASUREMENT last_measure = measurements[measurement_count - 1];
        for (int i = 0; i < measurement_count; i++){
          measurements[i].pedro = (float)((float)last_measure.ad1_sum * (float)((float)i / (float)(measurement_count - 1))) - (float)measurements[i].ad1_sum;
          Serial.print(measurements[i].time);
          Serial.print(" ");
          Serial.print(measurements[i].ad1_sum);
          Serial.print(" ");
          Serial.print(measurements[i].pedro);
          Serial.print(" ");
          Serial.println(measurements[i].value);
        }
        uint16_t largest_index = 0;
        int16_t largest_value = 0;
        for (int i = 0; i < measurement_count; i++){
          if(largest_value < measurements[i].pedro) {
            largest_value = measurements[i].pedro;
            largest_index = i;
          }
        }
        oled.clearMsgArea();

        // oled.print("start = ");
        // oled.println(tone_start);
        // oled.print("max pedro = ");
        // oled.print(measurements[largest_index].pedro);
        // oled.print(" at ");
        // oled.println(measurements[largest_index].time);
        oled.print("delta = ");
        oled.print(measurements[largest_index].time - tone_start);
        oled.print("us");
        oled.display();
        measurement_state = NOT_STARTED;
      }
    }
    default: {break;}
  }
}

void measure(){
  if (measurement_state != NOT_STARTED && measurement_count < MAX_MEASURE) {
    measurements[measurement_count].time = micros() - start_time;
    measurements[measurement_count].value = analogRead(MIC_PIN);
    if (measurement_count > 0) {
      measurements[measurement_count].ad1_sum = abs(measurements[measurement_count].value - measurements[measurement_count - 1].value) + measurements[measurement_count - 1].ad1_sum;
    }
    measurement_count++;
  }
}
