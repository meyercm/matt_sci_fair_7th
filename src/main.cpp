#include <Arduino.h>
#include "Adafruit_GFX.h"
#include "Adafruit_FeatherOLED.h"
#include "Adafruit_SSD1306.h"
#include <stdint.h>
#include <stdlib.h>

// ### Board Pin definitions
#define MIC_PIN A5
#define SPEAKER_PIN 11
#define LED_PIN 13
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5

// ### Application constants
#define TONE_DELAY_US 50000 // delay before playing tone
#define TONE_DURATION_US 50000 // duration of tone
#define MAX_MEASURE_US 150000 // total duration of measurements
#define MAX_MEASURE 1000 // max number of measurements
#define MAX_ARB_DELAY 20 // max random arbitrary delay between measurmenets

// ### Custom Types:
//  - Finite state machine for the measuring system
typedef enum {
  NOT_STARTED, // board is idle, this is the between-measurements and initial state
  START,       // button has been pushed, measurments are taken, no tone has been played
  TONE,        // tone is playing, measurements are taken
  WAITING      // tone is stopped, still taking measurements
} MEASUREMENT_STATE;

//  - Data structure to hold info about a single measurement
struct MEASUREMENT {
  // populate at measure time:
  int32_t time;
  int16_t value;
  uint32_t ad1_sum; // rolling sum of the absolute first difference.  Used in calculating Pedro statistic
  // populate after run:
  float pedro;
} ;

// ### Helper function prototypes
void button_a_isr();
void button_b_isr();
void button_c_isr();
void update_led();
void update_measure_state();
void measure();
void set_state(MEASUREMENT_STATE next_state);


// Global state variables
Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();
MEASUREMENT_STATE measurement_state = NOT_STARTED;
long start_time; // micros() when isr tripped
long tone_start; // us after `start_time` the tone actually was started
long tone_stop;  // us after `start_time` the tone was stopped
int measurement_count; // which measurement are we on
MEASUREMENT measurements[MAX_MEASURE]; // measurements array

void setup() {
  // setup board pins
  pinMode(MIC_PIN, INPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // initialize display
  oled.init();
  oled.clearDisplay();
  oled.println("");
  oled.display();

  // setup button interrupt handlers:
  attachInterrupt(digitalPinToInterrupt(BUTTON_A), button_a_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_B), button_b_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_C), button_c_isr, FALLING);

  // start serial port for full data download
  Serial.begin(115200);
}


void loop() {
  update_led();
  update_measure_state();
  measure();
}


void button_a_isr(){
  if(NOT_STARTED == measurement_state) {
    set_state(START);
  } else{ // we are already measuring, so bail
    return;
  }
}
void button_b_isr(){

}
void button_c_isr(){

}

// Controls the on board led. Called from the main loop.
// LED is off when not measuring, on when measuring.
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

// Cycles the FSM, completely time based.  Called from the main loop.
// FSM progression looks like:
// NOT_STARTED => START => TONE => WAITING => NOT_STARTED
void update_measure_state(){
  switch(measurement_state){
    case NOT_STARTED:
      break;
    case START: {
      if(start_time + TONE_DELAY_US < micros()){
        set_state(TONE);
      }
      break;
    }
    case TONE: {
      if (start_time + TONE_DELAY_US + TONE_DURATION_US < micros()) {
        set_state(WAITING);
      }
      break;
    }
    case WAITING: {
      if (start_time + MAX_MEASURE_US < micros()) {
        set_state(NOT_STARTED);
      }
    }
    default: { break; }
  }
}

void measure(){
  // don't measure in the not_started state, and don't overflow the measurements array
  if (measurement_state != NOT_STARTED && measurement_count < MAX_MEASURE) {
    delayMicroseconds(rand() % MAX_ARB_DELAY); // wait random bit of time to prevent quantization of results
    measurements[measurement_count].time = micros() - start_time; // save the time of this measurement
    measurements[measurement_count].value = analogRead(MIC_PIN); // store the actual measurement
    if (measurement_count > 0) {
      // update the rolling first difference sum, but not for the first item
      measurements[measurement_count].ad1_sum = abs(measurements[measurement_count].value - measurements[measurement_count - 1].value) + measurements[measurement_count - 1].ad1_sum;
    }
    measurement_count++;
  }
}

// logic for what to do during each state transition
void set_state(MEASUREMENT_STATE next_state){
  switch (next_state) {
    case START: {
      measurement_count = 0;
      measurements[0].ad1_sum = 0;
      start_time = micros();
      break;
    }
    case TONE: {
      tone_start = micros() - start_time;
      analogWrite(SPEAKER_PIN, 127);
      break;
    }
    case WAITING: {
      tone_stop = micros() - start_time;
      analogWrite(SPEAKER_PIN, 0);
      break;
    }
    case NOT_STARTED: {
      MEASUREMENT last_measure = measurements[measurement_count - 1];
      float total_time = last_measure.time - measurements[0].time;
      // calculate pedro stat for each measurement and send each record up the serial connection
      for (int i = 0; i < measurement_count; i++){
        measurements[i].pedro = (float)((float)last_measure.ad1_sum * (float)((float)measurements[i].time / total_time)) - (float)measurements[i].ad1_sum;
        Serial.print(measurements[i].time);
        Serial.print(" ");
        Serial.print(measurements[i].ad1_sum);
        Serial.print(" ");
        Serial.print(measurements[i].pedro);
        Serial.print(" ");
        Serial.println(measurements[i].value);
      }

      // scan for the largest pedro stat and save it
      uint16_t largest_index = 0;
      float largest_value = 0;
      for (int i = 0; i < measurement_count; i++){
        if(largest_value < measurements[i].pedro) {
          largest_value = measurements[i].pedro;
          largest_index = i;
        }
      }

      // display the results
      oled.clearMsgArea();
      oled.print("max pedro = ");
      oled.println(largest_value);
      oled.print("delta = ");
      oled.print(measurements[largest_index].time - tone_start);
      oled.println("us");
      oled.display();
      break;
    }
    default: break;
  }
  measurement_state = next_state;

}
