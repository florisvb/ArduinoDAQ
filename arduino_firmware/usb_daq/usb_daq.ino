// Firmware for controlling a Stepper Motor over USB, intended for use with ROS
// Floris van Breugel 2012

// Written for Arduino Uno


#include "Streaming.h"
#include "SerialReceiver.h"

SerialReceiver receiver;

// Variables used for decoding and delimiting TCP msg
long action;
long value;

bool stream_data = 0;
long data_pin = 0;
long time_to_record_data = 0;
long initial_time = 0;
long last_time = 0;
long tdiff = 0;
long ddiff = 0;
long time_to_stream_for = 100;
long data = 0;
long last_data = 0;
long stream_method = 1;

void setup()
{
  // start the serial for debugging
  Serial.begin(115200);
  delay(1000);
  
  stream_data = 0;
  data_pin = 0;
  time_to_record_data = 0;
  initial_time = 0;
  last_time = 0;
  tdiff = 0;
  ddiff = 0;
  time_to_stream_for = 100;
  data = 0;
  last_data = 0;
  stream_method = 1;

}

void loop()
{
  
  while (Serial.available() > 0) {
    receiver.process(Serial.read());
    if (receiver.messageReady()) {
        action = receiver.readLong(0);
        value = receiver.readLong(1);
        receiver.reset();
    }
  }
  
  if (action!=0) { 
    perform_action(action, value);
  }
  
  // stream data
  if (stream_data==1) {
    
    if (stream_method == 1) {
      tdiff = micros()-last_time;
      data = analogRead(data_pin);
      ddiff = data-last_data;
      Serial << ddiff << "," << tdiff << endl;
      last_time = last_time + tdiff;
      last_data = data;
    }
    
    if (stream_method==0) {
      Serial << analogRead(data_pin) << "," << micros()-initial_time << endl;
    }
    
    if (stream_method == 2) {
      Serial << analogRead(data_pin) << endl;
    }
   
  }
  
  action = 0;

}


void perform_action(long action, long value) {
  switch (action) {
    case 1: turn_on_streaming(value); break;
    case 2: turn_off_streaming(); break;
    case 3: return_analog_value(value); break;
    case 100: set_streaming_method(value); break;
    default: return;
  }
}

void turn_on_streaming(long value) {
  stream_data = 1;
  initial_time = micros();
  last_time = initial_time;
  last_data = 0;
  data_pin = value;
}

void turn_off_streaming() {
  stream_data = 0;
  Serial << 0 << "," << -10000 << endl;
}
  
void return_analog_value(long value) {
  Serial << analogRead(value) << "," << millis() << endl;
}

void set_streaming_method(long value) {
  stream_method = value;
}
