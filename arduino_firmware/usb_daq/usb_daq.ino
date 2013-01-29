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

void setup()
{
  // start the serial for debugging
  Serial.begin(115200);
  
  delay(1000);

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
  
  // turn on streaming
  if (action==1) {
    stream_data = 1;
    initial_time = micros();
    last_time = initial_time;
    data_pin = value;
  }
  
  // turn off streaming
  if (action==2) {
    stream_data = 0;
    Serial << 0 << "," << -1 << endl;
  }
  
  // return one value
  if (action==3) {
    Serial << analogRead(value) << "," << millis() << endl;
  }
  
  // stream data
  if (stream_data==1) {
    tdiff = micros()-last_time;
    Serial << analogRead(data_pin) << "," << tdiff << endl;
    last_time = last_time + tdiff;
  }
  
  action = 0;

}



