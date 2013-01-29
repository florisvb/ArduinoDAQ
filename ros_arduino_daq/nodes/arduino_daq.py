#!/usr/bin/env python
import roslib
roslib.load_manifest('ros_arduino_daq')
import rospy

import serial
import numpy as np
import pickle
import time

from std_msgs.msg import *

class Arduino_DAQ(serial.Serial):
    def __init__ (self, pin, topic_name, **kwargs):
        super(Arduino_DAQ,self).__init__(**kwargs)
        
        rospy.init_node('arduino_daq', anonymous=True)
        
        self.stream = False
        self.pin = pin
        
        subscriber_name = topic_name + '_control'
        self.subscriber = rospy.Subscriber(subscriber_name,Int32,self.subscriber_callback)
        self.publisher = rospy.Publisher(topic_name, Float32)
        
        while not rospy.is_shutdown():
            self.stream_analog_data()
        
    def subscriber_callback(self, data):
        if data.data:
            self.stream = True
            self.time_start = time.time()
            self.write('[%s,%s]\n'%(1,pin))
        else:
            self.stream = False
            self.write('[%s,%s]\n'%(2,pin))
                
    def stream_analog_data(self):
        if self.stream:
            data = self.readline().strip()
            if data is not None:
                try:
                    val, micros = data.split(',')
                    val = int(val)
                    self.publisher.publish(val)
                except:
                    print "Bad data -- should be a rare occurence, data looked like: ", data
        else:
            time.sleep(.01)
            
                
if __name__ == '__main__':
    pin = 0
    topic_name = 'arduino_data'
    daq = Arduino_DAQ(pin, topic_name, port='/dev/ttyACM0',timeout=1, baudrate=115200)
    
    # to start streaming from the command line type:
    #   $ rostopic pub /arduino_data_control std_msgs/Int32 1 -1
    #
    # to stop streaming from the command line type:
    #   $ rostopic pub /arduino_data_control std_msgs/Int32 0 -1
