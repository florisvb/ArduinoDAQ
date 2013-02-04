#!/usr/bin/env python
import roslib
roslib.load_manifest('ros_arduino_daq')
import rospy

from optparse import OptionParser

import serial
import numpy as np
import pickle
import time

from std_msgs.msg import *
from ros_arduino_daq.srv import *

class Arduino_DAQ(serial.Serial):
    def __init__ (self, pin, topic_name, streaming_delay=0, **kwargs):
        print 'Initializing arduino, please wait a few seconds...'
        super(Arduino_DAQ,self).__init__(**kwargs)
        time.sleep(2)
        
        rospy.init_node('arduino_daq', anonymous=True)
        
        self.stream = False
        self.pin = pin
        self.streaming_delay = streaming_delay
        
        controller_name = topic_name + '_control'
        self.service = rospy.Service(controller_name, DAQControl, self.controller_callback)
        
        self.publisher = rospy.Publisher(topic_name, Int32)
        
        print 'ready to stream!'
        while not rospy.is_shutdown():
            self.stream_analog_data()
        
    def controller_callback(self, data):
        if data.action == 1:
            print 'streaming!'
            self.flushInput()
            self.stream = True
            self.time_start = time.time()
            self.write('[%s,%s]\n'%(100,2))
            self.write('[%s,%s]\n'%(101,self.streaming_delay))
            self.write('[%s,%s]\n'%(1,self.pin))
            return 1
        elif data.action == 0:
            print 'stopping streaming!'
            self.stream = False
            self.write('[%s,%s]\n'%(2,self.pin))
            return 0
            
    def stream_analog_data(self):
        if self.stream:
            data = self.readline().strip()
            if data is not None:
                try:
                    val = int(data)
                    self.publisher.publish(val)
                except:
                    print "Bad data -- should be a rare occurence, data looked like: ", data
        else:
            time.sleep(.01)
            
                
if __name__ == '__main__':
    
    parser = OptionParser()
    parser.add_option("--pin", type="int", dest="pin", default=0,
                        help="analog pin on arduino to stream")
    parser.add_option("--port", type="string", dest="port", default=None,
                        help="port where arduino is connected to, look at /dev/serial/by-id/")
    parser.add_option("--topic", type="string", dest="topic", default="arduino_data",
                        help="topic name you would like to publish to")
    parser.add_option("--streaming-delay", type="int", dest="streaming_delay", default=0,
                        help="delay in streaming, microseconds (approximately corresponds to period)")
    (options, args) = parser.parse_args()
    
    topic_control_name = options.topic + '_control'
    
    print 
    print
    print 'Publishing arduino data from analog pin ', options.pin, '\nto rostopic: ', options.topic
    print
    print 'Toggle data streaming using ROS service: ', topic_control_name
    print 'Toggle on  from command line: $ rosservice call', topic_control_name, '1'
    print 'Toggle off from command line: $ rosservice call', topic_control_name, '0'
    
    daq = Arduino_DAQ(options.pin, options.topic, streaming_delay=options.streaming_delay, port=options.port,timeout=1, baudrate=115200)








    
