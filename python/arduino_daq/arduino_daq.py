import serial
import numpy as np
import pickle
import time

import re
import subprocess

def find_port_from_serial_number(s):

    df = subprocess.check_output("lsusb -vvv", shell=True)
    device_strings = df.split('\n\n')
    
    devices = []
    
    device_header_pattern_tagged = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
    device_header_pattern_untagged = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)$", re.I)
    for device_string in device_strings:
        device_string_lines = device_string.split('\n')
        for line in device_string_lines:
            info = device_header_pattern_tagged.match(line.strip())
            if info is None:
                info = device_header_pattern_untagged.match(line.strip())
            if info:
                dinfo = info.groupdict()
                dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
            elif 'iSerial' in line:
                info = (line.strip()).split(' ')
                serial = info[-1]
                dinfo.setdefault('serial', serial)
        devices.append(dinfo)
                
        
    
    return devices



class Arduino_DAQ(serial.Serial):
    def __init__ (self, **kwargs):
        super(Arduino_DAQ,self).__init__(**kwargs)
        
    def get_analog_input(self, pin):
        self.flushInput()
        self.write('[%s,%s]\n'%(3,pin))
        
        while 1:
            data = self.readline().strip()

            if data is not None:
                val, millis = data.split(',')
                val = int(val)
                millis = int(millis)
                return val, millis
        
    def save_analog_data(self, pin, time_to_record_data_for_in_sec=3, time_units='sec', save_to_file=False, skip_bad_data=True, stream_difference=False):
        self.flushInput()
        
        if stream_difference:
            self.write('[%s,%s]\n'%(100,1))
        else:
            self.write('[%s,%s]\n'%(100,0))
    
        val_array = []
        micros_array = []
        time_start = time.time()
        self.write('[%s,%s]\n'%(1,pin))
                
                    
        while 1:
            data = self.readline().strip()

            if data is not None and len(data) > 0:
                print data
                try:
                    val, micros = data.split(',')
                except:
                    # bad data
                    if skip_bad_data:
                        continue
                    else:
                        val = int(data)[0]
                        micros = int(data)[1:]
                val = int(val)
                micros = float(int(micros))
                
                if time_units == 'sec' or time_units=='s':
                    t = micros*1e-6
                elif time_units == 'ms':
                    t = micros*1e-3
                else:
                    t = micros
                    
                if micros <= -10000:
                    self.flushInput()
                    if save_to_file == False:
                        if stream_difference:
                            return np.cumsum(np.array(val_array)), (np.cumsum(np.array(micros_array)))
                        else:
                            return (np.array(val_array)), (np.array(micros_array))
                    else:
                        if type(save_to_file) is not str:
                            save_to_file = time.strftime("arduino_daq_%Y%m%d_%H%M%S",time.localtime())
                        f = open(save_to_file)
                        data = {'value': np.array(val_array), 'time': np.cumsum(np.array(micros_array)), 'time_units': time_units}
                        pickle.dump(data, f)
                else:
                    val_array.append(val)
                    micros_array.append(t)
            
            if time.time() - time_start > time_to_record_data_for_in_sec:
                print 'turning off'
                self.write('[%s,%s]\n'%(2,pin))
                
                
if __name__ == '__main__':
    daq = Arduino_DAQ(port='/dev/ttyACM0',timeout=1, baudrate=115200)
