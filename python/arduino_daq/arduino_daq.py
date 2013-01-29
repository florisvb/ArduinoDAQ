import serial
import numpy as np
import pickle
import time

class Arduino_DAQ(serial.Serial):
    def __init__ (self, **kwargs):
        super(Arduino_DAQ,self).__init__(**kwargs)
        
    def get_analog_input(self, pin):
        self.write('[%s,%s]\n'%(1,pin))
        
        while 1:
            data = self.readline().strip()

            if data is not None:
                val, millis = data.split(',')
                val = int(val)
                millis = int(millis)
                return val, millis
        
    def save_analog_data(self, pin, time_to_record_data_for_in_sec=3, time_units='sec', save_to_file=False):
        time_start = time.time()
        self.write('[%s,%s]\n'%(1,pin))
                
        val_array = []
        micros_array = []
                    
        while 1:
            data = self.readline().strip()

            if data is not None:
                val, micros = data.split(',')
                val = int(val)
                micros = float(int(micros))
                
                if time_units == 'sec' or time_units=='s':
                    t = micros*1e-6
                elif time_units == 'ms':
                    t = micros*1e-3
                else:
                    t = micros
                    
                
                if micros < 0:
                    if save_to_file == False:
                        return np.array(val_array), np.cumsum(np.array(micros_array))
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
                self.write('[%s,%s]\n'%(2,pin))
                
if __name__ == '__main__':
    daq = Arduino_DAQ(port='/dev/ttyACM0',timeout=1, baudrate=115200)
