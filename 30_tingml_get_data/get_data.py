import serial
from datetime import datetime  
from multiprocessing import Process, Event, Queue  
import signal
import sys
import numpy as np
import glob
import os

MAX_NUM = 200
class SerialControl(object):
    def __init__(self, port='/dev/ttyUSB0', direction='none'):
        ports = glob.glob('/dev/ttyUSB*')  
        if len(ports) == 0:
            print("No Serial Port")
            sys.exit(0)
        port = ports[0]        
        self.serial = serial.Serial(port, 115200, timeout=1)
        if self.serial.isOpen():
            print(f"{port} is connected")
        self.history = []
        self.direction = direction
        
    def get_log(self):
        sub_history = None
        history = []
        while len(history) < MAX_NUM:
            try:
                response = ""
                try:
                    response = self.serial.readline().decode()
                except Exception as e:
                    print(e)
                    pass
                if response == "":
                    continue
                if response[:6] == "[main]":
                    sub_history = dict()
                    response_list = response.split(':')[1:-1]
                    sub_history = [float(x) for x in response_list]                    
                    # sub_history = response_list
                    print(len(history), sub_history)
                    history.append(sub_history)
            except UnicodeDecodeError:
                sub_history = None
            except ValueError:
                sub_history = None
            except Exception as e:
                print(e)
                sub_history = None
        from datetime import datetime  
        # Get the current date and time  
        now = datetime.now()  
        # Format the date and time and remove spaces  
        dir_path = f"../10_raw_data/"
        os.makedirs(dir_path, exist_ok=True)
        file_name = f"../10_raw_data/{self.direction}_"
        file_name += now.strftime("%Y-%m-%d_%H-%M-%S") 
        file_name += ".npy" 
        history = np.array(history)
        np.save(file_name, history)
        print(f"Save to {file_name}")
        return sub_history
    
    def save(self, history):
        from datetime import datetime  
        # Get the current date and time  
        now = datetime.now()  
        # Format the date and time and remove spaces  
        file_name = f"../30_data/{self.direction}_"
        file_name += now.strftime("%Y-%m-%d_%H-%M-%S") 
        file_name += ".npy" 
        historys = np.array(history)
        print(f"Save to {file_name}")
        
        np.save(file_name, historys)
        print(f"Save to {file_name}")
            
    def __del__(self):
        print(f"DEL")    
        self.serial.close()


def main():
    _directions = ['Stationary', 'Tilted', 'Rotating', 'Moving']
    control = SerialControl(direction=_directions[3])
    control.get_log()
    del control
if __name__=="__main__":
    main()
