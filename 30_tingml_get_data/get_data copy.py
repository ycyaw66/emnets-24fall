import serial
from datetime import datetime  
from multiprocessing import Process, Event, Queue  
import signal
import sys
import numpy as np
MAX_NUM = 200
class SerialControl(object):
    def __init__(self, port='/dev/ttyUSB0'):
        self.serial = serial.Serial(port, 115200, timeout=1)
        if self.serial.isOpen():
            print(f"{port} is connected")
        self.history = []
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
                    print(sub_history)
                    history.append(history)
            except UnicodeDecodeError:
                sub_history = None
            except ValueError:
                sub_history = None
            except Exception as e:
                print(e)
                sub_history = None
                
        if sub_history != None:
            print(sub_history)
        return sub_history
    
    def send(self):
        command = "send\n"
        self.serial.write(command.encode())
            
    def __del__(self):
        print(f"DEL")    
        self.serial.close()

def add_log(direction, interrupt_event):
    import glob
    from datetime import datetime  
  
    # Get the current date and time  
    now = datetime.now()  
    
    # Format the date and time and remove spaces  
    file_name = f"../30_data/{direction}_"
    file_name += now.strftime("%Y-%m-%d_%H-%M-%S") 
    file_name += ".npy" 
    ports = glob.glob('/dev/ttyUSB*')  
    if len(ports) == 0:
        return
    control = SerialControl(ports[0])
    historys = []
    while True:
        try:
            sub_history = control.get_log()
            historys.append(sub_history)
            if interrupt_event.is_set():
                break
        except KeyboardInterrupt:
            print("[add_log] interrupt exit successfully!")
            break
    del control
    historys = np.array(historys)
    np.save(file_name, historys)
    print(f"Save to {file_name}")

def main():
    interrupt_event = Event()
    get_logger = Process(target=add_log, args=("none", interrupt_event))
    get_logger.start()
    def handler(signum, frame):  
        print("Time's up!")  
        interrupt_event.set()  
        get_logger.join()  
        return 
    signal.signal(signal.SIGALRM, handler)  
    signal.alarm(5)
    try:    
        get_logger.join()
    except KeyboardInterrupt:
        print("Interrupt")
        interrupt_event.set()
        get_logger.join()

if __name__=="__main__":
    main()