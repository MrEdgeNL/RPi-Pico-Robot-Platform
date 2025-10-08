import _thread
from time import sleep_ms
import usocket as socket
import uselect as select
import network
from machine import UART
#import sys
import lib.secrets as secrets
wlan = network.WLAN(network.STA_IF)


# -----------------------------
# Network object 
# -----------------------------
def connect_to_lan():
    wlan.active(True)
    wlan.connect(secrets.SSID,secrets.PASSWORD)

    # Wait for connect or fail
    max_wait = 10
    while max_wait > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        max_wait -= 1
        print('>pico: Waiting for connection..')
        sleep_ms(1500)

    # Handle connection error
    if wlan.status() != 3:
        #raise RuntimeError('>pico: Network connection failed!')
        print( '>pico: Network connection failed!' )
        return "none"   #return 'none'
    else:
        status = wlan.ifconfig()
        print( '>pico: Connected, device IP: ' + status[0] )  

    return status[0]


# -----------------------------
# Message abstraction 
# Arguments are created in list style:
#   if an empty string argument is passed, this argument will not be reconized and deleted.
# -----------------------------
class Message:
    def __init__(self, command_raw):
        parts = command_raw.strip().split()
        self.target = parts[0].strip().upper()
        if len(parts) > 1:
            self.command = parts[1].strip().lower()
        else:
            self.command = "??"
        if len(parts) > 2:  # Join the rest and split by comma, stripping extra spaces            
            arg_str = " ".join(parts[2:])
            self.args = [a.strip().lower() for a in arg_str.split(",") if a.strip()]
        else:
            self.args = []

    def __str__(self):
        return f"{self.target} {self.command} " + ", ".join(str(x) for x in self.args)


# -----------------------------
# Simple thread-safe queue
# -----------------------------
class SimpleQueue:
    def __init__(self, maxsize=8):
        self._data = []
        self._maxsize = maxsize
        self._lock = _thread.allocate_lock()

    def put(self, item):
        with self._lock:
            if len(self._data) < self._maxsize:
                self._data.append(item)
            else:
                self._data.pop(0)
                self._data.append(item)

    def get(self):
        with self._lock:
            if self._data:
                return self._data.pop(0)
            return None

    def empty(self):
        with self._lock:
            return len(self._data) == 0
        

# -----------------------------
# Comm class (Core 1)
# -----------------------------
class Comm:
    def __init__(self, input_q, output_q):
        self.in_q = input_q         # message que: PC -> Robot
        self.out_q = output_q       # message que: Robot -> PC        
        self.port_recv = 5005       # port PC → Pico
        self.port_send = 5006       # port Pico → PC
        self._ip = "unkown"         # IP this device in LAN
        self._addrrecv = ""         # IP from sending PC
        self._inf_loop = False
        self.use_uart0 = False


    def loop(self):
        self._ip = connect_to_lan()
        if self._ip == "none":
            self.use_uart0 = True
        if self.use_uart0:            # USB serial
            self.uart = UART(0, 115200)
            print(">pico: USB communication enabled.")
        
        # Update robot with settings:
        self.in_q.put(f"RBT ip {self._ip}")
        self.in_q.put(f"RBT uart0 {self.use_uart0}")

        # Start communication thread:
        _thread.start_new_thread(self._thread_loop, ())


    def _thread_loop(self):
        if self._ip != "none":
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((self._ip, self.port_recv)) #PICO needs an IP adres and recieve port doesnot work?
            poll = select.poll()
            poll.register(s, select.POLLIN)
                
        self._inf_loop = True        
        print(">pico: READY for your command..")

        while self._inf_loop:            
            # Non-blocking poll, check INCOMING data from socket:
            if self._ip != "none":
                events = poll.poll(50)  # 50 ms
                if events:                                
                    try:                    
                        data, self._addrrecv = s.recvfrom(256)
                        msg = data.decode()
                        print(f">pico recv upd: {msg}")
                        #s.sendto(  ("echo: " + msg).encode(), (self._addrrecv[0], self.port_send) )                    
                        self.in_q.put(msg)   #if not msg.startswith("COM"): there are no com-commands.
                    except Exception:
                        print(f">pico receving failed: {msg}")
            
            # Check INCOMING data from uart0:
            if self.use_uart0:
                if self.uart.any():
                    print(f">pico recv uart: {msg}")
                    msg = self.uart.readline()
                    self.in_q.put(msg)
                    sleep_ms(100)
            
            
            # SEND outgoing messages to PC (print them)
            while not self.out_q.empty():                    
                msg = self.out_q.get()
                #print(">pico sending: " + msg)
                if self.use_uart0:
                    self.uart.write(msg + "\n")
                if self._addrrecv:                                    
                    try:
                        #print(f">pico send upd: {msg} to: {self._addrrecv[0]}, port: {self.port_send}.")
                        s.sendto(msg.encode(), (self._addrrecv[0], self.port_send))
                    except Exception:
                        pass
            
        # Close socket when done:
        s.close()
    
    def close(self):
        self._inf_loop = False



# ---------------------------------------------------------------------
# Example usage / wiring
# ---------------------------------------------------------------------
if __name__ == "__main__":    
    m = Message("RBT Lan ,fdg  ")
    print(f"> Message str: '{m}'")
    print(f"> Command: '{m.target}' '{m.command}' '{m.args}' ")
    
    print(f"> Having {len(m.args)} args.")