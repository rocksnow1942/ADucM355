"""
A simple script to continuously run SWV on pico

"""


import serial
from serial.tools import list_ports
import time

def connectPico(port):
    try:
        ser = serial.Serial(
        port = port,
        baudrate=230400,  # Baudrate is 230400 for EmstatPico
        bytesize=serial.EIGHTBITS,  # number of bits per bytes
        parity=serial.PARITY_NONE,  # set parity check: no parity
        stopbits=serial.STOPBITS_ONE,  # number of stop bits
        timeout=1,  # timeout block read
        xonxoff=False,  # disable software flow control
        rtscts=False,  # disable hardware (RTS/CTS) flow control
        dsrdtr=False,  # disable hardware (DSR/DTR) flow control
        write_timeout=2,  # timeout for write is 2 seconds)
        )        
        return ser
    except Exception as e:
        print(f'Connect to {port} error. {e}')
        return None
    

def IsConnected(ser):   
    ser.write(bytes("t\n",  'ascii'))  # write the command
    response = ser.read_until(bytes("*\n", 'ascii'))  # read until *\n
    response = str(response, 'ascii')  # convert bytes to ascii string
    start = response.find('esp')  # check the presents of "esp" in the repsonse  
    if start == -1:  # return if string is found
        return False
    return True


def findPico():
    """
    Finds the communication port for the M355
    :returns: COM for the serial port
    """
    ports = list_ports.comports()
    ports = [i.device for i in ports]
    #print(ports)
    for p in ports:
        ser = connectPico(p)
        if ser and IsConnected(ser):
            return ser
    return None

ser = findPico()

script = """e
var c
var p
var f
var r
set_pgstat_chan 0
set_pgstat_mode 3
set_max_bandwidth 400
set_pot_range -600m 0m
set_autoranging 100u 100u
cell_on
set_e -600m
wait 200m
meas_loop_swv p c f r -600m 0m 5m 50m 100
	pck_start
	pck_add c
	pck_end
endloop
on_finished:
cell_off

"""

if ser:
    print(f'Connected to Pico on {ser.port}')

    # continuously issue SWV comment to Pico
    # set the interval to 3 seconds, 
    # so that pico will do 1 SWV every 3 seconds
    interval = 3 
    count = 1
    while 1:
        print(f'Scan Pico... {count}')
        ser.write(script.encode())
        time.sleep(1)
        res = ser.read_all()
        while res:
            time.sleep(0.5)
            res = ser.read_all()
        time.sleep(interval)
        count += 1
            
        




