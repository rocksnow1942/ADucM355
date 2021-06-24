"""
A simple script to continuously run SWV on pico

Package required to run this script:

pyserial: https://pypi.org/project/pyserial/

"""


import serial
from serial.tools import list_ports
import time

def connectPico(port):
    "This function connects to Pico on a given port"
    try:
        ser = serial.Serial(
            port = port, # this is the port, on my PC, the port is 'COM7'. 
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
    """
    This function test for if pico is connected to this serial port object.
    It writes command 't\n' to the port. 
    If pico is connected, pico will respond with esp....,
    And I check if the response is the same as expected.
    If it is the same, then it is connected correctly.
    """
    ser.write(bytes("t\n",  'ascii'))  # write the command
    time.sleep(0.1)
    response = ser.read_until(bytes("*\n", 'ascii'))  # read until *\n
    response = str(response, 'ascii')  # convert bytes to ascii string
    start = response.find('esp')  # check the presents of "esp" in the repsonse  
    if start == -1:  # return if string is found
        return False
    return True


def findPico():
    """
    Finds the communication port for the Pico,
    then establish a connection to the pico using serial library.
    returns the connected serial object.
    The serial object can be used to communicate with Pico
    """
    ports = list_ports.comports()
    ports = [i.device for i in ports]
    #print(ports)
    for p in ports:
        ser = connectPico(p)
        time.sleep(0.1)
        if ser and IsConnected(ser):
            return ser
    return None


# this finds the pico and connect to it.
ser = findPico()





# Uncomment the following block to manually connect to the pico port:
#============================================================================
# ser = serial.Serial(
#     port = "COM7",
#     baudrate=230400, 
#     bytesize=serial.EIGHTBITS, 
#     parity=serial.PARITY_NONE,  
#     stopbits=serial.STOPBITS_ONE, 
#     timeout=1,  
#     xonxoff=False, 
#     rtscts=False,  
#     dsrdtr=False,  
#     write_timeout=2 )
#============================================================================




# script is a string to be written to the Pico via serial port.
# this script issues the command to perform a SWV measurement.
# the Vbias will be from -600mV to 0mV,
# Amplitude of the square wave is set to 50mV,
# frequency of square wave is 100Hz
# Ramp increment of square wave is 5mV.
# this program also set a pretreatment Vbias at -600mV for 200ms (milliseconds)
# after SWV is done, it will turn off the cell.
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



"""
To Fermi:
You can change the interval and count to different numbers,
so that it suits your need for your experiment.
"""


if ser: # here checks for if the pico is connected.
    print(f'Connected to Pico on {ser.port}')

    # continuously issue SWV comment to Pico
    # set the interval to 3 seconds, 
    # so that pico will do 1 SWV, then wait for 3 seconds for to do another SWV.
    interval = 3 # interval of the scan in seconds

    # it will repeat for [count] times.
    count = 1000 # how many total scan it will repeat. You can set it to very large number so it will keep running.


    for i in range(count):
        print(f'Scan Pico ... {i+1} / {count}') 
        ser.write(script.encode("ascii")) # writes script to pico 
        time.sleep(1) # wait for 1 second
        res = ser.read_all() # read all data returned from pico.
        while res:  # keep reading until all data is returned.
            time.sleep(0.5)
            res = ser.read_all()
        time.sleep(interval) # sleep for [interval] seconds
else:
    print('Pico is not connected.')
            
        