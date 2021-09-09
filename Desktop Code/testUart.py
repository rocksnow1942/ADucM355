"""
Script to run on M355
to stress test the chip.
"""
import serial
import serial.tools.list_ports




from datetime import datetime

def findComPort():
    """
    Finds the communication port for the M355

    :returns: COM for the serial port
    """
    ports = serial.tools.list_ports.comports()
    #print(ports)
    if(ports):
        for port in ports:
            #print(port.description)
            if port.description[0:15] == 'USB Serial Port':
                print("Found ADuCM355 @ " +  str(port.device))
                return port.device        
    return None
    
    
com = findComPort()    
ser = serial.Serial(port=com,  # set the port
    baudrate=115200,  # Baudrate is 230400 for EmstatPico
    bytesize=serial.EIGHTBITS,  # number of bits per bytes
    parity=serial.PARITY_NONE,  # set parity check: no parity
    stopbits=serial.STOPBITS_ONE,  # number of stop bits
    timeout=1,  # timeout block read
    xonxoff=False,  # disable software flow control
    rtscts=False,  # disable hardware (RTS/CTS) flow control
    dsrdtr=False,  # disable hardware (DSR/DTR) flow control
    write_timeout=2,  # timeout for write is 2 seconds)
    )
    
    
ser.write('hello from PC'.encode())


ser.read_all()












