import serial
import json
import serial.tools.list_ports
import matplotlib.pyplot as plt
import time
import numpy as np
from collections import namedtuple

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
    
class M355:
    def __init__(self,port):
        self.ser = serial.Serial(port=port,
                                baudrate=115200,
                                timeout=5)
    def write(self,string):
        self.ser.write(string.encode())
    def read(self):
        return self.ser.read_all()        
    def close(self):
        self.ser.close()
    def json(self,data,wait=0.1):
        self.write(self.encode(data))
        time.sleep(wait)
        return json.loads(self.read_until('*'))
    def encode(self,data):
        return (json.dumps(data,separators=(',',':')) + '*')
        
    def read_until(self,char='*',timeout=5):
        res = ''
        t0=time.monotonic()
        while 1:
            d = self.ser.read_all().decode()
            if d:
                res += d
                if d[-1]==char:
                    break            
            time.sleep(0.1)
            if time.monotonic() - t0 > timeout:
                raise RuntimeError ('time out')
                
        return res[0:-1]



m = M355(findComPort())

# m.close()

print(m.json({'status':0},0.1))

print(m.json({'chipInserted':1},0.1))

# 
# 
# para = dict(
# vScale              =   .001,   #   mV
# vStart              =   -200.0,    #   0mV
# vEnd                =   -400.0, #   -600mV
# vIncrement          =   -5.0,   #   5mV
# vAmplitude          =   100.0,  #   100mV
# freqHz              =   100.0,  #   100Hz
# iScale              =   1e-06,  #   uA
# maxCurrent          =   50.0,   #   50uA
# vPretreatment       =   0.0,    #   0uA
# secsPretreatment    =   0.2,    #   200mS
# iForwardRecv        =   1,      #   Flag for having forward current reported
# iReverseRecv        =   1,      #   Flag for having subtracted current reported
# channel             =   0,      #   potential channel (0-1)
# muxSelect           =   0       #   mux select pins (0-3)
# )


para=dict(
vScale=1e-3, #mV
vStart=-600,
vEnd=0,
vIncrement=5,
vAmplitude=200,
freqHz=100,
iScale=.000001, #uA
maxCurrent=100, #uA
vPretreatment=0,
secsPretreatment=0,
iForwardRecv=1, # flag for forward current
iReverseRecv=1,
muxSelect= 3      #   mux select pins (0-3)
) # flag for forward current



res = m.json(para)


f = np.array( res['forwardCurrent'])
r = np.array(res['reverseCurrent'])
c = np.array(res['subtractCurrent'])

v = np.linspace(-600,0,len(f))



fig,ax = plt.subplots()
ax.plot(v,f,label='forward')
ax.plot(v,r,label='reverse')
ax.plot(v,c,label='delta')
ax.legend()




f = res['forwardCurrent']
r = res['reverseCurrent']
c = res['subtractCurrent']

 

fig,ax = plt.subplots()
ax.plot(np.linspace(-500,0,len(f)),f,label='forward')
ax.plot(np.linspace(-500,0,len(r)),r,label='reverse')
ax.plot(np.linspace(-500,0,len(c)),c,label='delta')
ax.legend()





















