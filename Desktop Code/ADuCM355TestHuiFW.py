import serial
import json
import serial.tools.list_ports
import matplotlib.pyplot as plt
import time
import numpy as np
from threading import Thread,get_ident,Lock
import random

from mymodule import ft_decorator,ft

def measure(para):
    t0 = time.perf_counter()
    res = m.json(para)
    print(f'Measure take {time.perf_counter() - t0:.2f}s')
    print(f"Avergage current {np.array(res['c']).mean():.4f}uA")
    return res,para
    
def plotRes(R):
    res,para = R
    f = np.array( res.get('f',[]))
    r = np.array(res.get('r',[]))
    c = np.array(res['c'])
    v = np.linspace(para['vS'],para['vE'],len(c))
    s=0
    fig,ax = plt.subplots()
    ax.plot(v[s:len(f)],f[s:],label='forward')
    ax.plot(v[s:len(f)],r[s:],label='reverse')
    ax.plot(v[s:],c[s:],label='delta')
    ax.legend()
    
    plt.tight_layout()

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
        self.picoLock = Lock()
        self.ser = serial.Serial(port=port,  # set the port
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
    def __repr__(self):
        return 'M355'
    def _write(self,string):
        # print(f'{time.monotonic():10.2f} write:',string)
        self.ser.write(string.encode())
    def _read(self):
        return self.ser.read_all()        
    def close(self):
        self.ser.close()
    def _flush(self):       
        res = self._read()
        while res:
            time.sleep(0.01)        
            res = self._read()
    
    @ft_decorator(1)
    def json(self,data,wait=0.1):
        """
        Always use json method to read and write data to M355.
        if didn't get data before timeout, return {error:}.
        """
        if not self.picoLock.acquire(timeout=10):
            print('cannot acquire lock')
            return {"error": 1}
        self._flush()
        self._write(self._encode(data))        
        res = self._read_until('*')
        try:
            result = json.loads(res)
        except Exception as e:
            print(f"M355.json decode error:{e}, cmd={data}, data = {res[-50:]}")
            result = {'error': 2}
        self.picoLock.release()
        return result
        
    def _encode(self,data):
        return (json.dumps(data,separators=(',',':')) + '*')
        
    def _read_until(self,char='*',timeout=30):
        res = ''
        t0=time.monotonic()
        while 1:
            d = self.ser.read_all().decode()            
            if d:
                res += d
                if d[-1]==char:
                    break
            # time.sleep(0.1)
            if time.monotonic() - t0 > timeout:
                break
        return res[0:-1] # remove the last '*' character



m = M355(findComPort())


m.read().decode(errors='ignore')

print(rr.decode(errors='ignore'))


def writeStuff(m):
    res = m.json({"cI":0})
    res = m.json({"vS":-600,"vE":0,"vI":5,"vA":100,"Hz":100,"iS":100,"vP":-600,"tP":100,"f":0,"r":0,"ch":4,"ps":1})
    m.json({'s':1})
    
    


rr.decode().split('*')


mpara = {"vS":-600,"vE":0,"vI":5,"vA":100,"Hz":100,"iS":100,"vP":-600,"tP":100,"f":0,"r":0,"ch":4,"ps":1}

res = m.json(mpara)


res = m.json({"cI":0})
print(res)
res = m.json(mpara)
print(res)
print(m.json({'s':1}))



for i in range(20):
    m.write(m.encode({"cI":0}))
    m.write(m.encode(mpara))
    m.write(m.encode({'s':1}))
    
time.sleep(1)
m.json({'s':1})
    
m.json({'cI':1})
    
ft(m.json,args=({'s':1}))

ft(m.json,args=(mpara,),number=1)

m.json(mpara)


def writeStuff(m):
    for i in range(10):                
        res = m.json({"vS":-600,"vE":0,"vI":5,"vA":100,"Hz":100,"iS":100,"vP":-600,"tP":100,"f":0,"r":0,"ch":4,"ps":1})
        print(get_ident(), f'read result length: {len(res.get("c",[]))}')
        
        

ws = []
for i in range(10):
    t = Thread(target=writeStuff,args=(m,))
    t.start()
    ws.append(t)

ws
m.json({'s':1})



# send status query, should respond with {status:1}
print(m.json({'s':0},0.1))

print(m.json({'v':0},0.1))

print(m.json({'led':0},0.1))



res,para = measure(( dict(
vS= -800,
vE= -400,
vI=50,
vA=100,
Hz=100,
iS=100, #uA
vP=0,
tP=0,
f=1, # flag for forward current
r=1,
ch= 0,     #   mux select pins (0-3)
ps = 0,
) # flag for forward current

))

plotRes((res,para)) 
print( f"Resistance {(np.arange(-800,-401,50) / np.array(res['f'])).mean()}" )



cmd = m.encode({'runqc':1})

ffpara = dict(
vS= -800,
vE= -400,
vI=50,
vA=100,
Hz=100,
iS=100, #uA
vP=0,
tP=0,
f=1, # flag for forward current
r=1,
ch= 0,     #   mux select pins (0-3)
ps = 0,
)

cmd = m.encode(ffpara)


for i in range(8):
    ffpara['ch'] = i // 2
    ffpara['ps'] = i % 2
    cmd = m.encode(ffpara)
    m.write(cmd)
    time.sleep(1)
    
m.write(cmd)

res = m.read()

len(res)

print(res.decode())

res.decode().split('*')

res.decode().split('*')

data = json.loads(res.decode().split('*')[1])

plt.plot(np.linspace(-800,-400,8),data['f'])

m.read()



# run 1 scan on potentiostat 2 first
para["channel"] = 2
res0 = m.json(para)


# then run 10 repeated scan on potentiostat 1
count = 10
for i in range(count):
    #then run another scan on potentiostat 1
    para["channel"] = 1
    res = m.json(para)
    time.sleep(1)

# 
# f = np.array( res['forwardCurrent'])
# r = np.array(res['reverseCurrent'])
# c = np.array(res['subtractCurrent'])
# 
# v = np.linspace(-600,0,len(f))
# 
# fig,ax = plt.subplots()
# ax.plot(v,f,label='forward')
# ax.plot(v,r,label='reverse')
# ax.plot(v,c,label='delta')
# # ax.set_ylim([-30,30])
# ax.legend()
# 
# # plt.savefig('./355out.png')
# 
# m.close()
# 
# 
# 
# 
# res = m.json(para)
# 
# f = np.array( res['forwardCurrent'])
# r = np.array(res['reverseCurrent'])
# c = np.array(res['subtractCurrent'])
# 
# v = np.linspace(-600,0,len(f))
# 
# fig,ax = plt.subplots()
# ax.plot(v,f,label='forward')
# ax.plot(v,r,label='reverse')
# ax.plot(v,c,label='delta')
# # ax.set_ylim([-30,30])
# ax.legend()












