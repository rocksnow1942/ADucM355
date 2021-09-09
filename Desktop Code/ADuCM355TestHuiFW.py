import serial
import json
import serial.tools.list_ports
import matplotlib.pyplot as plt
import time
import numpy as np
from threading import Thread,get_ident,Lock
import random
from datetime import datetime

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
    
    # @ft_decorator(1)
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
        
    def _read_until(self,char='*',timeout=5):
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
        # print(res)
        return res[0:-1] # remove the last '*' character

def date():
    return datetime.now().strftime('%m/%d %H:%M:%S')

def log(name,text):
    toW = f"{date()} | {text} \n"
    with open(f'./{name}.log','a') as f:
        f.write(toW)
    






m = M355(findComPort())

print(m.json({'s':1}))

print(m.json({'s':1}))



print(m.json({'s':1}))





cmd = {'vS': -843.5, 'vE': 0, 'vI': 5, 'vA': 100.0, 
             'Hz': 100, 'iS': 100.0, 'vP': -600, 'tP': 10.0, 
             'f': 0, 'r': 0, 'ch': 0, 'ps': 0}


step = 57
vS = -100
cmd['vS'] = vS  
cmd['vE'] = vS + step * 5 /2

cmd

cmdStr = m._encode(cmd)

cmdStr

m._write(cmdStr)
res = m._read_until(timeout=5)
print(res)




step = 244
vS = -100
cmd['vS'] = vS  
cmd['vE'] = vS + step * 5 /2
vS

cmdStr = m._encode(cmd)

cmdStr

m._write(cmdStr)
res = m._read_until(timeout=5)
print(res)



json.loads(res)['c']

print(len(json.loads(res.split('*')[1])['c']))

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














"""
=================================================
M355 burnning test
=================================================
"""


m = M355(findComPort())

print(m.json({'s':1}))

cmd = {'vS': -843.5, 'vE': 0, 'vI': 5, 'vA': 100.0, 
             'Hz': 100, 'iS': 100.0, 'vP': -600, 'tP': 10.0, 
             'f': 0, 'r': 0, 'ch': 0, 'ps': 0}


log('M355 Burn', "="*50)
log('M355 Burn', "Started, with 3 checking loops and on board LED routines")
log('M355 Burn', "="*50)
for stepSize in range(6): #test for 3 rounds.
    for vS in range(-600+stepSize,-50+stepSize,37):
        for step in range(4+stepSize,290,7):
            cmd['vP'] = vS
            cmd['vS'] = vS  
            cmd['vE'] = vS + step * 5 /2    
        
            for j in range(2):        
                c = 'empty'
                try:
                    t0 = time.perf_counter()
                    c = m.json(cmd)['c']
                    dt = time.perf_counter() - t0
                    time.sleep(1)
                    avgC = sum(c) / (len(c) + 1)
                    maxC = max(c)
                    minC = min(c)
                    log('M355 Burn',f"Step= {step:<7}, vS= {vS:<7}, vE= {cmd['vE']:<7}, readTime= {dt:<5.2f}s, avgC= {avgC:<10.3f}, maxC= {maxC:<10.3f}, minC= {minC:<10.3f} ")
                except Exception as e:
                    log('M355 Burn',f"Step = {step}, Error: {e}, Data = {c}")

log('M355 Burn', "="*50)
log('M355 Burn', "Done")
log('M355 Burn', "="*50)
                    
                    
"""
=================================================
M355 burnning test
=================================================
"""


"""
summary analysis
"""

file = './M355 burn.log'
file = r"C:\Users\hui\codes\PGHM355burn.log"
with open(file) as f:
    data = f.read().split('\n')

getVal = lambda x:float(x.split('=')[-1])
getTime = lambda x:float(x.split('=')[-1][:-1])

errors = []
totalScan = 0
for line in data:
    if 'Error' in line:
        errors.append(line)
        totalScan+=1
        continue
    if 'Step=' in line:
        totalScan+=1
        fields = line.split(',')
        minC = getVal(fields[-1])        
        rT = getTime(fields[-4])
        if rT>2 or minC<-1:
            errors.append(line)
            continue

with open(f'./summary.log','w') as f:
    f.write(f'Total Scan {totalScan}\n')
    f.write('\n'.join(errors))
    if (len(errors) == 0):
        f.write('all good. no error')

