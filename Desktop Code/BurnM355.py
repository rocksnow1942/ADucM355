"""
Script to run on M355
to stress test the chip.
"""
import serial
import json
import time
from threading import Thread,get_ident,Lock
import random
from datetime import datetime


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
    print(toW.strip(), end='\r')
    with open(f'./{name}.log','a') as f:
        f.write(toW)

def scanAndLog(m,cmd):
    c = 'empty'
    cmdstr = f"vS= {cmd['vS']:>7}, vE= {cmd['vE']:>7}, vI= {cmd['vI']:>2}, ps= {cmd['ps']:<2}, ch= {cmd['ch']},"
    try:
        t0 = time.perf_counter()
        c = m.json(cmd)['c']
        dt = time.perf_counter() - t0
        time.sleep(0.1)        
        maxC = max(c)
        minC = min(c)
        log(logfile,f"{cmdstr} T= {dt:>5.2f}, maxC= {maxC:>10.3f}, minC= {minC:>10.3f}")
    except Exception as e:
        log(logfile,f"{cmdstr} Error: {e}, Data = {c}")
        
        
def stepSizeScan():    
    for stepSize in range(24): #test for 3 rounds.
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
                        log(logfile,f"Step= {step:<7}, vS= {vS:<7}, vE= {cmd['vE']:<7}, readTime= {dt:<5.2f}s, avgC= {avgC:<10.3f}, maxC= {maxC:<10.3f}, minC= {minC:<10.3f} ")
                    except Exception as e:
                        log(logfile,f"Step = {step}, Error: {e}, Data = {c}")
        
if __name__ == '__main__':    
    # copy this file to Pico with scp:
    """
    scp ./BurnM355.py pi@ams-abe.local:/home/pi/BurnM355.py
    """
    
    m = M355('/dev/serial0')
    
    logfile = 'M355Burn'
    
    cmd = {'vS': -843.5, 'vE': 0, 'vI': 5, 'vA': 100.0, 
             'Hz': 100, 'iS': 100.0, 'vP': -600, 'tP': 100.0, 
             'f': 0, 'r': 0, 'ch': 0, 'ps': 0}
    ff = {"vS":-600,"vE":0,"vI":50,"vA":100,"Hz":100,
            "iS":100,"vP":-600,"tP":1,"f":0,"r":0,"ch":0,"ps":1}

    log(logfile, "="*50)
    log(logfile, "Started, with 3 checking loops and on board LED routines")
    log(logfile, "="*50)
    
    # test parameter
    # on PS0
    # potential range: 590 - 610mV, vS, -700mV to -300mV
    # step size is always 5mV
    # on PS1: 
    # {"vS":-600,"vE":0,"vI":50,"vA":100,"Hz":100,"iS":100,"vP":-600,"tP":1,"f":0,"r":0,"ch":ch,"ps":1}
    # switch channel between each scan. 
    # scan 4 times on PS0 then 1 time on PS1    
    for pR in range(599,601):        
        for vS in range(-500,-490):
            cmd['vS'] = vS
            cmd['vP'] = vS
            cmd['vE'] = vS + pR
            cmd['vI'] = 5             
            for ch in range(4):
                cmd['ch'] = ch
                scanAndLog(m,cmd)
            ff['ch'] = random.randint(0,3)
            scanAndLog(m,ff)
            
            
    log(logfile, "="*50)
    log(logfile, "Done")
    log(logfile, "="*50)
    
    
    
    
    """
    summary analysis
    """
    with open(f'./{logfile}.log') as f:
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
        if 'vS=' in line:
            totalScan+=1
            fields = line.split(',')
            minC = getVal(fields[-1])        
            rT = getTime(fields[-4])
            if rT>2 or minC<-1:
                errors.append(line)
                continue
    
    with open(f'./{logfile}summary.log','w') as f:
        f.write(f'Total Scan {totalScan}\n')
        f.write('\n'.join(errors))
        if (len(errors) == 0):
            f.write('all good. no error')

    