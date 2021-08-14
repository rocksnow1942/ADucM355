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
    with open(f'./{name}.log','a') as f:
        f.write(toW)
        
        
        
if __name__ == '__main__':    
    m = M355('/dev/serial0')
    
    logfile = 'M355Burn'
    
    cmd = {'vS': -843.5, 'vE': 0, 'vI': 5, 'vA': 100.0, 
             'Hz': 100, 'iS': 100.0, 'vP': -600, 'tP': 10.0, 
             'f': 0, 'r': 0, 'ch': 0, 'ps': 0}

    log(logfile, "="*50)
    log(logfile, "Started, with 3 checking loops and on board LED routines")
    log(logfile, "="*50)
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
        if 'Step=' in line:
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

    