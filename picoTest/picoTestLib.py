"""
Simplified pico library for stand alone test on computer
- Hui Kang 20210520
"""

import serial
import numpy as np
import time
import sys
from serial.tools import list_ports


def findComPort():
    """
    Finds the communication port for the M355

    :returns: COM for the serial port
    """
    ports = list_ports.comports()
    #print(ports)
    if(ports):
        for port in ports:
            #print(port.description)
            if port.description[0:15] == 'USB Serial Port':
                print("Found Pico @ " +  str(port.device))
                return port.device        
    return None

# template for use in raspberry pico
no_channel_template = """e
var c
var p
var f
var r
set_pgstat_chan 0
set_pgstat_mode 3
set_max_bandwidth {Freq*4}
set_pot_range {E_begin} {E_end}
set_autoranging {crMin} {crMax}
cell_on
set_e {E_begin}
wait {waitTime}
meas_loop_swv p c f r {E_begin} {E_end} {E_step} {E_amp} {Freq}
	pck_start
	pck_add p
	pck_add c
	pck_end
endloop
on_finished:
cell_off

"""

measure_channel_template = """e
var c
var p
var f
var r
{setPin}
set_pgstat_chan 0
set_pgstat_mode 3
set_max_bandwidth {Freq*4}
set_pot_range {E_begin} {E_end}
set_autoranging {crMin} {crMax}
cell_on
set_e {E_begin}
wait {waitTime}
meas_loop_swv p c f r {E_begin} {E_end} {E_step} {E_amp} {Freq}
	pck_start
	pck_add c
	pck_end
endloop
on_finished:
cell_off

"""

# dictionary list for conversion of the SI prefixes
sip_factor = {
    'a': 1e-18,
    'f': 1e-15,
    'p': 1e-12,
    'n': 1e-09,
    'u': 1e-06,
    'm': 0.001,
    ' ': 1.0,
    'i': 1.0,
    'k': 1000.0,
    'M': 1000000.0,
    'G': 1000000000.0,
    'T': 1000000000000.0,
    'P': 1000000000000000.0,
    'E': 1e+18,
}

# dictionary containing all used var types for MethodSCRIPT
ms_var_types = [{"vt": "aa", "type": "unknown", "unit": " "},
                {"vt": "ab", "type": "WE vs RE potential", "unit": "V"},
                {"vt": "ac", "type": "CE potential", "unit": "V"},
                #{"vt":"ad", "type": "WE potential"        , "unit" : "V" },
                {"vt": "ae", "type": "RE potential", "unit": "V"},
                {"vt": "ag", "type": "WE vs CE potential", "unit": "V"},

                {"vt": "as", "type": "AIN0 potential", "unit": "V"},
                {"vt": "at", "type": "AIN1 potential", "unit": "V"},
                {"vt": "au", "type": "AIN2 potential", "unit": "V"},

                {"vt": "ba", "type": "WE current", "unit": "A"},

                {"vt": "ca", "type": "Phase", "unit": "Degrees"},
                {"vt": "cb", "type": "Impedance", "unit": "Ohm"},
                {"vt": "cc", "type": "ZReal", "unit": "Ohm"},
                {"vt": "cd", "type": "ZImag", "unit": "Ohm"},

                {"vt": "da", "type": "Applied potential", "unit": "V"},
                {"vt": "db", "type": "Applied current", "unit": "A"},
                {"vt": "dc", "type": "Applied frequency", "unit": "Hz"},
                {"vt": "dd", "type": "Applied AC amplitude", "unit": "Vrms"},

                {"vt": "eb", "type": "Time", "unit": "s"},
                {"vt": "ec", "type": "Pin mask", "unit": " "},

                {"vt": "ja", "type": "Misc. generic 1", "unit": " "},
                {"vt": "jb", "type": "Misc. generic 2", "unit": " "},
                {"vt": "jc", "type": "Misc. generic 3", "unit": " "},
                {"vt": "jd", "type": "Misc. generic 4", "unit": " "}]


def Flush(ser):
    """
    Use a while loop to ensure flush all data.
    """    
    ser.write(bytes("\n",  'ascii'))  # write a linefeed to flush
    # blocks the reading until a line break is read.
    res = ser.read_until(bytes("\n", 'ascii')).decode()
    while ser.read_all().decode():                      # read all remaining data.
        time.sleep(0.01)                                # wait 10ms.
    attempt=10
    while res!='\n' and attempt>0:                      # read all remaining data.
        time.sleep(0.01)                                # wait 10ms.
        ser.write(bytes("\n",  'ascii'))  # write a linefeed to flush
        # blocks the reading until a line break is read.
        res = ser.read_until(bytes("\n", 'ascii')).decode()
        attempt -= 1

def IsConnected(ser):   
    ser.write(bytes("t\n",  'ascii'))  # write the command
    response = ser.read_until(bytes("*\n", 'ascii'))  # read until *\n
    response = str(response, 'ascii')  # convert bytes to ascii string
    start = response.find('esp')  # check the presents of "esp" in the repsonse
  
    if start == -1:  # return if string is found
        return False
    return True


def GetResults(ser, timeout=10, emptyout=200):
    "return results as a list of lines, use timeout and max_empty_line to stop."
    results = []
    empty = 0
    t0 = time.monotonic()
    while 1:
        res = ser.readline()
        strline = res.decode('ascii')
        results.append(strline)
        if strline == '\n' or '!' in strline:
            break
        if not strline.strip():
            empty += 1
        if empty > emptyout or (time.monotonic() - t0 > timeout):
            break
    return results


def ValConverter(value, sip):
    return sip_factor.get(sip, np.nan) * value


def ParseVarString(varstr):
    SIP = varstr[7]  # get SI Prefix
    varstr = varstr[:7]  # strip SI prefix from value
    val = int(varstr, 16)  # convert the hexdecimal number to an integer
    val = val - 2**27  # substract the offset binary part to make it a signed value
    return ValConverter(val, SIP)  # return the converted floating point value


def ParseResultsFromLine(res_line):
    lval = list()  # Create a list for values
    lvt = list()  # Create a list for values
    if res_line.startswith('P'):  # data point start with P
        pck = res_line[1:len(res_line)]  # ignore last and first character
        for v in pck.split(';'):  # value fields are seperated by a semicolon
            str_vt = v[0:2]  # get the value-type
            str_var = v[2:2+8]  # strip out value type
            val = ParseVarString(str_var)  # Parse the value
            lval.append(val)  # append value to the list
            lvt.append(str_vt)
    return lval, lvt


def GetValueMatrix(content):
    val_array = [[]]
    j = 0
    for resultLine in content:
        # check for possible end-of-curve characters
        if resultLine.startswith('*') or resultLine.startswith('+') or resultLine.startswith('-'):
            j = len(val_array)  # step to next section of values

        else:
            # parse line into value array and value type array
            vals, _ = ParseResultsFromLine(resultLine)
            # Ignore lines that hold no actual data package
            if len(vals) > 0:
                # If we're here we've found data for this curve, so check that space in allocated for this curve
                # This way we don't allocate space for empty curves (for example when a loop has no data packages)
                if j >= len(val_array):
                    val_array.append([])
                # Add values to value array
                val_array[j].append(vals)
    return val_array


def openSerialPort(comport, logger=None):
    "use proxy for port, so that can auto restart port if problem occur."
    # ser = serial.Serial()
    ser = SerialProxy(
        logger=logger,
        port=comport,  # set the port
        baudrate=230400,  # Baudrate is 230400 for EmstatPico
        bytesize=serial.EIGHTBITS,  # number of bits per bytes
        parity=serial.PARITY_NONE,  # set parity check: no parity
        stopbits=serial.STOPBITS_ONE,  # number of stop bits
        timeout=1,  # timeout block read
        xonxoff=False,  # disable software flow control
        rtscts=False,  # disable hardware (RTS/CTS) flow control
        dsrdtr=False,  # disable hardware (DSR/DTR) flow control
        write_timeout=2,  # timeout for write is 2 seconds
    )
    return ser


def convertUnit(v):
    "convert value to m, or raw value"
    a = np.abs(v)
    if a < 1e-6:
        return f"{v*1e9:.0f}n"
    elif a < 1e-3:
        return f"{v*1e6:.0f}u"
    elif a < 1:
        return f"{v*1e3:.0f}m"
    elif a < 1e3:
        return f"{v:.0f}"
    elif a < 1e6:
        return f"{v/1e3:.0f}k"
    elif a < 1e9:
        return f"{v/1e6:.0f}M"


def convert_voltage(v):
    assert (v >= -1.61 and v <= 1.81), 'Potential out of range'
    return f"{v*1000:.0f}m"


def convert_currange_range(r):
    "'100 nA','1 uA','10 uA','100 uA','1 mA','5 mA'"
    n, u = r.split(' ')
    return n+u[0]


class SerialProxy():
    "a class to proxy serial port so that it can recover automatically. "

    def __init__(self, logger=None, *args, **kwargs):
        self.logger = logger
        self.ser = serial.Serial(*args, **kwargs)
        self.initArgs = args
        self.initKwargs = kwargs
        self.info = self.logger.info if logger else print
        self.error = self.logger.error if logger else print
        self.debug = self.logger.debug if logger else print

    def __setattr__(self, name, val):
        if name in ['logger', 'ser', 'initArgs', 'initKwargs', 'info', 'error', 'debug']:
            super().__setattr__(name, val)
        else:
            setattr(self.ser, name, val)

    def __getattr__(self, name):
        ""
        res = getattr(self.ser, name)
        if callable(res):
            return self.logDeco(res)
        return res

    def logDeco(self, func):
        "a decorator function"
        def wrapped(*args, **kwargs):
            try:
                res = func(*args, **kwargs)
                return res
            except Exception as e:
                self.error(
                    f'Serial Proxy calling serial port function <{func.__name__}> error. Error: {e}')
            # try reconnect:
            try:
                self.ser.close()
            except Exception as e:
                self.error(f"Serial Proxy close port error. Error: {e}")

            # try reconnect for 2 times. if cannot, raise error.
            try:
                for _ in range(2):
                    self.ser = serial.Serial(*self.initArgs, **self.initKwargs)
                    if IsConnected(self.ser):
                        self.info(
                            f'Serial Proxy reopened serial port after {_+1} attempt.')
                        return getattr(self.ser, func.__name__)(*args, **kwargs)
            except Exception as e:
                self.error(f"Serial Proxy reconnect Failed. Error: {e}")
                raise RuntimeError(
                    'Serial Proxy can not recover the serial port after all attempts.')
        return wrapped


class Pico:
    # FIXME
    # need to refine the threshold with fluid fill script.
    FLUID_FILL_THREASHOLD = 0.05

    def __init__(self, port="/dev/serial0", logger=None):
        self.ser = openSerialPort(port, logger)
        self.settings = {}
        
        self.currentMethod = 'default'
        self.currentScript = None

        if logger:
            self.logger = logger
        else:
            class LOG:
                info = debug = error = critical = lambda x: print(x)
            self.logger = LOG

    def initialize(self):
        """
        flush the serial port and test if pico is connected
        if app is started with -dev, then bypass initialization.
        """
        if sys.argv[-1] != '-dev':
            Flush(self.ser)          
            attempt = 1
            # normally, the below part shouldn't be executed.
            while (not self.isConnected):
                self.logger.error(
                    f"Pico Serial port is not open, attempt = {attempt}")
                attempt += 1
                Flush(self.ser)
                if attempt >= 3:
                    self.logger.critical(
                        "Cannot open Pico Serial port after 3 attempt!!!")
                    break

    @property
    def isConnected(self):
        "return True if pico is connected"
        return IsConnected(self.ser)

    def close(self):
        "close serial port"
        self.ser.close()


    # testing funcitons
    def runScript(self, script, parseValue=False):
        "run given script and return results."
        Flush(self.ser)
        self.ser.write(script.encode('ascii'))
        time.sleep(0.05)
        results = GetResults(self.ser, timeout=10, emptyout=20)
        if parseValue:
            return results, GetValueMatrix(results)
        return results


