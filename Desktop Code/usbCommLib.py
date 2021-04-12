# This library facilitates the JSON communication with the Covid Project: Raspberry Pi
# This can be reduced with a dispatch but it's not a biggie for right now
import json



def createJson(Mode,Type,Data):
    """
    Returns a serialized JSON

    :parameter Mode: string of "get" or "set"
    :parameter Type: string
    :parameter Data: array of data, usually floats, but sometimes strings
    :returns: boolean of if the write was successful
    """
    tempDict = {
    }
    tempDict["mode"] = Mode
    tempDict["type"] = Type
    tempDict["data"] = Data
    return json.dumps(tempDict) + "\n"

# Temperature Communication

def getGoalTemp(ser):
    """
    Sends a JSON to fetch the goal temperature

    :parameter ser: writes a serialized json message to the serial port
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("get","goalTemp", [])
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False


def setGoalTemp(ser, celsiusTemp):
    """
    Sends a JSON to set the goal temperature

    :parameter ser: writes a serialized json message to the serial port
    :parameter celsiusTemp: Writes string/number to set the goal temperature ... ex) "65" or 65.0
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("set","goalTemp", [float(celsiusTemp)])
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False

def getTemperatureControlling(ser):
    """
    Sends a JSON to fetch if it is currently temperature controlling

    :parameter ser: writes a serialized json message to the serial port
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("get","isTemperatureControlling", [])
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False

def setTemperatureControlling(ser, status):
    """
    Sends a JSON to set if it is currently temperature controlling

    :parameter ser: writes a serialized json message to the serial port
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("set","isTemperatureControlling", [float(status)])
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False

def getSecondsTempControlling(ser):
    """
    Sends a JSON to fetch how many seconds it has been temperature controlling

    :parameter ser: writes a serialized json message to the serial port
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("get","secondsTempControlling", [])
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False

def setSecondsTempControlling(ser, seconds):
    """
    Sends a JSON to set how many seconds it has been temperature controlling ... usually to reset

    :parameter ser: writes a serialized json message to the serial port
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("get","secondsTempControlling", [seconds])
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False

def getValidTemperatureReading(ser):
    """
    Sends a JSON to fetch to see if the temperature reading is valid or not

    :parameter ser: writes a serialized json message to the serial port
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("get","validTemperatureReading", [])
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False


# PID Communication

def getPIDTuning(ser):
    """
    Sends a JSON to fetch the PID tuning parameters

    :parameter ser: writes a serialized json message to the serial port
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("get","pidTuning", [])
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False

def setPIDTuning(ser, tuning):
    """
    Sends a JSON to set the PID tuning parameters

    :parameter ser: writes a serialized json message to the serial port
    :parameter tuning: array of floats in the format [kp,ki,kd]
    
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("get","pidTuning", tuning)
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False

# Emstat Pico Communication

def getIsPicoConnected(ser):
    """
    Sends a JSON to check if the Pico is connected and operating normally

    :parameter ser: writes a serialized json message to the serial port
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("get","isPicoConnected", [])
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False

def setMethodScriptFileContents(ser, methodScriptData):
    """
    Sends a JSON that overwrites the default method script read for the pico

    :parameter ser: writes a serialized json message to the serial port
    :parameter methodScriptData: string of the method script
    
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("set","methodScriptFileContents", ["MSfiles/defaultScript.mscr", methodScriptData]) # RPi takes in the file location and then the method script
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False


def getMethodScriptFileContents(ser):
    """
    Sends a JSON to read the contents of the default pico script

    :parameter ser: writes a serialized json message to the serial port
    
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("get","methodScriptFileContents", ["MSfiles/defaultScript.mscr"]) # Reading the default script
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False

def setStartScan(ser):
    """
    Sends a JSON that starts the scan of the Emstat Pico

    :parameter ser: writes a serialized json message to the serial port
    
    :returns: boolean of if the write was successful
    """
    try:
        outputString =  createJson("set","startScan", []) # RPi takes in the file location and then the method script
        ser.write(bytes(outputString,  'ascii'))  # Write the string to the serial
        return True
    except Exception as e:
        print(str(e))
        return False