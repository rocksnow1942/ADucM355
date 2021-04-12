"""

Here's my spaghetti code  usb script for interfacing with the ADuCM355.
All UART messages use the char '*' as a terminating byte.
    *   Could be changed to '}' in the future since it's all JSON, but there's bigger things to do as of this writing
Many improvements can be made but I'm prototyping here so whoever is reading this, cut me some slack

"""

import serial
import serial.tools.list_ports
import time
import threading
import sys
import signal
import os
import csv
from itertools import zip_longest
import pandas as pd
from pstraceplot import *
import json
from collections import namedtuple
from json import JSONEncoder
import M355ScanParameters as M355
import pickle


"""
USB class for interfacing with the ADuCM355
"""
class usbM355():
    def __init__(self):
        self.serialPort = None
        self.startUsbProcessor()
        self.processing = False

    def findComPort(self):
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

    def openPort(self):
        """
        Opens up the serial port to the M355

        :returns: serial object
        """
        comPort = self.findComPort()
        if comPort:
            return serial.Serial(   port=comPort,
                                    baudrate=115200,
                                    timeout=5)
        else:
            return None

    def getDataFromPort(self):
        """
        Reads the JSON from the open serial port
        End character is '*'

        :returns: string of message
        """
        recvString = ""
        while True:
            recvChar = self.serialPort.read(1).decode("utf-8")
            if recvChar == "*":
                break
            recvString += recvChar
        return recvString

    def backgroundProcessor(self):
        """
        Connects to the M355 through USB 

        :parameter ser: serial object to read from
        :returns: string of message
        """
        while True: # loop to connect and reconnect
            self.serialPort = None
            try:
                while True: # try to connect every 5 seconds
                    self.serialPort = self.openPort()
                    if(self.serialPort==None):
                        print("Unable to connect to COMPORT, try again")
                        time.sleep(5)
                    else:
                        print("Connected to COMPORT")
                        break
                # Connected to COMPORT
                while True:
                    recvString = self.getDataFromPort()
                    if(recvString != ""): # If data is not empty
                        #print(recvString)
                        #continue
                        data_M355 = json.loads(recvString, object_hook=self.M355_DataDecoder) # turn json into data structure
                        """
                        Here are the current values that can be read from data_M355:
                                * vScale            -   scale of volts (ex: .001 = mV, 1.0=V)
                                * vStart            -   starting voltage (in terms of vScale)
                                * vEnd              -   ending voltage (in terms of vScale)
                                * vIncrement        -   voltage scan increment value (in terms of vScale)
                                * vAmplitude        -   peak-to-peak amptitude for SWV
                                * iScale            -   scale of amperes (ex: .001 = mA)
                                * forwardCurrent    -   array of forward current (in terms of iScale)
                                * reverseCurrent    -   array of reverse current (in terms of iScale)
                                * subtractCurrent   -   array of the difference between forward and reverse current (in terms of iScale)
                        """

                        print("Scanned!")

                        dataDict = {
                            "vData":None,
                            "iData":None,
                            "iDataF":None,
                            "iDataR":None
                        }
                        dataDict["vData"] = list(range(data_M355.vStart,data_M355.vEnd,data_M355.vIncrement))[6:-2] # creates the voltage list
                        # cuts off the first few and last points. They are garbage data for garbage people .. honestly not sure why the current is bad since the o-scope voltage looks fine
                        dataDict["iData"] = data_M355.subtractCurrent[6:-2] # grabs the current from the M355
                        dataDict["iDataF"] = data_M355.forwardCurrent[6:-2] # grabs the forward current from the M355
                        dataDict["iDataR"] = data_M355.reverseCurrent[6:-2] # grabs the reverse current from the M355
                        #plt.plot(dataDict["vData"],dataDict["iData"],'r')
                        #plt.plot(dataDict["vData"],dataDict["iDataF"],'g')
                        #plt.plot(dataDict["vData"],dataDict["iDataR"],'b')
                        #plt.show()
                        #time.sleep(1000)
                        #np.save('data.npy', dataDict) 


                        # TODO: Create better file structure for scan data
                        f=open('scan data/data.p', 'ab')
                        pickle.dump(dataDict, f)
                        f.close()
                        #csvfile = open("csv/data.csv", 'a', newline='') # Write the data to a csv

                        #with csvfile:
                        #    wr = csv.writer(csvfile)
                        #    wr.writerow(vData)
                        #    wr.writerow(iData)
                        #csvfile.close()
                        #print("saved to csv")
                        print("Plotting...", end = '')
                        plotter("scan data/data.p") # Plot that mf thang
                        print("Plotted!")

                        self.processing = False # Set flag to False so main loop can continue

                        #print(vData)
                        #print(iData)


            except Exception as e: # something went wrong with usb connection
                print(str(e))

            
    def startUsbProcessor(self):
        """
        Connects to the com port, reads from the port, and processes each message
        This runs the USB processor in the background on another thread

        :returns: None
        """
        usbThread = threading.Thread(   target=self.backgroundProcessor, 
                                        args=()) 
        usbThread.start() # Starts the thread

    def send(self, data):
        """
        Sends the string data to the connected serial port
        """
        if(data[-1] != '\n'):
            data += '\n'
        self.serialPort.write(bytes(data,  'ascii'))  # Write the string to the serial

    
    def M355_DataDecoder(self,dataDict):
        """
        Turns a json string into a data structure
        """
        return namedtuple('X', dataDict.keys())(*dataDict.values())



def main():
    
    # Create the scan parameters to the ADuCM355
    scanParams = M355.scanParameters(   vScale=1e-3, #mV
                                        vStart=-600,
                                        vEnd=0,
                                        vIncrement=5,
                                        vAmplitude=100,
                                        freqHz=100,
                                        iScale=.000001, #uA
                                        maxCurrent=100, #uA
                                        vPretreatment=0,
                                        secsPretreatment=0,
                                        iForwardRecv=1, # flag for forward current
                                        iReverseRecv=1) # flag for forward current
    #time.sleep(4) # Delay for initial usb connection
    print(str(scanParams.get_json()))
    while(1):
        if(usbObj.serialPort != None): # Check if connected, if not sleep for a second
            while(1):
                usbObj.processing = True
                jsonString = str(scanParams.get_json())
                print("Scanning...", end = "")
                usbObj.send(jsonString + "*") #ADuCM355 sees the '*' as the terminating symbol. Can be changed to '{' in the future
                while(usbObj.processing):
                    time.sleep(1) # poll every 1 second
                time.sleep(30) # Delay between scans ... plotter takes about 1-2 seconds
        #else:
            #print("No serial port object?")
        time.sleep(1)


if __name__ == '__main__':
    try:
        usbObj = usbM355()
        main()
    except KeyboardInterrupt:
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)


usbObj = usbM355()

scanParams = M355.scanParameters(   vScale=1e-3, #mV
                                    vStart=-600,
                                    vEnd=0,
                                    vIncrement=5,
                                    vAmplitude=100,
                                    freqHz=100,
                                    iScale=.000001, #uA
                                    maxCurrent=100, #uA
                                    vPretreatment=0,
                                    secsPretreatment=0,
                                    iForwardRecv=1, # flag for forward current
                                    iReverseRecv=1) # flag for forward current

print(str(scanParams.get_json()))


