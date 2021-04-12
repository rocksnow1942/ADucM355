"""
A simple class for setting SWV scan parameters.
The scale for voltage/current can be change through vScale/iScale respectively.
    *   Ex: vScale=.001 => mV, 1.0=>V
    *   All following parameters will be interpretted to this value
    *   Note to self: C may get the string in scientific notation so use atof("1e-06")
Time units will be interpretted in seconds
"""

import json

class scanParameters():
    def __init__(   
        self,
        vScale              =   .001,   #   mV
        vStart              =   0.0,    #   0mV
        vEnd                =   -500.0, #   -600mV
        vIncrement          =   -5.0,   #   5mV
        vAmplitude          =   100.0,  #   100mV
        freqHz              =   100.0,  #   100Hz
        iScale              =   1e-06,  #   uA
        maxCurrent          =   50.0,   #   50uA
        vPretreatment       =   0.0,    #   0uA
        secsPretreatment    =   0.2,    #   200mS
        iForwardRecv        =   0,  #   Flag for having forward current reported
        iReverseRecv        =   0):  #   Flag for having subtracted current reported
        

        self.vScale             =   vScale
        self.vStart             =   vStart
        self.vEnd               =   vEnd
        self.vIncrement         =   vIncrement
        self.vAmplitude         =   vAmplitude
        self.freqHz             =   freqHz
        self.iScale             =   iScale
        self.maxCurrent         =   maxCurrent
        self.vPretreatment      =   vPretreatment       # pretreatment potential in scale of vScale
        self.secsPretreatment   =   secsPretreatment    # pretreatment time in scale of seconds
        self.iForwardRecv       =   iForwardRecv
        self.iReverseRecv       =   iReverseRecv

    def set_vScale(self, vScale):
        self.vScale = vScale
    def set_vStart(self, vStart):
        self.vStart = vStart
    def set_vEnd(self, vEnd):
        self.vEnd = vEnd
    def set_vIncrement(self, vIncrement):
        self.vIncrement = vIncrement
    def set_vAmplitude(self, vAmplitude):
        self.vAmplitude = vAmplitude
    def set_freqHz(self, freqHz):
        self.freqHz = freqHz
    def set_iScale(self, iScale):
        self.iScale = iScale
    def set_maxCurrent(self, maxCurrent):
        self.maxCurrent = maxCurrent
    def set_vPretreatment(self, vPretreatment):
        self.vPretreatment = vPretreatment
    def set_secsPretreatment(self, secsPretreatment):
        self.secsPretreatment = secsPretreatment
    def set_currentFlags(self, iForwardRecv,iReverseRecv,iSubtractRecv): # Boolean values or 1/0
        self.iForwardRecv = int(iForwardRecv)
        self.iReverseRecv = int(iReverseRecv)
    def set_scanParamaters(self,vScale,vStart,vEnd,vIncrement,vAmplitude, freqHz,iScale,maxCurrent,vPretreatment,secsPretreatment,iForwardRecv,iReverseRecv):
        self.vScale = vScale
        self.vStart = vStart
        self.vEnd = vEnd
        self.vIncrement = vIncrement
        self.vAmplitude = vAmplitude
        self.freqHz = freqHz
        self.iScale = iScale
        self.maxCurrent = maxCurrent
        self.vPretreatment = vPretreatment
        self.secsPretreatment = secsPretreatment
        self.iForwardRecv = int(iForwardRecv)
        self.iReverseRecv = int(iReverseRecv)
    def get_json(self):
        return json.dumps(self.__dict__)
        
        