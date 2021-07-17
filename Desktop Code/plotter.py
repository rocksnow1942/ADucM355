import pandas as pd
import csv
from pstraceplot import *

vData = []
iData = []
with open("csv/data.csv", "r") as csv_file:
    lines = csv_file.readlines()
curLineType = 'mV'
for line in lines:
    #tempList = line.split(',')[:-1]
    tempList = []
    for i in line[:-1].split(","):
        tempList.append(float(i))
    if curLineType == 'mV':
        curLineType = 'uA'
        vData.append(tempList)
        continue
    else:
        curLineType = 'mV'
        iData.append(tempList)
        

#for i in range(len(iData)):
#    iData[i] = round(iData[i],3)
plottogrid(vData,iData)





RTIAvals = [0,200,1000,2000,3000,4000,6000,8000,10000,12000,16000,20000,24000,30000,32000,40000,48000,64000,85000,96000,100000,
									120000,128000,160000,196000,256000,512000]
                                    
                                    
def getRTIA(maxCurrent):
    idealResistor = 9e5/maxCurrent
    i = 0
    while (idealResistor >= RTIAvals[i+1]):
        i+=1
        if (i==26):
            return i
    return i                 
    
def getR(m):
    ir = 0.9e6/m
    cd = ir - RTIAvals[0]    
    i=1
    j=0
    nD = ir - RTIAvals[i]
    while(nD<cd):
        if i==26:
            return 26
        cd = nD
        i += 1
        nD = abs(ir - RTIAvals[i])
    if (RTIAvals[i-1]>ir):
        j = i-2
    else:
        j = i-1    
    return j
    

for value in range(4):
    print(f"channgel = {value}")
    mask = ((value & 1) << 2) | ((value &2) << 3)

    print(f"{(16 | 4) & mask:08b}")
    print(f"{(16 | 4) ^ mask:08b}")

value = 2
mask = ((value & 1) << 2) | ((value & 2) << 4)
print(f"{mask:08b}")



print(f"{(16 | 4) & mask:08b}")

    
f"{(16 | 4) :08b}"



    