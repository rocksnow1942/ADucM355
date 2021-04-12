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