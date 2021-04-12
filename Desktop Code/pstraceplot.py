"""
plot data from the pstrace monitor log file to grids.
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import pickle
"""
my fit for echem data.
it's ~ 6.6 fold faster than fitpeak2; 180 fold faster than oldmethod.
TODO:
1. test on more messy data.
2. heuristic for finding tangent.
3. ways to measure fitting error by the angle between tangent line and curve slope at contact point.
update 6/9:
change intercept to account for min(0,) in whole
change peak finding prominence requirements.
"""

def smooth(x,windowlenth=11,window='hanning'):
    "windowlenth need to be an odd number"
    s = np.r_[x[windowlenth-1:0:-1],x,x[-2:-windowlenth-1:-1]]
    w = getattr(np,window)(windowlenth)
    return np.convolve(w/w.sum(),s,mode='valid')[windowlenth//2:-(windowlenth//2)]


def intercept(x, x1, x2, whole=False):
    """
    determine whether the line that cross x1 and x2 and x[x1],x[x2] will intercept x.
    if whole == False, will only consider one side.
    Only consider the direction from x2 -> x1,
    that is:
    if x1 > x2; consider the right side of x2
    if x1 < x2; consider the left side of x2
    """
    # set tolerance to be 1/1e6 of the amplitude
    xtol = - (x.max() - x.min())/1e6
    y1 = x[x1]
    y2 = x[x2]
    k = (y2-y1)/(x2-x1)
    b = -k*x2 + y2
    maxlength = len(x)
    res = x - k*(np.array(range(maxlength)))-b
    if whole:
        return np.any(res[max(0, x1 - maxlength//20 - 5):x2 + maxlength//20 + 5] < xtol)
    if x1 > x2:
        return np.any(res[x2: x1 + maxlength//20 + 5] < xtol)
    else:
        # only consider extra half max width; make sure at least 5 points
        return np.any(res[max(0, x1 - maxlength//20 - 5):x2] < xtol)



def sway(x,center,step,fixpoint):
    if center==0 or center==len(x):
        return center

    if not intercept(x,center,fixpoint):
        return center
    return sway(x,center+step,step,fixpoint)


def find_tangent(x, center):
    newleft = left = center - 1
    newright = right = center + 1
    while intercept(x, left, right, True):
        if intercept(x, left, right):
            newleft = sway(x, left, -1, right)

        if intercept(x, right, left):
            newright = sway(x, right, 1, newleft)

        if newleft == left and newright == right:
            break
        left = newleft
        right = newright
    return left, right


def pickpeaks(peaks, props, totalpoints):
    "the way to pick a peak"
    if len(peaks) == 1:
        return peaks[0]
    # scores = np.zeros(len(peaks))
    # heights = np.sort(props['peak_heights'])
    # prominences = np.sort(props['prominences'])
    # widths = np.sort(props['widths'])
    normheights = props['peak_heights']/(props['peak_heights']).max()
    normprominences = props['prominences']/(props['prominences']).max()
    normwidths = props['widths']/(props['widths']).max()
    # bases = ((props['left_ips'] == props['left_ips'].min()) &
    #      (props['right_ips'] == props['right_ips'].max()))
    leftbases = props['left_ips'] < totalpoints/10

    scores = normheights + normprominences + normwidths - 2*leftbases #- 2*bases
    topick = scores.argmax()
    return peaks[topick]


def myfitpeak(xydataIn):
    x = xydataIn[0,:] #voltage
    y = xydataIn[1,:] #current

    y = smooth(y)
    # limit peak width to 1/50 of the totoal scan length to entire scan.
    # limit minimum peak height to be over 0.2 percentile of all neighbors
    heightlimit = np.quantile(np.absolute(y[0:-1] - y[1:]), 0.8) * 3
    # heightlimit = np.absolute(y[0:-1] - y[1:]).mean() * 3
    # set height limit so that props return limits
    peaks, props = signal.find_peaks(
        y, height=heightlimit, prominence=heightlimit, width=len(y) / 30, rel_height=0.5)

    # return if no peaks found.
    if len(peaks) == 0:
        return x,y,0,0,0,0,0,-1

    peak = pickpeaks(peaks,props,len(y))

    # find tagent to 3X peak width window
    x1,x2 = find_tangent(y,peak)

    y1=y[x1]
    y2=y[x2]
    k=(y2-y1)/(x2-x1)
    b = -k*x2 + y2

    peakcurrent = y[peak] - (k*peak + b)
    peakvoltage = x[peak]

    twopointx = np.array([x[x1],x[x2]])
    twopointy = np.array([y[x1],y[x2]])

    # for compatibility return the same length tuple of results.
    # currently, no error is calculated.
    return x,y,twopointx,twopointy,twopointy,peakcurrent,peakvoltage,0






def myfitplotax(v, a, ax, index, color, aForward,aReverse,axislabel=False):

    fit = myfitpeak(np.array([v, a]))
    xnotjunk, ynotjunk, xforfit, gauss, baseline, peakcurrent, peakvoltage, fiterror = fit
    if isinstance(xforfit, int):
        ax.plot(v, a)
        ax.set_title("Failed To Fit", fontsize=10, color='r')
        ax.set_xticks([])
        ax.set_yticks([])
        return peakcurrent
    x1, x2 = xforfit
    y1, y2 = baseline
    k = (y2-y1)/(x2-x1)
    b = -k*x2 + y2
    baselineatpeak = k*peakvoltage + b
    ax.plot(v, a, v ,ynotjunk, xforfit, baseline,
            [peakvoltage, peakvoltage], [baselineatpeak, baselineatpeak+peakcurrent])

    if len(aForward) == len(v):
        ax.plot(v,aForward, 'c')
    if len(aReverse) == len(v):
        ax.plot(v,aReverse, 'm')
    ax.set_title("{},{:.2f}uA,{}mV".format(index, peakcurrent,peakvoltage),
                 fontsize=10, color=color)
    #if not axislabel:
    #    ax.set_xticks([])
    #    ax.set_yticks([])
    ax.axis('on')
    return peakcurrent


def plottogrid(voltages, amps, aForward,aReverse,axislabel=False):
    """
    voltages: [ [...] , [...], [...], ... ]
    amps:     [ [...] , [...], [...], ... ]
    """
    l = len(voltages)
    rows = int(np.ceil(np.sqrt(l)))
    cols = int( np.ceil( l / rows) )
    fig, axes = plt.subplots(rows,cols, figsize=( 1.5*cols, 1.5*rows ))
    if rows==1 and cols==1:
        axes = [axes]
    elif cols == 1:
        axes = axes
    else:
        axes = [i for j in axes for i in j]  
    for ax in axes:
        ax.axis('off')
    peakArray = []
    for k, (v,a,aF,aR, ax) in enumerate(zip(voltages, amps, aForward,aReverse, axes)):

        peakArray.append(float(myfitplotax(v, a, ax, k, 'b',aF, aR, axislabel=axislabel)))
    plt.tight_layout()
    plt.savefig("IV Curves.png", dpi=400)
    plt.close()
    for peak in peakArray:
        print(peak)
    plt.plot(peakArray)
    plt.savefig("Peaks vs Time.png", dpi=400)


def plotter(fileName):
    """
    Plots the pickle data in fileName

    """
    vData = []
    iData = []
    iDataF = []
    iDataR = []
    f=open('scan data/data.p', 'rb')
    dataDicts = []
    while True:
        try:
            dataDicts.append(pickle.load(f))
        except EOFError:
            break

    for dataDict in dataDicts:
        vData.append(dataDict["vData"])
        iData.append(dataDict["iData"])
        iDataF.append(dataDict["iDataF"])
        iDataR.append(dataDict["iDataR"])

    plottogrid(vData,iData,iDataF,iDataR)
