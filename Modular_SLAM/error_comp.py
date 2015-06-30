import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np 
from math import hypot

xCorr=[]
yCorr=[]
tCorr=[]
with open("./Data/run5/corr2.txt",'r') as fCorr:
	for line in fCorr:
		data = line.split()
		if data[0]=='O':
			tCorr.append(float(data[1]))
			xCorr.append(float(data[2]))
			yCorr.append(float(data[3]))

tCorr = [i - tCorr[0] for i in tCorr]
xCi = interp1d(tCorr,xCorr)
yCi = interp1d(tCorr,yCorr)

xOvr=[]
yOvr=[]
tOvr=[]
with open("./Data/run5/OverHeadPath.txt",'r') as fOvr:
	for line in fOvr:
		data = line.split()
		if data[0]=='O':
			xOvr.append(float(data[1]))
			yOvr.append(float(data[2]))
			tOvr.append(float(data[3]))

tOvr = [i - tOvr[0] for i in tOvr]


xOi = interp1d(tOvr,xOvr)
yOi = interp1d(tOvr,yOvr)

timeBase = range(0,120000)
errx = abs(np.array(xOi(timeBase))-np.array(xCi(timeBase)))
erry = abs(np.array(yOi(timeBase))-np.array(yCi(timeBase)))

err = (errx**2+erry**2)**0.5
#err=[hypot(abs(xOi(t)-xCi(t)),abs(yOi(t)-yCi(t))) for t in timeBase]
#err=[abs(yOi(t)-yCi(t)) for t in timeBase]
# print tOvr[-1]
# print tCorr[-1]

plt.plot(timeBase,errx)
plt.plot(timeBase,erry)
#plt.plot(tOvr,xOvr,'x')
plt.show()

