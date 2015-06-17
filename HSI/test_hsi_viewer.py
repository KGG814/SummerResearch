from subprocess import Popen, PIPE
from math import sin


# Put this in the beginning of your program
# run the python script from the directory where HSI_Viewer.exe is located
pipe = Popen("HSI_Viewer", shell=True, bufsize=100, stdin=PIPE).stdin

t = 0
while 1:
    
    t = t + 0.001
    r = sin(t)
    p = sin(0.5*t + 0.1)
    y = sin(0.4*t + 0.4)
    # put this part in the while loop of your program and pass in roll (r), pitch (p), and yaw (y)
    pipe.write("%lf %lf %lf"%(r,p,y))
