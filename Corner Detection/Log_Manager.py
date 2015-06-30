# Class holding log data of robot.
# The logfile understands the following records:
# E motor (ticks from the odometer) data
# D Lidar Distances
# A Lidar Angles
import re

class LOG_FILE(object):
    def __init__(self):
        self.timeStamps = []
        self.motorTicks = []
        self.scanDistances = []
        self.scanAngles = []
        self.existingPositions = []
        self.lastTicks = None

    def read(self, filename):
        """Reads log data from file. Calling this multiple times with different
           files will result in a merge of the data, i.e. if one file contains
           M and S data, and the other contains M and P data, then Logfile
           will contain S from the first file and M and P from the second file."""
        # If information is read in repeatedly, replace the lists instead of appending,
        # but only replace those lists that are present in the data.
        firstScanDistances = True
        firstScanAngles = True
        firstMotorTicks = True
        firstTimeStamps = True
        firstexistingPositions = True

        with open(filename) as f:
            for l in f:
                sp = re.split("\s*\]?\s*\[?\s*",l.rstrip())
                # T is the motor data.
                # File format: T timestamp[in 0.5 ms] 
                # Stored: A list of tuples [ timeStamp1, timeStamp2,...] in timeStamps.
                if sp[0] == 'T':
                    if firstTimeStamps:
                        self.timeStamps = []
                        firstTimeStamps = False
                    self.timeStamps.append(float(sp[1]))

                # E is the motor data.
                # File format: M timestamp[in 0.5 ms]   rightAccumulator[in degrees]    leftAccumulator[in degrees]
                # Stored: A list of tuples [ (time-stamp, inc-right, inc-left), ... ] with tick increments, in motorTicks.
                # Note that the file contains absolute ticks, but motorTicks contains the increments (differences).
                #       Difference between current time stamp and previous time stamp
                elif sp[0] == 'E':
                    timeStamp = min(self.timeStamps, key=lambda x:abs(x-float(sp[1])))
                    ticks = (float(sp[2]), float(sp[3]))
                    if firstMotorTicks:
                        self.motorTicks = []
                        firstMotorTicks = False
                        self.lastTicks = ticks
                    self.motorTicks.append(
                        tuple((timeStamp,[ticks[i]-self.lastTicks[i] for i in range(2)])))
                    self.lastTicks = ticks

                # D is the scan distances.
                # File format:
                #   D timestamp[in 0.5 ms] distances[in mm] ...
                # Stored: A list of tuples [ [(timeStamp, scan1Distance,... ), (timeStamp, scan2Distance,...) ]
                #   containing all scans, in scanDistances.
                elif sp[0] == 'D':
                    timeStamp = min(self.timeStamps, key=lambda x:abs(x-float(sp[1])))
                    if firstScanDistances:
                        self.scanDistances = []
                        firstScanDistances = False
                    self.scanDistances.append(tuple((timeStamp, map(float, sp[2:]))))

                # A is the scan angles.
                # File format:
                #   D timestamp[in 0.5 ms] angles[in mm] ...
                # Stored: A list of tuples [ [(timeStamp, scan1Angle,... ), (timeStamp, scan2Angle,...) ]
                #   containing all scans, in scanAngles.
                elif sp[0] == 'A':
                    timeStamp = min(self.timeStamps, key=lambda x:abs(x-float(sp[1])))
                    if firstScanAngles:
                        self.scanAngles = []
                        firstScanAngles = False
                    self.scanAngles.append(tuple((timeStamp, map(float, sp[2:]))))

                # O is the ground truth as extracted by Over head camera.
                # File format: O timestamp[in 0.5 ms]   x   y
                # Stored: A list of tuples [ (time-stamp, [x,y]), ... ]
                elif sp[0] == 'O':
                    ticks = (float(sp[2]), float(sp[3]))
                    if firstexistingPositions:
                        self.existingPositions = []
                        firstexistingPositions = False
                    self.existingPositions.append(ticks)

        self.motorTicks = dict(self.motorTicks)
        self.scanDistances = dict(self.scanDistances)
        self.scanAngles = dict(self.scanAngles)
        return

def init(filename,path):
    global Data
    global currentTime
    global Img_path
    Img_path = path
    Data=LOG_FILE()
    Data.read(filename)
    currentTime = 0
    return Data

def get_data(datatype):
    if datatype == "Encoder":
        return Data.motorTicks[currentTime]
    elif datatype == "Lidar":
        return Data.scanAngles[currentTime],Data.scanDistances[currentTime]
    elif datatype == "Camera":
        image = _getImage(currentTime)
        return image

def _getImage(currentTime):
    global Data
    global path
    t=Data.timeStamps.index(currentTime)
    imgName = path+str(t)
    img = cv2.imread(imgName)
    return img

class STORE_DATA(object):
    """Class to store map data"""

    def __init__(self,filename):
        with open(filename,'w'):
            pass
        self.fWrite=open(filename,'a+')

    def write_to_file(self,array,head):
        self.fWrite.write(head)
        for item in array:
            self.fWrite.write(str(item)+"  ")
        self.fWrite.write("\n")
        return

    def write_map(self,mapObj):
        landmarks=[]
        
        for Obj in mapObj:
            if Obj.__class__.__name__=="ROBOT":
                data=[]
                data.append(currentTime)
                data=data+Obj.position.tolist()
                self.write_to_file(data,"O  ")
            else:
                landmarks.append(Obj.position)
        self.write_to_file(landmarks,"L  ")

    def release(self):
        self.fWrite.close()
