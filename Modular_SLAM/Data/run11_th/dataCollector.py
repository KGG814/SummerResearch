import Communication
import Camera
import Lidar
import cv2
from threading import Thread 
import os

IMUData={ 
	"Gyro":[0,0,0],
	"Mag" :[0,0,0],
	"Acc1":[0,0,0],
	"Acc2":[0,0,0]
}
Encoders=[0,0]
PressedKey=[]
timeStamp=0

def writeFile(f,array,head,count):
	f.write(head+str(count)+" ")
	for item in array:
		f.write(str(item)+"  ")
	f.write("\n")
	return

def clearAllFiles(fileName):
	with open(fileName,'w'):
		pass
	return


def keyPress():
	global PressedKey
	print "waiting for key"
	k=raw_input()
	PressedKey.append(k)
	return

def dataHandler(floatArray):
	global timeStamp
	global Encoders
	global IMUData
	timeStamp=floatArray[0]
	Encoders=floatArray[1:3]
	IMUData["Gyro"]=floatArray[3:6]
	IMUData["Mag"]=floatArray[6:9]
	IMUData["Acc1"]=floatArray[9:12]
	IMUData["Acc2"]=floatArray[12:15]
	return

def doHeavyStuff(CameraObj,LidarObj,fileName,count):
	global timeStamp
	angles=[]
	distances=[]
	angles,distances=LidarObj.getData()
	with open(fileName,"ab") as f:
		writeFile(f,[],"T  ",timeStamp)
		writeFile(f,Encoders,"E  ",timeStamp)
		writeFile(f,IMUData["Gyro"],"G  ",timeStamp) 
		writeFile(f,IMUData["Mag"],"M  ",timeStamp)
		writeFile(f,IMUData["Acc1"],"A1  ",timeStamp)
		writeFile(f,IMUData["Acc2"],"A2  ",timeStamp)
		writeFile(f,angles,"A  ",timeStamp)
		writeFile(f,distances,"D  ",timeStamp)
	cameraFrame=CameraObj.getFrame()
	cv2.imwrite('Image_'+str(count)+'.jpg',cameraFrame)
	#print "stored "+str(timeStamp)
	return

def main():
	global PressedKey
	global timeStamp
	i = 0
	while os.path.exists("SlamData%s.txt" % i):
		i += 1

	fName = "SlamData%s.txt" % i
	clearAllFiles(fName)
	timeOld=0
	velocity=0
	omega=0
	step=0.05
	t1=Thread(target=keyPress, args = ())
	t1.daemon=True
	t1.start()
	camObject=Camera.webcamImageGetter()
	camObject.start()
	AP_Comm=Communication.DownLink()
	PC_Comm=Communication.UPLink(9003)
	lidarObject=Lidar.Hokoyu()
	try:
		count = 0
		t2=Thread(target=doHeavyStuff, args = (camObject,lidarObject,fName,count))
		t2.daemon=True
		t2.start()
		while True:
			message=AP_Comm.getData()
			dataHandler(message)
			# print timeStamp
			timeNew=timeStamp
			if PressedKey:
				k=PressedKey[0]
				if k=='x':
					velocity=0
					omega=0
					break
				if k=='w':
					velocity=velocity+step
				if k=='s':
					velocity=velocity-step
				if k=='a':
					omega=omega+step
				if k=='d':
					omega=omega-step
				if k=='q':
					velocity=0
					omega=0
				if k=='l':
					Communication.commands.append("START_LOGGING")
				if k=='k':
					Communication.commands.append("RENEW_LOGFILE")
				if k=='j':
					Communication.commands.append("STOP_LOGGING")
				print velocity,omega
				PressedKey=[]
				t1.join()
				t1=Thread(target=keyPress, args = ())
				t1.daemon=True
				t1.start()
				AP_Comm.sendValues(velocity,omega)
				AP_Comm.sendCommand()

			if PC_Comm.newData:
				velocity,omega=PC_Comm.getData()
				AP_Comm.sendValues(velocity,omega)
				PC_Comm.newData=False

			if PC_Comm.newCommand:
				AP_Comm.sendCommand()
				PC_Comm.newCommand=False

			if (timeNew-timeOld)>400:
				print (timeNew-timeOld)
				t2.join()
				count = count+1
				t2=Thread(target=doHeavyStuff, args = (camObject,lidarObject,fName,count))
				t2.daemon=True
				t2.start()
				timeOld=timeNew
	finally:
		AP_Comm.release()
		PC_Comm.release()
		camObject.release()
		lidarObject.release()

if __name__ == '__main__':
	main()
