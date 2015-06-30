import serial
import struct
import numpy as np
import socket
from threading import Thread

byteMap={
"START_LOGGING"	:"\x81",
"RENEW_LOGFILE"	:"\x82",
"STOP_LOGGING" 	:"\x83",
"START_TX" 		:"\x84",
"START_RX" 		: 0x85,
"END_CONN"		: 0x87
}

commands=[]
class DownLink:

	def __init__(self):
		self.port=serial.Serial(port='/dev/ttySAC2',baudrate=115200)		# Define blocking serial port @ 115200 baudrate
		#self.port=serial.Serial(port='COM21',baudrate=115200)		# Define blocking serial port @ 115200 baudrate
		print("connected to: " + self.port.portstr)
		self.message=None

	def getData(self):
		while True:
			byte=self.port.read()
			if ord(byte)==byteMap["START_RX"]:
				m1=self.port.read(60)
				chkSum=self.port.read()
				if DownLink.isValid(m1,chkSum):
					m2=[m1[i:i+4] for i in range(0, len(m1), 4)]
					self.message= [struct.unpack('<f', val) for val in m2]
					self.newdata=True
				else:
					print "Invalid Check Sum Received"
				break
			else:
				continue
		return np.array(self.message)

	@staticmethod
	def isValid(byteArray,checkSum):
		check=0
		for ch in byteArray[0:8]:
			check=check+ord(ch)
		check = check&0xFF
		if ord(checkSum)==check:
			return True
		else:
			return False

	def sendValues(self,value1=0,value2=0):
		self.port.write(byteMap["START_TX"])
		txMessage=struct.pack('<f',value1)+struct.pack('<f',value2)
		for bytes in txMessage:
			self.port.write(chr(ord(bytes)))
		chk=DownLink.calcCheckSum(txMessage)
		self.port.write(chk)

	@staticmethod
	def calcCheckSum(byteArray):
		check=0 
		for bytes in byteArray:
			check=check+ord(bytes)
		return chr(check&0xFF)

	def sendCommand(self):
		global commands
		if commands:
			for commandString in commands:
				self.port.write(byteMap[commandString])
				print commandString
			commands=[]
		return

	def release(self): 
		self.port.close()


class UPLink:
	def __init__(self,portNum):
		# Create a TCP/IP socket
		self.connObj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		# Bind the socket to the port
		server_address = ('', portNum)
		print 'starting up on %s port %s' % server_address
		self.connObj.bind(server_address)

		self.connStatus=False
		self.connection=None
		self.client_address=None
		self.message=None
		self.newData=False
		self.newCommand=False
		tWaiting=Thread(target=self.KeepListening, args=())
		tWaiting.daemon=True
		tWaiting.start()
		return

	def getData(self):
		if self.message:
			return self.message[0][0],self.message[1][0]
		else:
			return 0,0

	def KeepListening(self):
		while True:
			# Listen for incoming connections
			self.connObj.listen(1)
			print 'waiting for a connection'
			self.connection, self.client_address = self.connObj.accept()
			print 'connection from', self.client_address
			self.connStatus=True
			while True:
				byte=self.recv_blocking(1)
				if ord(byte)==byteMap["START_RX"]:
					m1=self.recv_blocking(8)
					chkSum=self.recv_blocking(1)
					if UPLink.isValid(m1,chkSum):
						m2=[m1[i:i+4] for i in range(0, len(m1), 4)]
						self.message= [struct.unpack('<f', val) for val in m2]
						print "got data"
						self.newData=True
					else:
						print "Invalid Check Sum Received from UpLink"
				if ord(byte)==byteMap["START_LOGGING"]:
					commands.append("START_LOGGING")
					self.newCommand=True
				if ord(byte)==byteMap["RENEW_LOGFILE"]:
					commands.append("RENEW_LOGFILE")
					self.newCommand=True
				if ord(byte)==byteMap["STOP_LOGGING"]:
					commands.append("STOP_LOGGING")
					self.newCommand=True
				if ord(byte)==byteMap["END_CONN"]:
					break
		return

	@staticmethod
	def isValid(byteArray,checkSum):
		check=0
		for ch in byteArray:
			check=check+ord(ch)
		check = check&0xFF
		if ord(checkSum)==check:
			return True
		else:
			return False

	def recv_blocking(self,size):
		msg_buffer = ""
		while len(msg_buffer) < size:
			msg_buffer += self.connection.recv(1)
		return msg_buffer

	def release(self):
		self.connection.close()
		return