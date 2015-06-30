import os
import serial
import numpy as np 
from time import sleep

class Hokoyu:

	def __init__(self):
		#	NOTE: Wait until green LED on sensor is rapidly flashing, to indicate completion of boot-up process
		# 	Otherwise, commands issued will fail to execute correctly and result in weird error codes!!
		
		try:
			self.Port = serial.Serial(port="/dev/ttyACM0", baudrate=115200);
			#self.Port = serial.Serial(port="COM24", baudrate=115200);
		except:
			print("Could not open serial port, please check your Connection!");
			return
		
		send_status = self.Port.write("SCIP2.0\n");
		receive_modeswitch_status = self.Port.read(13);
		send_status=self.Port.write("BM\n");
		receive_modeswitch_status = self.Port.read(8);
		self.angles = np.zeros(682);
		self.distances = np.zeros(682);
		self.delayTime = 0.1 		# Delay time for continuos updation in seconds

	def getData(self):

		data_acq_init_bytes = "MD0044072501001\n"; 				# Last character--> 1 = only one scan requested, 0 = indefinite until "QT" command issued
		send_status = self.Port.write(data_acq_init_bytes); 	#Send Data acquisition command bytes

		data_acq_received_data = self.Port.read(47);

		lidar_scan_data_buffer = "";

		step_left = 725; # End step
		step_right = 44; # Start step

		i = 0;

		# Does not include carriage returns/line feeds or checksum bytes, these are read in separately:
		bytes_remaining = (step_left - step_right + 1)*3; # 3 bytes per measurement step

	#	for i in range(31): #32
		while bytes_remaining > 0:
			
			# Receive each line of data; read either a full 64 or N bytes (excluding checksum/LF) depending on number of bytes remaining to be read:
			if bytes_remaining >= 64:
				lidar_single_line_data_buffer = self.Port.read(64);
				bytes_remaining -= 64;
			else:
				lidar_single_line_data_buffer = self.Port.read(bytes_remaining);
				bytes_remaining = 0;

			lidar_scan_data_buffer += lidar_single_line_data_buffer;
			i += 1;

			# Separately store the checksum byte for later verification:
			checksum = self.Port.read(1);
			# Take off the newline:
			newling_char = self.Port.read(1);

			data_sum = 0;
			for j in range(len(lidar_single_line_data_buffer)):
				data_sum += ord(lidar_single_line_data_buffer[j]);
			data_sum = data_sum & 0x3f;
			data_sum += 0x30;
			if(data_sum != ord(checksum)):
				print("Checksum error on iteration "+repr(i)+" calculated checksum: "+repr(data_sum)+" actual checksum in stream: "+repr(ord(checksum)));
			

		newling_char = self.Port.read(1);

		# Convert encoded data in buffer into range data and place in global data buffer:
		meas_step = 44;
		i=0;

		while i < len(lidar_scan_data_buffer)-3:
			dist_mm = ((ord(lidar_scan_data_buffer[i]) - 0x30)<<12) | ((ord(lidar_scan_data_buffer[i+1]) - 0x30)<<6) | (ord(lidar_scan_data_buffer[i+2]) - 0x30);
			meas_step += 1; 
			i += 3;
			self.angles[meas_step-44] = (float(meas_step-44)*240.0*(np.pi/180.0)*1.0/float(725-44+1))-(np.pi/6.0);
			self.distances[meas_step-44] = float(dist_mm)/1000.0;
		
		self.Port.flush()

		return self.angles,self.distances

	def release(self):
		self.Port.close()
		

