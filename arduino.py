import serial
import time
from threading import Thread, Event
from queue import Queue
from queue import Empty as QueueEmpty
import struct

PKT_SIZE = 5
PKT_FLAG = 128

class SerialThread(Thread):

	def __init__(self, rxqueue, txqueue):
		super(SerialThread, self).__init__()
		self.rxqueue = rxqueue
		self.txqueue = txqueue
		self._stop_flag = Event()

	def open(self, port): #try to open serial port and return true if connected
		try: self.ser = serial.Serial(port, baudrate=115200, timeout=1)
		except: return False
		time.sleep(1)
		if self.ser.isOpen(): return True
		return False

	def stop(self):
		self._stop_flag.set()

	def run(self):
		while not self._stop_flag.isSet():
			#write data
			if len(self.txqueue) >= 2:
				self._send_data(self.txqueue[0])
				self._send_data(self.txqueue[1])
				self._send_data(PKT_FLAG*2-1)
				del self.txqueue[0]
				del self.txqueue[0]

			#read data
			if self.ser.in_waiting > 300 * PKT_SIZE: self.ser.flushInput()
			data = self.ser.read(PKT_SIZE)

			if len(data) < PKT_SIZE: continue
			if data[-1] != PKT_FLAG: 
				while ord(self.ser.read(1)) != PKT_FLAG: pass
				continue
			dist = data[0] << 8 | data[1]
			angle = data[2] << 8 | data[3]
			if 50 < dist < 6000 and 0 <= angle <= 360:
				self.rxqueue.put((dist, angle))
			else: pass
				#print('Data Error when parsing LIDAR inputs')

		#close port when done
		self.ser.close()

	def _send_data(self, data):
		self.ser.write(struct.pack('>B', int(data)))