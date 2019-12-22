from smbus2 import SMBus
import numpy as np
from math import atan2, sqrt, cos, sin, radians


class alt_imu:
	def __init__(self):
		self.bus = SMBus(1)
		#gyroscoe and accelerometer
		self.gyro_name = 0x0f
		self.gyro_addr = 0x6b
		self.conf_regs = {0x10: 0x80, 0x11:0x80, 0x12:0x04}
		self.outx_l_g = 0x23
		self.outx_l_acc = 0x28
		if self.read_reg(self.gyro_addr, self.gyro_name)==0x69:
			print("gyroscope and accelerometer init done!")
		self.enable()
	def write_reg(self, addr, reg, value):
		self.bus.write_byte_data(addr, reg, value)
	def read_reg(self, addr, reg):
		return self.bus_read_byte_data(addr, reg)
	def enable(self):
		for i in self.conf_reg:
			self.write_reg(self.gyro_addr, i, self.conf_regs[i])
	def read_gyro(self):
		out_gyro = np.array([self.read_reg(self.gyro_addr, i) for i in range(self.outx_l_g, (self.outx_l_g+6))], dtype=np.int8)
		axis_gyro = np.array([self.int16((out_gyro[i+1]<<8|out_gyro[i])) for i in range(0, 6, 2)], dtype=np.int16)
		return axis_gyro
	def read_acc(self):
		out_acc = np.array([self.read_reg(self.gyro_addr, i) for i in range(self.outx_l_acc, (self.outx_l_acc+6))], dtype=np.int8)
		axis_acc = np.array([self.int16((out_acc[i+1]<<8|out_acc[i])) for i in range(0, 6, 2)], dtype=np.int16)
		return axis_acc
	def get_angle(self):
		try:
			axis_acc = self.read_acc()
			z = 1
			if axis_acc[2]<0:
				z*=-1
			pitch = 180*atan2(axis_acc[0], z*sqrt(pow(axis[1], 2)+pow(axis[2], 2)))/3.14
			roll = 180*atan2(axis_acc[1], z*sqrt(pow(axis[0], 2)+pow(axis[2], 2)))/3.14
			return pitch, roll
		except Exception as e:
			self.enable()
			return 999, 999
			