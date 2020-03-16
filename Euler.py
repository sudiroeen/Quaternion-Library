import numpy as np 

TO_DEG = 180.0/np.pi
TO_RAD = np.pi/180.0

class Euler:
	def __init__(self, v=None):
		if v != None:
			self.setValue(v)
		else:
			self.roll = None
			self.pitch = None
			self.yaw = None

	def setValue(self, val):
		self.roll, self.pitch, self.yaw = val

	def getValue(self):
		return [self.roll, self.pitch, self.yaw]

	def __getitem__(self, i):
		return self.getValue()[i]

	def __len__(self):
		return len(self.getValue())

	def __mul__(self, v):
		objm = Euler()
		if type(v) != Euler:
			objm.setValue([r*v for r in self])
		return objm

	def __rmul__(self, v):
		return self * v

	def to_deg(self):
		deg_val = [rpy*TO_DEG for rpy in self]
		return Euler(deg_val)

	def from_deg(self):
		rad_val = [rpy*TO_RAD for rpy in self]
		return Euler(rad_val)

	def __iter__(self):
		return self.EulerIterator(self)

	def __str__(self):
		return str(self.getValue())

	class EulerIterator:
		def __init__(self, E):
			self._E = E
			self.ind_e = 0

		def __next__(self):
			if self.ind_e < len(self._E):
				ret_val = self._E[self.ind_e]
				self.ind_e += 1
				return ret_val
			raise StopIteration