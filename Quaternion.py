import numpy as np
from Library.Euler import *

class Quaternion:
	def __init__(self, val=None):
		self.EULER = Euler()
		if str(type(val)) != "<class 'NoneType'>":
			if type(val) == Euler:
				self.setEulerValue(val)
			else:
				self.setValue(val)
		else:
			self.q0 = None
			self.q1 = None
			self.q2 = None
			self.q3 = None

	class EULER:
		pass

	def __iter__(self):
		return self.QuatIterator(self)

	def __str__(self):
		return str(self.getValue())

	def __getitem__(self, i):
		return self.getValue()[i]

	def __len__(self):
		return len(self.getValue())
	
	def setValue(self, v):
		self.q0, self.q1, self.q2, self.q3 = v
		self.getEulerValue()

	def getValue(self):
		return [self.q0, self.q1, self.q2, self.q3]

	def unit(self):
		norm_ = self.norm() + 1e-7
		vu = [u/norm_ for u in self.getValue()]
		return Quaternion(vu)

	def __add__(self, v):
		obj_p = Quaternion()
		if type(obj_p) != Quaternion:
			val = [v_+v for v_ in self]
			obj_p.setValue(val)
		else:
			obj_p.setValue((self.q0 + v.q0, self.q1 + v.q1, self.q2 + v.q2, self.q3 + v.q3))
		return obj_p

	def __radd__(self, v):
		return self + v

	def __sub__(self, v):
		obj_m = Quaternion()
		if type(v) != Quaternion:
			val = [v_-v for v_ in self]
			obj_m.setValue(val)
		else:
			obj_m.setValue((self.q0 - v.q0, self.q1 - v.q1, self.q2 - v.q2, self.q3 - v.q3))
		return obj_m

	def __rsub__(self, v):
		if type(v) != Quaternion:
			return - self + v

	def __mul__(self, v2):
		obj_x = Quaternion()
		if type(v2) != Quaternion:
			val = [v_*v2 for v_ in self]
			obj_x.setValue(val)
		else:
			obj_x.q0 = self.q0*v2.q0 - self.q1*v2.q1 - self.q2*v2.q2 - self.q3*v2.q3
			obj_x.q1 = self.q0*v2.q1 + self.q1*v2.q0 + self.q2*v2.q3 - self.q3*v2.q2
			obj_x.q2 = self.q0*v2.q2 - self.q1*v2.q3 + self.q2*v2.q0 + self.q3*v2.q1
			obj_x.q3 = self.q0*v2.q3 + self.q1*v2.q2 - self.q2*v2.q1 + self.q3*v2.q0
		return obj_x

	def __rmul__(self, v):
		if type(v) != Quaternion:
			return self * v
	
	def __neg__(self):
		nQ = Quaternion()
		val_q = [-q for q in self]
		nQ.setValue(val_q)
		return nQ

	def __isub__(self, v):
		return self - v

	def __truediv__(self, v):
		obj_d = Quaternion()
		if type(v) != Quaternion:
			val = [q/v for q in self]
			obj_d.setValue(val)
			return obj_d

	def __idiv__(self, v):
		return self / v

	def norm(self):
		length = [q**2 for q in self]
		return sum(length)**.5

	def inv(self):
		return ~self

	def __invert__(self):
		return self.cjg() / self.norm()**2.

	def cjg(self):
		q0 = self.q0
		self = - self
		self.q0 = q0
		return self
		
	def rotate(self, q_rotate, vector):
		qr_v = q_rotate * vector
		q_rotate_c = q_rotate.cjg()
		qr_v_qrc = qr_v * q_rotate_c
		return qr_v_qrc
	
	def rotation_q(self, axis3d, angle):
		q_r_ = np.cos(angle/2.), axis3d[0]*np.sin(angle/2.), axis3d[1]*np.sin(angle/2.), axis3d[2]*np.sin(angle/2.)
		self.setValue(q_r_)
		return self

	# for auto get quat value with euler initialization
	def getQuatValue(self):
		return self.euler2quat(self.EULER)

	# for auto get euler with quaternion initialization
	def getEulerValue(self):
		self.EULER = self.quat2euler(self)
		return self.EULER

	def setEulerValue(self, euler_):
		self.EULER.setValue(euler_)
		self.getQuatValue()
	
	def euler2quat(self, euler_):
		roll, pitch, yaw = euler_
		
		rp2 = roll/2.
		pp2 = pitch/2.
		yp2 = yaw/2.
		
		q0 = np.cos(rp2)*np.cos(pp2)*np.cos(yp2) + np.sin(rp2)*np.sin(pp2)*np.sin(yp2)
		q1 = np.sin(rp2)*np.cos(pp2)*np.cos(yp2) - np.cos(rp2)*np.sin(pp2)*np.sin(yp2)
		q2 = np.cos(rp2)*np.sin(pp2)*np.cos(yp2) + np.sin(rp2)*np.cos(pp2)*np.sin(yp2)
		q3 = np.cos(rp2)*np.cos(pp2)*np.sin(yp2) - np.sin(rp2)*np.sin(pp2)*np.cos(yp2)
		
		self.setValue((q0, q1, q2, q3))
		return self

	def quat2euler(self, quat_):
		if type(quat_) != Quaternion:
			quat_ = Quaternion(quat_)
		norm_ = quat_.norm() + 1e-7

		q0, q1, q2, q3 = [vq/norm_ for vq in quat_]

		roll_ = np.arctan((q0*q1 + q2*q3)/ (0.5 - (q1*q1 + q2*q2)))
		pitch_ = np.arcsin(2*(q0*q2 - q3*q1))
		yaw_ = np.arctan((q0*q3 + q1*q2)/ (0.5 - (q2*q2 + q3*q3)))

		self.EULER.setValue((roll_, pitch_, yaw_))
		return self.EULER
	
	def quat_error(self, q_des, q_current):
		cjg_current = q_current.cjg()
		q_err = q_des * cjg_current
		return q_err

	class QuatIterator:
		def __init__(self, Q):
			self._Q = Q
			self.ind_q = 0

		def __next__(self):
			if self.ind_q < len(self._Q):
				ret_val = self._Q[self.ind_q]
				self.ind_q += 1
				return ret_val
			raise StopIteration


# Qu = Quaternion(Euler((30., 30., 30.)).from_deg())
# red = np.array([0])
# hasil = np.concatenate((red, [1,2,3]), axis=0)
# Qu.setValue(hasil)