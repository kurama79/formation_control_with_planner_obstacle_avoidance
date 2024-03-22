import math
# from formation_package import math_ as rvo_math
import math_ as rvo_math

class Vector2:

	# Defines a 2-D vector.
	def __init__(self, x=0.0, y=0.0):

		'''
		Constructs and initializes a 2-D vector from the specified xy-coordinates.

		Args:
			x (float): The x-coordinate of the 2-D vector.
			y (float): The y-coordinate of the 2-D vector.
		'''

		self.x_ = x 
		self.y_ = y 

	def __str__(self):
		return "Vector2(x={}, y={})".format(self.x_, self.y_)

	@property
	def x(self):
		return self.x_

	@property
	def y(self): 
		return self.y_ 

	def __matmul__(self, other):

		assert isinstance(other, Vector2), '__matmul__ argument should be a Vector2'
		return self.x_*other.x_ + self.y_*other.y_

	def __mul__(self, other):

		assert not isinstance(other, Vector2), '__mul__ argument should be a float'
		return Vector2(self.x_*other, self.y_*other)

	def __rmul__(self, other):

		assert not isinstance(other, Vector2), '__rmul__ argumetn should be a float'
		return Vector2(other*self.x_, other*self.y_)

	def __truediv__(self, scalar):
		return Vector2(self.x_/scalar, self.y_/scalar)

	def __add__(self, other):
		return Vector2(self.x_ + other.x_, self.y_ + other.y_)

	def __radd__(self, other):
		return Vector2(other.x_ + self.x_, other.y_ + self.y_)

	def __sub__(self, other):
		return Vector2(self.x_ - other.x_, self.y_ - other.y_)

	def __rsub__(self, other):
		return Vector2(other.x_ - self.x_, other.y_ - self.y_)

	def __neg__(self):
		return Vector2(-self.x_, -self.y_)

	def __abs__(self):

		'''
		Computes the length of a specified 2-dimensional vector.

		Args:
			vector (Vector2): The 2-dimensional vector whose length is to be computed.

		Returns:
			float: the length of the 2-dimensional vector.
		'''

		return math.sqrt(rvo_math.abs_sq(self))

class Vector3:

	# Defines a 3-D vector.
	def __init__(self, x=0.0, y=0.0, z=0.0):

		'''
		Constructs and initializes a 3-D vector from the specified xy-coordinates.

		Args:
			x (float): The x-coordinate of the 3-D vector.
			y (float): The y-coordinate of the 3-D vector.
			z (float): The z-coordinate of the 3-D vector.
		'''

		self.x_ = x 
		self.y_ = y 
		self.z_ = z 

	def __str__(self):
		return "Vector3(x={}, y={}, z={})".format(self.x_, self.y_, self.z_)

	@property
	def x(self):
		return self.x_

	@property
	def y(self):
		return self.y_

	@property
	def z(self):
		return self.z_

	def __matmul__(self, other):

		assert isinstance(other, Vector3), '__matmul__ argument should be a Vector3'
		return self.x_*other.x_ + self.y_*other.y_ + self.z_*other.z_

	def __mul__(self, other):

		assert not isinstance(other, Vector3), '__mul__ argument should be a float'
		return Vector3(self.x_*other, self.y_*other, self.z_*other)

	def __rmul__(self, other):

		assert not isinstance(other, Vector3), '__rmul__ argument should be a float'
		return Vector3(other*self.x_, other*self.y_, other*self.z_)

	def __truediv__(self, scalar):
		return Vector3(self.x_/scalar, self.y_/scalar, self.z_/scalar)

	def __add__(self, other):
		return Vector3(self.x_ + other.x_, self.y_ + other.y_, self.z_ + other.z_)

	def __radd__(self, other):
		return Vector3(other.x_ + self.x_, other.y_ + self.y_, other.z_ + self.z_)

	def __sub__(self, other):
		return Vector3(self.x_ - other.x_, self.y_ - other.y_, self.z_ - other.z_)

	def __rsub__(self, other):
		return Vector3(other.x_ - self.x_, other.y_ - self.y_, other.z_ - self.z_)

	def __neg__(self):
		return Vector3(-self.x_, -self.y_, -self.z_)

	def __abs__(self):

		'''
		Computes the length of a specified 3-D vector.

		Args:
			vector (Vector3): The 3-D vector whose length is to be computed.

		Returns:
			float: the length of the 3-D vector.
		'''

		return math.sqrt(rvo_math.abs_sq(self))