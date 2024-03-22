# Contains functions and constants used in multiple classes

import math

# A sufficiently small positive number
EPSILON = 0.00001

def abs_sq(vector):

	'''
	Computes the square length of a specified 2-dimensional vector.

	Args:
		vector (Vector2): The 2-dimensional vector whose squared length is to be computed.

	Returns:
		float: The squared length of the 2-dimensional vector.
	'''

	return vector @ vector

def normalize(vector):

	'''
	Computes the normalization of the specified 2-dimensional vector.

	Args:
		vector (Vector2): The 2-dimensional vector whose normalization is to be computed.

	Returns:
		Vector2: The normalization of the 2-dimensional vector.
	'''

	return vector / abs(vector)

def det(vector1, vector2):

	'''
	Computes the determinant of 2-dimensional square matrix with rows consisting of the 
	specified 2-dimensional vectors.

	Args:
		vector1 (Vector2): The top row of the 2-dimensional square matrix
		vector2 (Vector2): The bottom row of the 2-dimensional square matrix

	Returns:
		float: The determinant of the 2-dimensional square matrix
	'''

	return vector1.x_*vector2.y_ - vector1.y_*vector2.x_

def dist_sq_point_line_segment(vector1, vector2, vector3):

	'''
	Compute the squared distance from a line segment with the specified endpoints to a specified
	point.

	Args:
		vector1 (Vector2): The first endpoint of the line segment.
		vector2 (Vector2): The second endpoint of the line segment.
		vector3 (Vector2): The point to which the squared distance is to be calculated.

	Return:
		float: The squared distance from the line segment to the point.
	'''

	r = ((vector3 - vector1) @ (vector2 - vector1))/abs_sq(vector2 - vector1)

	if r < 0.0:
		return abs_sq(vector3 - vector1)

	if r > 1.0:
		return abs_sq(vector3 - vector2)

	return abs_sq(vector3 - (vector1 + r*(vector2 - vector1)))

def left_of(a, b, c):

	'''
	Computes the signed distance from a line connecting the specified points to a specified point.

	Args:
		a (Vector2): The first point on the line.
		b (Vector2): The second point on the line.
		c (Vector2): The point to which the signed distance is to be calculated.

	Returns:
		float: Positive when the point c lies to the line ab.
	'''

	return det(a - c, b - a)

def square(scalar):

	'''
	Computes the square of a float.

	Args:
		scalar (float): The float to be squared.

	Returns:
		float: The square of the float.
	'''

	return scalar*scalar