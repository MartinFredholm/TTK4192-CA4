from math import cos, sin, atan2, pi, sqrt

def M(theta):
    """
    Return the angle phi = theta mod (2 pi) such that -pi <= theta < pi.
    """
    theta = theta % (2*pi)
    if theta < -pi: return theta + 2*pi
    if theta >= pi: return theta - 2*pi
    return theta

def cart2polar(x, y):
    """
    Return the polar coordinates (r, theta) of the point (x, y).
    """
    r = sqrt(x*x + y*y)
    if r>0:
        theta = atan2(y, x)
    else:
        theta = 0
    return r, theta

def change_of_basis(p1, p2, minRadius :float):
    """
    Given p1 = (x1, y1, theta1) and p2 = (x2, y2, theta2) represented in a
    coordinate system with origin (0, 0) and rotation 0 (in degrees), return
    the position and rotation of p2 in the coordinate system which origin
    (x1, y1) and rotation theta1.
    """
    theta1 = p1[2]
    dx = (p2[0] - p1[0])/minRadius
    dy = (p2[1] - p1[1])/minRadius
    new_x = dx * cos(theta1) + dy * sin(theta1)
    new_y = -dx * sin(theta1) + dy * cos(theta1)
    new_theta = p2[2] - p1[2]
    return new_x, new_y, new_theta

def revert_change_of_basis(p1, local_p, minRadius: float):
    """
    Given p1 = (x1, y1, theta1) and local_p = (x_local, y_local, theta_local) represented in
    the local coordinate system with origin (x1, y1) and rotation theta1, return
    the position and rotation of local_p in the global coordinate system (origin (0, 0) and rotation 0).
    
    Parameters:
    - p1: The original point defining the local coordinate system (x1, y1, theta1).
    - local_p: The point in the local coordinate system (x_local, y_local, theta_local).
    - minRadius: The scaling factor used in the original transformation.
    
    Returns:
    - (x_global, y_global, theta_global): The point in the global coordinate system.
    """
    theta1 = p1[2]
    x_local, y_local, theta_local = local_p
    
    # Rotate the local coordinates back by -theta1 (inverse rotation)
    dx_rotated = x_local * cos(theta1) - y_local * sin(theta1)
    dy_rotated = x_local * sin(theta1) + y_local * cos(theta1)
    
    # Scale by minRadius and add the original translation (p1[0], p1[1])
    x_global = p1[0] + dx_rotated * minRadius
    y_global = p1[1] + dy_rotated * minRadius
    
    # Adjust the angle
    theta_global = theta_local + p1[2]
    
    return x_global, y_global, theta_global



def rad2deg(rad):
    return 180 * rad / pi

def deg2rad(deg):
    return pi * deg / 180

def sign(x):
    return 1 if x >= 0 else -1
