from math import fmod, pi

def sub_pi_angle(angle):
    angle = fmod(angle, 2.0*pi)
    if angle > pi:
        angle -= 2.0*pi
    if angle < -pi:
        angle += 2.0*pi
    return angle

def clip(val, min_val, max_val):
    if val < min_val:
        return val
    elif val > max_val:
        return max_val
    return val

def sign(x):
    if x < 0:
        return -1
    elif x > 0:
        return 1
    else:
        return 0
