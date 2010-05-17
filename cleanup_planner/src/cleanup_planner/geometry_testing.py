from math import hypot
from numpy import array, cross, dot

class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y
    def __str__(self):
        return '('+str(self.x)+', '+str(self.y)+')'

def point_in_polygon(pt, poly):
    """
    Determine if a point lies within a polygon
    """
    n = len(poly)
    inside = False

    p1 = poly[0]
    for i in xrange(n+1):
        p2 = poly[i % n]
        if (pt.y > min(p1.y,p2.y) and pt.y <= max(p1.y,p2.y) and
            pt.x <= max(p1.x,p2.x)):
            if p1.y != p2.y:
                x_cross = (pt.y - p1.y)*(p2.x - p1.x)/(p2.y-p1.y)+p1.x
            if p1.x == p2.x or pt.x <= x_cross:
                inside = not inside

        p1 = p2

    return inside

def point_in_zone(pt, zone):
    """
    Determine if the point lies within the zone
    """
    for poly in zone:
        if point_in_polygon(pt, poly.boundary):
            return True

    return False

def closest_point_on_polygon(point, poly):
    """
    Find the closest point on a polygon to the point
    """
    n = len(poly)
    min_dist = 2000000
    min_point = Point(-1,-1)

    p1 = poly[0]
    for i in xrange(n):
        p2 = poly[(i+1) % n]

        (closest_point, dist) = closest_point_on_line(point, p1, p2)

        if dist < min_dist:
            min_dist = dist
            min_point = closest_point
        p1 = p2

    return (min_point, min_dist)

def closest_point_on_line(q, a1, a2):
    """
    Find the closest point to q on the line defined by x1 and x2.
    Return the distance to this point and the point
    """
    if a1.x == a2.x:
        p = Point(a1.x, q.y)

        if q.y > max(a1.x, a2.x):
            p.y = max(a1.x, a2.x)
        elif p.y < min(a1.x, a2.x):
            p.y = min(a1.x, a2.x)
        dist = hypot(p.x - q.x, p.y - q.y)
        return (p, dist)

    elif a1.x < a2.x:
        x1 = a1
        x2 = a2
    else:
        x1 = a2
        x2 = a1

    X1 = array([x1.x, x1.y])
    X2 = array([x2.x, x2.y])
    Q  = array([ q.x,  q.y])

    v = X2 - X1
    w = Q - X1

    c1 = dot(w,v)
    c2 = dot(v,v)
    b = c1/c2

    P = X1 + b*v
    p = Point(P[0],P[1])
    dist = hypot(p.x - q.x, p.y - q.y)

    if c1 <= 0 or p.x < x1.x:
        d1 = hypot(q.x - x1.x, q.y - x1.y)
        return (x1, d1)
    if c2 <= 0 or p.x > x2.x:
        d2 = hypot(q.x - x2.x, q.y - x2.y)
        return (x1, d2)


    return (p, dist)

def closest_point_on_zone(point, zone):

    min_dist = 200000000
    min_pt = Point(-1,-1)

    for poly in zone:
        (pt,dist) = closest_point_on_polygon(point, poly.boundary)
        if dist < min_dist:
            min_dist = dist
            min_pt = pt

    return (min_pt, min_dist)
