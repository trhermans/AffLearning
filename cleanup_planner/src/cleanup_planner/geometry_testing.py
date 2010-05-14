def point_in_polygon(pt, poly):
    n = len(poly)
    inside = False

    p1 = poly[0]
    for i in range(n+1):
        p2 = poly[i % n]
        if pt.y > min(p1.y,p2.y) and pt.y <= max(p1.y,p2.y):
            if pt.x <= max(p1.x,p2.x):
                if p1.y != p2.y:
                    x_cross = (pt.y - p1.y)*(p2.x - p1.x)/(p2.y-p1.y)+p1.x
                if p1.x == p2.x or pt.x <= x_cross:
                    inside = not inside

        p1 = p2

    return inside

def point_polygon_intersection(point, poly):
    return False
