from math import hypot, atan2, degrees
import roslib; roslib.load_manifest('cleanup_planner')
import rospy
from geometry_msgs.msg import Pose2D
from geometry_testing import point_in_polygon, closest_point_on_zone

class GreedyCleanupPlanner:

    def __init__(self):
        self.cleanup_zones = None
        self.standoff_dist = 200.0

    def get_object_ordering(self, objects, start_pose):
        """
        Do a greedy best-first search of the objects
        """
        cleanup_path = []
        cleanup_path_ids = []
        cleanup_ids = {}

        for i, obj in enumerate(objects):
            if self.cleanup_zones is None:
                rospy.logwarn("No cleanup zone!")
                cleanup_ids[obj.object_id] = i
            else:
                add_point = True
                for zone in self.cleanup_zones:
                    if point_in_polygon(obj.pose, zone.boundary):
                        add_point = False
                if add_point:
                    cleanup_ids[obj.object_id] = i

        active_pose = start_pose

        for i in xrange(len(cleanup_ids)):
            min_dist = 10000000
            min_id = -1

            for obj_id in cleanup_ids:
                query_obj = objects[cleanup_ids[obj_id]]
                # TODO: change this measure from dist to time, incorporate
                # use of current heading
                dist = hypot(active_pose.y - query_obj.pose.y,
                             active_pose.x - query_obj.pose.x)

                if dist < min_dist:
                    min_dist = dist
                    min_id = obj_id

            active_pose = objects[cleanup_ids[min_id]].pose
            cleanup_path.append(active_pose)
            cleanup_path_ids.append(min_id)
            cleanup_ids.pop(min_id)

        # Translate object positions into visit poses
        for i, place in enumerate(cleanup_path):
            cleanup_path[i] = self.get_object_visit_pose(place)

        for i, place in enumerate(cleanup_path):
            rospy.loginfo("[%d]: (%g, %g, %g)" %
                          (cleanup_path_ids[i], place.x, place.y,
                           degrees(place.theta)))

        return cleanup_path

    def get_object_visit_pose(self, pt):
        """
        Method returns a pose such that the object pose given will be between
        the returned pose and the closest point on the cleanup polygon
        """
        pose = Pose2D()
        pose.x = pt.x
        pose.y = pt.y

        poly_pt, dp = closest_point_on_zone(pt, self.cleanup_zones)

        if pt.x == poly_pt.x:
            pose.x = pt.x
            if pt.y < poly_pt.y:
                pose.y = pt.y - self.standoff_dist
            else:
                pose.y = pt.y + self.standoff_dist
        else:
            m = (pt.y - poly_pt.y) / (pt.x - poly_pt.x)
            standoff_x = self.standoff_dist / dp * abs(pt.x - poly_pt.x)

            if pt.x < poly_pt.x:
                pose.x = pt.x - standoff_x
            else:
                pose.x = pt.x + standoff_x

            pose.y = m*pose.x - m*pt.x + pt.y

        pose.theta = atan2(pose.y - poly_pt.y, poly_pt.x - pose.x)

        return pose
