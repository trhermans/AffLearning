from math import hypot
import rospy
from geometry_testing import point_in_polygon, point_polygon_intersection

class GreedyCleanupPlanner:

    def __init__(self):
        self.cleanup_zones = None

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

        for i, place in enumerate(cleanup_path):
            rospy.loginfo("[%d]: (%g, %g)" %
                          (cleanup_path_ids[i], place.x, place.y))

        return cleanup_path
