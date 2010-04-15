#include "ros/ros.h"
#include "overhead_tracking/CleanupObjectArray.h"
#include "geometry_msgs/Pose2D.h"

class CleanupPlannerNode
{
 public:
  CleanupPlannerNode(ros::NodeHandle &n) :
      n_(n)
  {
    cleanup_objs_sub_ = n_.subscribe("cleanup_objs", 1,
                                    &CleanupPlannerNode::objectCallback, this);
    robot_pose_sub_ = n_.subscribe("robot_pose", 1,
                                  &CleanupPlannerNode::robotPoseCallback, this);
  }

  void objectCallback(const overhead_tracking::CleanupObjectArrayConstPtr &msg)
  {
  }

  void robotPoseCallback(const geometry_msgs::Pose2DConstPtr &msg)
  {
  }

  void spin()
  {
    while(n_.ok()) {
      
      ros::spinOnce();
    }
  }

 protected:
  ros::Subscriber cleanup_objs_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::NodeHandle n_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cleanup_planner");
  ros::NodeHandle n;
  CleanupPlannerNode cleanup_node(n);
  cleanup_node.spin();
}
