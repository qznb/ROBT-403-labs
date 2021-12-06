#include <moveit/planning_scene_interface/planning_scene_interface.h>
  #include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include <vector>

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(0);
  double angle, cosined, sined, rad, temp_cos = 0, temp_sin = 0;
  double radius = 0.8;
  spinner.start();
  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped target_pose;

  current_pose = move_group.getCurrentPose();
  target_pose = current_pose;

  for(angle = 0; angle <= 360; angle = angle + 10) {
    target_pose = current_pose;
    ros::Rate loop_rate(50);
    rad = angle * 3.14159265 / 180;
    cosined = cos(rad);
    sined = sin(rad);
    current_pose = move_group.getCurrentPose();
    target_pose.pose.position.x = current_pose.pose.position.x - (radius - radius * cosined - (radius - radius * temp_cos));
    target_pose.pose.position.y = current_pose.pose.position.y + (radius * sined - radius * temp_sin);
    while (ros::ok()) {

      move_group.setApproximateJointValueTarget(target_pose);
      move_group.move();
      current_pose = move_group.getCurrentPose();

      if (abs(current_pose.pose.position.x - target_pose.pose.position.x) < 0.01 && abs(current_pose.pose.position.y - target_pose.pose.position.y) < 0.01) {
        break;
      }
    }
    temp_cos = cosined;
    temp_sin = sined;
    ROS_INFO("%f", angle);
    loop_rate.sleep();
  }

  ROS_INFO("Done");
  ros::shutdown();
  return 0;
}
