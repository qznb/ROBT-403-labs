#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped default_pose;
  geometry_msgs::PoseStamped target_pose1;
  geometry_msgs::PoseStamped target_pose2;
  geometry_msgs::PoseStamped target_pose3;
  geometry_msgs::PoseStamped target_pose4;

  current_pose = move_group.getCurrentPose();

  default_pose = current_pose;

  target_pose1 = current_pose;
  target_pose1.pose.position.x = target_pose1.pose.position.x - 0.5;

  target_pose2 = target_pose1;
  target_pose2.pose.position.y = target_pose2.pose.position.y + 0.5;

  target_pose3 = target_pose2;
  target_pose3.pose.position.x = target_pose3.pose.position.x - 0.5;

  target_pose4 = target_pose3;
  target_pose4.pose.position.y = target_pose4.pose.position.y - 0.5;

  ros::Rate loop_rate(50);
  while (ros::ok()) {
    move_group.setApproximateJointValueTarget(target_pose1);
    move_group.move();
    current_pose = move_group.getCurrentPose();
    if (abs(current_pose.pose.position.x - target_pose1.pose.position.x) < 0.01 && abs(current_pose.pose.position.y - target_pose1.pose.position.y) < 0.01) {
      break;
    }
    loop_rate.sleep();
  }
  ROS_INFO("Target pose");

  while (ros::ok()) {
    move_group.setApproximateJointValueTarget(target_pose2);
    move_group.move();
    current_pose = move_group.getCurrentPose();
    if (abs(current_pose.pose.position.x - target_pose2.pose.position.x) < 0.01 && abs(current_pose.pose.position.y - target_pose2.pose.position.y) < 0.01) {
      break;
    }
    loop_rate.sleep();
  }
  ROS_INFO("Target pose");


  while (ros::ok()) {
    move_group.setApproximateJointValueTarget(target_pose3);
    move_group.move();
    current_pose = move_group.getCurrentPose();
    if (abs(current_pose.pose.position.x - target_pose3.pose.position.x) < 0.01 && abs(current_pose.pose.position.y - target_pose3.pose.position.y) < 0.01) {
      break;
    }
    loop_rate.sleep();
  }
  ROS_INFO("Target pose");


  while (ros::ok()) {
    move_group.setApproximateJointValueTarget(target_pose4);
    move_group.move();
    current_pose = move_group.getCurrentPose();
    if (abs(current_pose.pose.position.x - target_pose4.pose.position.x) < 0.01 && abs(current_pose.pose.position.y - target_pose4.pose.position.y) < 0.01) {
      break;
    }
    loop_rate.sleep();
  }
  ROS_INFO("Target pose");

  while (ros::ok()) {
    move_group.setApproximateJointValueTarget(target_pose1);
    move_group.move();
    current_pose = move_group.getCurrentPose();
    if (abs(current_pose.pose.position.x - target_pose1.pose.position.x) < 0.01 && abs(current_pose.pose.position.y - target_pose1.pose.position.y) < 0.01) {
      break;
    }
    loop_rate.sleep();
  }
  ROS_INFO("Target pose");

  while (ros::ok()) {
    move_group.setApproximateJointValueTarget(default_pose);
    move_group.move();
    current_pose = move_group.getCurrentPose();
    if (abs(current_pose.pose.position.x - default_pose.pose.position.x) < 0.01 && abs(current_pose.pose.position.y - default_pose.pose.position.y) < 0.01) {
      break;
    }
    loop_rate.sleep();
  }
  ROS_INFO("Target pose");



  ROS_INFO("Done");
  ros::shutdown();
  return 0;
}
