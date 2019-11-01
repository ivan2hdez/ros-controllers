/**
 * 03_test.cpp node to test 03_controllers
 *
 * This program selects a control law and a sequence of paths to follow
 * waiting for the robot to finish each
 *
 * Publishes to:
 *    /auto_driver/next_pose	(geometry_msgs/Pose2D) next pose in path
 *    /auto_driver/set_control_law (std_msgs/String) either of
 *        "MoveToPoint", "FollowPath", "MoveToPose", "FollowLine"
 * Subscribes to:
 *    /AutoNOMOS_mini/real_pose_from_gazebo   (geometry_msgs/Pose2D)
 * Services requested:
 *    /auto_driver/reset_path   (to reset the path)
 *    /gazebo/reset_simulation   (to reset simulation and move robot to origin)
 *    /auto_driver/is_goal_set	(to see if robot got to goal)
 */

#include "03_robot.cpp"

#include <deque>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>


deque<geometry_msgs::Pose2D> path;

double x, y, theta;

#define ros_buffer_size 1 // number of messages to buffer before starting to discard
#define ros_loop_rate 1 // 2Hz is the frequency of the ros loop

void PoseCallback(const geometry_msgs::Pose2D& msgIn) {
  x = msgIn.x;
  y = msgIn.y;
  theta = msgIn.theta;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "03_controllers"); // Node 03_controllers
  ros::NodeHandle nh; // Main access point to communications with ROS

  // publisher for setting next pose in paths
  ros::Publisher  next_pose_pub = nh.advertise<std_msgs::Pose2D>("/auto_driver/next_pose", ros_buffer_size);

  // publisher for setting control law
  ros::Publisher contro_law_pub = nh.advertise<std_msgs::String>("/auto_driver/set_control_law", ros_buffer_size);
  
  // service client to reset path requests
  ros::ServiceClient reset_path_clnt = nh.ServiceClient<std_srvs::Empty>("auto_driver/reset_path");

  // add a service client to gazebo/reset_simulation
  ros::ServiceClient reset_simulation_clnt = nh.ServiceClient<std_srvs::Empty>("gazebo/reset_simulation");

  // add a service client to gazebo/reset_simulation
  ros::ServiceClient is_goal_set_clnt = nh.ServiceClient<std_srvs::Empty>("auto_driver/is_goal_set");

  std_msgs::Pose2D new_pose;
  std_msgs::String control_law;

  ros::Rate loop_rate(ros_loop_rate);
  double delta_t = 1.0/ros_loop_rate;
  int n_times;
 
  std::vector<string> control_laws = { "FollowPath", "MoveToPoint"};
  std::array<array<double, 3>, 5> poses = { {2.0, 2.0, 30*M_PI/180},
					    {2.0, 2.0, 30*M_PI/180},
					    {2.0, 2.0, 30*M_PI/180},
					    {2.0, 2.0, 30*M_PI/180},
					    {2.0, 2.0, 30*M_PI/180}
                                          };

  for (const auto& control_law: control_laws) {
    reset_path_clnt.call();
    reset_simulation_clnt.call();
    control_law.data = control_law;
    control_law_pub.publish(control_law);
    
    for (const auto& pose: poses) { //iterator pose = poses.begin(); pose != poses.end() && ros::ok(); pose++) {
      new_pose.x = pose[0];
      new_pose.y = pose[1];
      new_pose.theta = pose[2];
      next_pose_pub.publish(new_pose);
      // remaining necessary calls for ROS loop and callbacks
      loop_rate.sleep(); // waits what necessary to keep loop_rate
      ros::spinOnce(); // handles callbacks
      n_times--;
    }

    while(is_goal_set_clnt.call() && ros::ok()) {// wait until path is completed
      loop_rate.sleep();
      ros::spinOnce();
    }
  }

  return 0;
}
