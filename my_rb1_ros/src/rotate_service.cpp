#include "my_rb1_ros/Rotate.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class RotateService {

public:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;
  ros::Subscriber odom_sub_;
  ros::Publisher vel_pub;
  geometry_msgs::Twist vel_msg;
  double initial_yaw = 0.0;
  double current_yaw;
  double target_yaw;
  double delta_yaw; 

  RotateService() {
    
    service_ = nh_.advertiseService("/rotate_robot",
                                    &RotateService::rotateCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &RotateService::odomCallback, this);
    ROS_INFO("Rotate service is ready.");
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }

  void rotateStart() {
    if (delta_yaw > 0) {
      vel_msg.angular.z = 0.2; 
      ROS_INFO("Rotating counterclockwise...");
    } else {
      vel_msg.angular.z = -0.2; 
      ROS_INFO("Rotating clockwise...");
    }
    vel_pub.publish(vel_msg);
  }

  void rotateStop() {
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
  }

  bool rotateCallback(my_rb1_ros::Rotate::Request &req,
                      my_rb1_ros::Rotate::Response &res) 
  {
    ROS_INFO("The Service has been called");

    
    delta_yaw = req.degrees * M_PI / 180.0;
    target_yaw = initial_yaw + delta_yaw;

    
    while (std::abs(current_yaw - target_yaw) >= 0.05) {
      rotateStart();
      ROS_INFO("Rotation in progress");
      ros::spinOnce();
    }

    rotateStop();
    initial_yaw = current_yaw;
    res.result = "Rotation Successful";
    ROS_INFO("Rotation has been successful");
    return true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    current_yaw =
        atan2(2.0 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_node");
  RotateService rotate_service;
  ros::spin();
  return 0;
}