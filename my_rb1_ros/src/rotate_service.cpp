#include "my_rb1_ros/Rotate.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/tf.h>

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
  double target_yaw_n;
  double delta_yaw; 
  double delta_yaw_n;

  RotateService() {
    
    service_ = nh_.advertiseService("/rotate_robot",
                                    &RotateService::rotateCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &RotateService::odomCallback, this);
    ROS_INFO("Rotate service is ready.");
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }
  
  double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
  }

  void rotateStart() {
    if (delta_yaw_n > 0) {
      vel_msg.angular.z = 0.5; 
    } else {
      vel_msg.angular.z = -0.5; 
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
    delta_yaw_n = normalizeAngle(delta_yaw);
    
    target_yaw = initial_yaw + delta_yaw_n;
    
    target_yaw_n = normalizeAngle(target_yaw);
    
    // ros::Rate rate(10);

    while (std::abs(current_yaw - target_yaw_n) >= 0.08 && std::abs(delta_yaw)<=((2*M_PI)-0.0872665)) {
      ros::spinOnce();
      rotateStart();
    //   rate.sleep();
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
    double unnormalized_yaw =
        atan2(2.0 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);
    current_yaw = normalizeAngle(unnormalized_yaw);
    }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_node");
  RotateService rotate_service;
  ros::spin();
  return 0;
}