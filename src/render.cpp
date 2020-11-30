#include <ros/ros.h>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <math.h>

#include <iostream>
#include <chrono>

#include "OsqpEigen/OsqpEigen.h"
#include <scp/traj_opt.h>

const double degree = M_PI/180;
#define PI 3.14159265

visualization_msgs::Marker line_strip;

void setup_markers() {
  line_strip.header.frame_id = "/map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns ="points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::POINTS;
  line_strip.scale.x = 0.1;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  line_strip.lifetime = ros::Duration();

  size_t n_points = 12;
  float t_f = 7.0;
  for (size_t ii = 0; ii < n_points; ii++) {
    float t_ii = (float)ii/(float)n_points*t_f;
    geometry_msgs::Point p;
    p.x = p.y = 0;
    p.z = -0.5*t_ii*t_ii + 1*t_ii + 1.;
    line_strip.points.push_back(p);
  }
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "state_publisher");

  ros::NodeHandle nh;
  ros::Rate r(10);

  // scp::TOP top = scp::TOP(20., 51);
  // top.x0 << 0.35, -0.5, 0., 0, 0, 0, 0.5, 0.5, 0.5, 0.5, 0, 0, 0;
  // top.xg << 0.35, -0.5, 0., 0, 0, 0, 0.5, 0.5, 0.5, 0.5, 0, 0, 0;
  // std::cout << "The num vars is " << top.GetNumTOPVariables() << "\n";
  // std::cout << "The num constraints is " << top.GetNumTOPConstraints() << "\n";

  auto start = std::chrono::high_resolution_clock::now();
  // if (!top.Solve()) {
  //   std::cout << "Solver failed!" << std::endl;
  // } else {
  //   auto stop = std::chrono::high_resolution_clock::now();
  //   auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
  //   std::cout << "Solver worked in " << duration.count()/1e6 << "s!" << std::endl;
  // }

  setup_markers();

  tf::TransformBroadcaster br;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("trajectory_marker", 10);

  float t_ii = 0;
  float t_f = 7.0;

  while (ros::ok()) {
    float z_ii  = -0.5*t_ii*t_ii + 0.1*t_ii + 1.;
    z_ii  = 4.0*sin(t_ii/t_f * (2*PI/t_f));
    t_ii += 0.5; 

    ROS_DEBUG_STREAM("Hello " << "World");

    tf::StampedTransform body_tf_stamped; 
    tf::Transform body_tf;
    body_tf.setOrigin(tf::Vector3(1.0,1.0,z_ii));
    body_tf.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
    body_tf_stamped = tf::StampedTransform(body_tf, ros::Time::now(), "map", "body");
    body_tf_stamped.stamp_ = ros::Time::now();

    br.sendTransform(body_tf_stamped);
    marker_pub.publish(line_strip);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
