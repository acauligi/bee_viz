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

visualization_msgs::Marker setup_markers(scp::TOP &top) {
  visualization_msgs::Marker traj_strip;
  traj_strip.header.frame_id = "/map";
  traj_strip.header.stamp = ros::Time::now();
  traj_strip.ns ="points_and_lines";
  traj_strip.action = visualization_msgs::Marker::ADD;
  traj_strip.pose.orientation.w = 1.0;
  traj_strip.id = 1;
  traj_strip.type = visualization_msgs::Marker::POINTS;
  traj_strip.scale.x = 0.1;
  traj_strip.color.b = 1.0;
  traj_strip.color.a = 1.0;
  traj_strip.lifetime = ros::Duration();

  for (size_t ii = 0; ii < top.N; ii++) {
    geometry_msgs::Point p;
    p.x = top.Xprev[ii](0);
    p.y = top.Xprev[ii](1);
    p.z = top.Xprev[ii](2);
    traj_strip.points.push_back(p);
  }
  return traj_strip;
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "state_publisher");

  ros::NodeHandle nh;
  ros::Rate r(10);

  scp::TOP top = scp::TOP(20., 51);
  top.x0 << 10.35, 6.0, 6.0, 0, 0, 0, 0.5, 0.5, 0.5, 0.5, 0, 0, 0;
  top.xg << 10.0, 5.0, 5.0, 0, 0, 0, 0.5, 0.5, 0.5, 0.5, 0, 0, 0;
  // std::cout << "The num vars is " << top.GetNumTOPVariables() << "\n";
  // std::cout << "The num constraints is " << top.GetNumTOPConstraints() << "\n";

  auto start = std::chrono::high_resolution_clock::now();
  if (!top.Solve()) {
    std::cout << "Solver failed!" << std::endl;

  } else {
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    std::cout << "Solver worked in " << duration.count()/1e6 << "s!" << std::endl;

    bool acc_con_satisfied = true;
    for (size_t ii = 0; ii < top.N-1; ii++) {
      float acc = 0.0;
      for (size_t jj = 0; jj < 3; jj++) acc += std::abs(top.Uprev[ii](jj));
      if (acc > top.desired_accel_) {
        acc_con_satisfied = false; 
        break;
      }
    }
    if (acc_con_satisfied) {
      std::cout << "Force constraint satisfied" << std::endl;
    } else {
      std::cout << "Force constraint violated" << std::endl;
    }
  }

  visualization_msgs::Marker traj_strip = setup_markers(top);

  tf::TransformBroadcaster br;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("trajectory_marker", 10);
  ros::Publisher init_pub = nh.advertise<visualization_msgs::Marker>("init_marker", 10);
  ros::Publisher goal_pub = nh.advertise<visualization_msgs::Marker>("goal_marker", 10);

  // Init marker
  visualization_msgs::Marker init_marker;
  init_marker.header.frame_id = "/map";
  init_marker.header.stamp = ros::Time::now();
  init_marker.ns = "goal";
  init_marker.action = visualization_msgs::Marker::ADD;
  init_marker.pose.orientation.w = 1.0;
  init_marker.id = 1;
  init_marker.type = visualization_msgs::Marker::SPHERE;
  init_marker.scale.x = 0.25;
  init_marker.scale.y = 0.25;
  init_marker.scale.z = 0.25;
  init_marker.color.b = 1.0;
  init_marker.color.a = 1.0;
  init_marker.pose.position.x = top.x0(0);
  init_marker.pose.position.y = top.x0(1);
  init_marker.pose.position.z = top.x0(2);
  init_marker.lifetime = ros::Duration();

  // Goal marker
  visualization_msgs::Marker goal_marker;
  goal_marker.header.frame_id = "/map";
  goal_marker.header.stamp = ros::Time::now();
  goal_marker.ns = "goal";
  goal_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.id = 1;
  goal_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.scale.x = 0.25;
  goal_marker.scale.y = 0.25;
  goal_marker.scale.z = 0.25;
  goal_marker.color.r = 1.0;
  goal_marker.color.a = 1.0;
  goal_marker.pose.position.x = top.xg(0);
  goal_marker.pose.position.y = top.xg(1);
  goal_marker.pose.position.z = top.xg(2);
  goal_marker.lifetime = ros::Duration();

  float t_ii = 0;
  float t_f = 7.0;

  float x_ii = 0.0;
  float y_ii = 0.0;
  float z_ii = 0.0;
  size_t print_ctr = 0;

  while (ros::ok()) {
    if (top.solved_) {
      x_ii = top.Xprev[print_ctr](0);
      y_ii = top.Xprev[print_ctr](1);
      z_ii = top.Xprev[print_ctr](2);
      print_ctr++;
      if (print_ctr == top.N) {
        print_ctr = 0;
      }
    }

    tf::StampedTransform body_tf_stamped; 
    tf::Transform body_tf;
    body_tf.setOrigin(tf::Vector3(x_ii,y_ii,z_ii));
    body_tf.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
    body_tf_stamped = tf::StampedTransform(body_tf, ros::Time::now(), "map", "body");
    body_tf_stamped.stamp_ = ros::Time::now();

    br.sendTransform(body_tf_stamped);

    init_pub.publish(init_marker);
    goal_pub.publish(goal_marker);

    if (top.solved_) {
      marker_pub.publish(traj_strip);
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
