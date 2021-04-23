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

  scp::TOP top = scp::TOP(200., 101);
  // scp::TOP top = scp::TOP(1., 101);
  // scp::TOP top = scp::TOP(200., 81);

  // // Case 1: attitude doesn't change
  // top.x0 << 7.35, 5.0, 5.0, 0.0, 0, 0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  // top.xg << 10.0, 6.0, 6.0, 0, 0, 0, 0.0, 0.0, 0.0, 1.0, 0, 0, 0;

  // // Case 2: attitude changes - doesn't work w or wo slerp
  // top.x0 << 7.35, 5.0, 5.0, 0.0, 0, 0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  // top.xg << 10.0, 6.0, 6.0, 0, 0, 0, 0.5, 0.5, 0.5, 0.5, 0, 0, 0;

  // // Case 3: attitude changes
  // top.x0 << 7.35, 5.0, 5.0, 0.0, 0, 0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  // top.xg << 10.0, 6.0, 6.0, 0, 0, 0, 0.0, 0.0, 1.0, 0.0, 0, 0, 0;

  // // Case 4: position same, attitude changes - works without force constraints, doesn't work with force constraints
  // // "works" without force constraints - moment is 0 but quat is changing 
  // top.x0 << 10.0, 6.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  // top.xg << 10.0, 6.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;

  // Case 5: everything same
  top.x0 << 7.35, 5.0, 5.0, 0.0, 0, 0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  top.xg << 7.35, 5.0, 5.0, 0.0, 0, 0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

  // Changed SimpleCosts to not have gradient and hessian for state xg
  // There are no force constraints right now
  // Case 5 all zeros (except forces are 1e-20 and moments are exactly 0)
  // Case 1 works moves from point to point with no rotation
  // Case 4 - with slerp - works BUT moments are all zero (yet it rotates) 
  // Case 4 - without slerp - works BUT moments are all zero (yet it rotates)

  // Add force constraints back in 
  // Case 5 with normal des_accel - WORKS all zeros (except forces are 1e-20 and moments are exactly 0)
  // Case 5 but des_accel = 0 - WORKS  all zeros (except forces are 1e-20 and moments are exactly 0)
  // Case 4 with normal des_accel - works BUT moments are all zero (yet it rotates) 
  
  // Change abs_tol_ from 1e-10 to 1e-5 
  // Case 4 -- works BUT moments are all zero (yet it rotates)

  // Add rel_tol_ and also set to 1e-5
  // Case 4 -- solver fails

  // Set rel_tol_ to 1e-4
  // Case 4 -- solver fails

  // Set rel_tol_ to 1e-3
  // Case 4 -- ???

  // Get rid of UpdateDynamics() in Solve() 
  // Case 4 - rotating without moment (sum of torques = 0)
  // Note that the warm start does not have changing quaternions
  // Changed rotational dynamics to be my simplified version
  // Doing update of fs everywhere
  // Printing violation of rotational dynamics, which turns out to be non-zero ~ 0.0098
  // set primal and dual infeasibility tolerances to 1e-5 - nothing changed
  // Removing goal boundary conditions - all violations go to 0
  // Looked at goal boundary condition violations - also non-zero! About 0.0098 
  // Trying to tighten equalities
  // setting rho high and sigma low

  // Case 4 - rotation without moment (sum of torques = 0)
  // added factor = 1000
  // change to Tf = 1
  // change N to 10 - solver failed
  // change N to 50 - solver failed
  // change Tf to 200 - solver failed
  // Try Tf = 200, N = 101 again - works
  // Tf = 200, N = 51 fails Tf = 200, N = 81 fails
  // added factor to update constraints 
  // Trying factor = 1000, Tf = 200, N = 101 again - it fails!
  // changing factor = 1.0, Tf = 200, N = 101 again - it works!
  // using Abhi's nl dynamics - factor = 1.0, it works with torques 0
  // factor = 1000.0, solver failed
  // factor = 100.0, solver failed again
  // get rid of force constraints nothing
  // still using Abhi's nl dynamics - updateFs() first

  // wrote a simplified quaternion dynamics only (due to ang velocity)
  // Case 1 no attitude change works
  // Case 4 rotates again without moment (ok) but also without change in ang velocity (why)
  // Changing initialization to ang velocity = 0.1 --> solver fails 
  // Changing initialization so everything is 0.0

  // Drastically simplified traj_opt.cpp took out most stuff
  // Change initialization to be equal to x0 everywhere

  // Change rotation dynamics to multiply by angular velocity 
  // Ang velocity is non-zero to cause change in quaternion (yay)
  // Quaternions are not normalized
  // Now they are! Results here: Rotation occurs (looks like due to angular velocity though the change is sudden) 
  // SAVEPOINT ABOVE

  // Decreased desired alpha - no diff because no control constraints (and no controls to ang velocity hook up)
  // Implemented Euler's equations - moment works! goes positive then negative!
  // SAVEPOINT ABOVE
  
  // Case 2 both position and attitude change - appears to work with both torques and controls
  

  // std::cout << "The num vars is " << top.GetNumTOPVariables() << "\n";
  // std::cout << "The num constraints is " << top.GetNumTOPConstraints() << "\n";

  auto start = std::chrono::high_resolution_clock::now();
  if (!top.Solve()) {
    std::cout << "Solver iterations: " << top.n_iter << std::endl;
    std::cout << "Solver failed!" << std::endl;

  } else {
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    std::cout << "Solver worked in " << duration.count()/1e6 << "s!" << std::endl;

    // bool force_con_satisfied = true;
    // float torque = 0.0;
    // for (size_t ii = 0; ii < top.N-1; ii++) {
    //   float force = 0.0;
    //   for (size_t jj = 0; jj < 3; jj++) force += std::abs(top.Uprev[ii](jj));
    //   if (force > top.mass * top.desired_accel_) {
    //     std::cout << "Force constraint violated at time "<< ii << " by " << force - top.mass * top.desired_accel_ << std::endl;  
    //     force_con_satisfied = false; 
    //     // break;
    //   }
    //   for (size_t jj = 3; jj < 6; jj++) torque += std::abs(top.Uprev[ii](jj));
    // }
    // std::cout << "Torque sum for all times " << torque << std::endl;
    // if (force_con_satisfied) {
    //   std::cout << "Force constraint satisfied" << std::endl;
    // } else {
    //   std::cout << "Force constraint violated" << std::endl;
    // }
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

  float qx_ii = 0.0;
  float qy_ii = 0.0;
  float qz_ii = 0.0;
  float qw_ii = 1.0;
  size_t print_ctr = 0;

  while (ros::ok()) {
    if (top.solved_) {
      x_ii = top.Xprev[print_ctr](0);
      y_ii = top.Xprev[print_ctr](1);
      z_ii = top.Xprev[print_ctr](2);
      qx_ii = top.Xprev[print_ctr](6);
      qy_ii = top.Xprev[print_ctr](7);
      qz_ii = top.Xprev[print_ctr](8);
      qw_ii = top.Xprev[print_ctr](9);

      print_ctr++;
      if (print_ctr == top.N) {
        print_ctr = 0;
      }
    }

    tf::StampedTransform body_tf_stamped; 
    tf::Transform body_tf;
    body_tf.setOrigin(tf::Vector3(x_ii,y_ii,z_ii));
    body_tf.setRotation(tf::Quaternion(qx_ii,qy_ii,qz_ii,qw_ii));
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
