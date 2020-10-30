#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>

const double degree = M_PI/180;
#define PI 3.14159265

tf::StampedTransform body_tf_stamped; 
visualization_msgs::Marker line_strip;
// const int Ngeo = 1;
// visualization_msgs::MarkerArray geo_array; 
// sensor_msgs::JointState joint_state;
// std_msgs::Float64MultiArray joint_msg;

// void define_robot_geometry() {
//   visualization_msgs::Marker marker_body;
//   marker_body.ns = "rb";
//   marker_body.id = 1;
//   marker_body.type = visualization_msgs::Marker::SPHERE;
//   marker_body.scale.x = 2*0.264;
//   marker_body.scale.y = 2*0.264;
//   marker_body.scale.z = 2*0.264;
//   geo_array.markers.push_back(marker_body);
// 
//   for (int i = 0; i < Ngeo; i++) {
//     geo_array.markers[i].header.frame_id = "/map";
//     geo_array.markers[i].header.stamp = ros::Time::now();
//     geo_array.markers[i].action = visualization_msgs::Marker::ADD;
//     geo_array.markers[i].pose.position.x = 1;
//     geo_array.markers[i].pose.position.y = 1;
//     geo_array.markers[i].pose.position.z = 1;
//     geo_array.markers[i].pose.orientation.x = 0.0;
//     geo_array.markers[i].pose.orientation.y = 0.0;
//     geo_array.markers[i].pose.orientation.z = 0.0;
//     geo_array.markers[i].pose.orientation.w = 1.0;
//     geo_array.markers[i].color.r = 0.0f;
//     geo_array.markers[i].color.g = 0.0f;
//     geo_array.markers[i].color.b = 0.0f;
//     geo_array.markers[i].color.a = 0.0;
//     geo_array.markers[i].lifetime = ros::Duration();
//   }
// }

// void define_obstacle_geometry() {
// }

// void robot_callback(const geometry_msgs::PoseArray &pose_array) { 
//   ROS_INFO_STREAM("Robot pose array received");
//   if (pose_array.poses.size() != Ngeo) {
//     ROS_ERROR("Number of robot poses received does not match defined geometry!");
//     return;
//   }
// 
//   for (int i = 0; i < Ngeo; i++) {
//     geo_array.markers[i].pose.position.x    = pose_array.poses[i].position.x;
//     geo_array.markers[i].pose.position.y    = pose_array.poses[i].position.y;
//     geo_array.markers[i].pose.position.z    = pose_array.poses[i].position.z;
//     geo_array.markers[i].pose.orientation.x = pose_array.poses[i].orientation.x;
//     geo_array.markers[i].pose.orientation.y = pose_array.poses[i].orientation.y;
//     geo_array.markers[i].pose.orientation.z = pose_array.poses[i].orientation.z;
//     geo_array.markers[i].pose.orientation.w = pose_array.poses[i].orientation.w;
//     geo_array.markers[i].color.a = 1.0;
//   }
// }


void body_tf_callback(const geometry_msgs::TransformStamped& body_tf_msg) { 
  tf::transformStampedMsgToTF(body_tf_msg, body_tf_stamped);
}

// void joint_state_callback(const geometry_msgs::PolygonStamped& joint_state_msg) { 
//   for (int i = 0; i < joint_state.position.size(); i++) joint_state.position[i] = joint_state_msg.polygon.points[i].x;
// }

void setup_markers() {
  line_strip.header.frame_id = "/map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns ="points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  line_strip.lifetime = ros::Duration();
  geometry_msgs::Point p;
  p.x = p.y = p.z = 0.0;
  line_strip.points.push_back(p);
  p.x = p.y = p.z = 1.0;
  line_strip.points.push_back(p);
}

void setup_messages() {
  tf::Transform body_tf;
  body_tf.setOrigin(tf::Vector3(1.0,1.0,1.0));
  body_tf.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
  body_tf_stamped = tf::StampedTransform(body_tf, ros::Time::now(), "map", "body");
  
  // std::vector<std::string> joint_names = {"top_aft", 
  //               "top_aft_arm_proximal_link",
  //               "top_aft_arm_distal_link",
  //               "top_aft_gripper_left_proximal_link",
  //               "top_aft_gripper_left_distal_link",
  //               "top_aft_gripper_right_proximal_link",
  //               "top_aft_gripper_right_distal_link"};
  // joint_state.name.resize(joint_names.size());
  // joint_state.position.resize(joint_names.size());
  // joint_state.velocity.resize(joint_names.size());
  // for (int i = 0; i < joint_state.position.size(); i++) {
  //   joint_state.name[i] = joint_names[i];
  //   joint_state.position[i] = 0;
  //   joint_state.velocity[i] = 0;
  // }
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "state_publisher");

  ros::NodeHandle nh;
  ros::Rate r(10);

  setup_messages();
  setup_markers();

  tf::TransformBroadcaster br;
  // ros::Subscriber joint_state_sub = nh.subscribe("/joint_states_", 1000, joint_state_callback);
  // ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Subscriber body_tf_sub = nh.subscribe("/body_tf", 1000, body_tf_callback);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // define_robot_geometry();
  // define_obstacle_geometry();

  while (ros::ok()) {
    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) return 0;
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    // for (int i = 0; i < Ngeo; i++) marker_pub.publish(geo_array.markers[i]);
    // for (int i = 0; i < Nobs; i++) marker_pub.publish(obs_array.markers[i]);

    body_tf_stamped.stamp_ = ros::Time::now();
    br.sendTransform(body_tf_stamped);

    // joint_state.header.stamp = ros::Time::now();
    // joint_pub.publish(joint_state);

    marker_pub.publish(line_strip);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


