#! /usr/bin/env python

import pdb
import sys
import rospy
import copy
import rosbag
from visualization_msgs.msg import MarkerArray, Marker

def retrieve_messages(zones_fn):
  bag = rosbag.Bag(zones_fn, 'r')

  for topic,msg,t in bag.read_messages(topics=['/mob/mapper/zones']):
      print(msg)
  bag.close()

  new_msg = MarkerArray()
  for ii in range(len(msg.markers)):
      marker = copy.deepcopy(msg.markers[ii])
      marker.header.frame_id = 'map'
      new_msg.markers.append(marker)
  return new_msg

if __name__ == '__main__':
    rospy.init_node('zones', anonymous=True)
    rate = rospy.Rate(10)

    zones_fn = '/home/acauligi/rviz_ws/src/bee_viz/worlds/zones.bag'
    zones_msg = retrieve_messages(zones_fn)
    zones_pub = rospy.Publisher('/zones', MarkerArray, queue_size=10)

    while not rospy.is_shutdown():
       zones_pub.publish(zones_msg)
       rate.sleep()
