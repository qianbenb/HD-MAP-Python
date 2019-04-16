#!/usr/bin/env python
# coding:utf-8

# Simple data_fetcher from .bag record file

import rospy
import std_msgs.msg
import sensor_msgs.msg

import rosbag
import yaml


# the data listener
def listener():
    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber('/camera_trigger', std_msgs.msg.Header, callback_trigger)
    # rospy.Subscriber('/velocity', std_msgs.msg.Float64MultiArray, callback_id)
    # rospy.Subscriber('/velocity_stamp', std_msgs.msg.Header, callback_stamp)
    # rospy.Subscriber('/gps_filtered', sensor_msgs.msg.NavSatFix, callback_gps_filtered)
    # rospy.Subscriber('/yaw_filtered', std_msgs.msg.Float64, callback_yaw_angle_filtered)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# main data fetcher program
def fetcher():

    # open the bag file
    bag = rosbag.Bag('Ladybug_Lidar_Calibration.bag')

    # get topics & types of the topic

    topics = bag.get_type_and_topic_info()[1].keys()

    print topics

    # close the bag file

    bag.close()

    # 接受处理好的velocity和trigger话题和yaw angle话题
    listener()


if __name__ == '__main__':
    try:
        fetcher()
    except rospy.ROSInterruptException:
        pass


