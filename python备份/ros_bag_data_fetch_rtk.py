#!/usr/bin/env python
# coding:utf-8

# Simple data_fetcher from .bag record file

import rospy
import std_msgs.msg
import sensor_msgs.msg

import rosbag
import yaml


# yaw angle filtered part ##timestamp##

# 错误的时间戳全部为-1
n280_timestamp = -1.0

def callback_gps_filtered(data):

    # 不断收集gps时间戳数据，存储到list
    global n280_timestamp

    # n280_timestamp.append(data.header.stamp.to_sec())
    n280_timestamp = data.header.stamp.to_sec()

    # print 'In gps stamp is: ', n280_timestamp, '\n'
    # rospy.loginfo(data)


# yaw angle filtered part ##get toogether##
def callback_yaw_angle_filtered(data):

    global n280_timestamp

    # print 'In yaw stamp is: ', n280_timestamp , '\n'

    f_yaw_angle = open('Yaw_Angle_Filtered.m', 'a')
    f_yaw_angle.writelines(format(n280_timestamp, '.7f') + ',' + str(data.data) + ';' + '\n')

    #rospy.loginfo(data.data)


# wheel speed part

def callback_stamp(data):
    f_trigger = open('Wheel_Speed_Stamp_Id.m', 'a')
    f_trigger.writelines(format(data.stamp.to_sec(), '.7f') + ',' + str(data.seq) + ';' + '\n')

    # rospy.loginfo(data.stamp.to_sec())
    # rospy.loginfo(data.seq)


def callback_id(data):
    f_trigger = open('Wheel_Speed_Id_Velocity.m', 'a')
    f_trigger.writelines(str(int(data.data[0])) +',' + format(data.data[1], '.7f') + ';' + '\n')

    # rospy.loginfo(str(int(data.data[0])))
    # rospy.loginfo(format(data.data[1], '.7f'))


# camera trigger part

def callback_trigger(data):

    f_trigger = open('Panorama_Camera_Trigger.m', 'a')
    f_trigger.writelines(format(data.stamp.to_sec(), '.7f') + ',' + str(data.seq) + ';' + '\n')

# rtk fix part

def callback_rtk_fix(data):

    f_trigger = open('rtk_fix.m', 'a')
    f_trigger.writelines(format(data.header.stamp.to_sec(), '.7f') + ',' + str(data.latitude) +',' + str(data.longitude)+ ';' + '\n')
    # print data.stamp.to_sec()


# the data listener
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera_trigger', std_msgs.msg.Header, callback_trigger)
    rospy.Subscriber('/velocity', std_msgs.msg.Float64MultiArray, callback_id)
    rospy.Subscriber('/velocity_stamp', std_msgs.msg.Header, callback_stamp)
    rospy.Subscriber('/gps_filtered', sensor_msgs.msg.NavSatFix, callback_gps_filtered)
    rospy.Subscriber('/yaw_filtered', std_msgs.msg.Float64, callback_yaw_angle_filtered)
    rospy.Subscriber('/rtk/fix', sensor_msgs.msg.NavSatFix, callback_rtk_fix)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# main data fetcher program
def fetcher():

    # open the bag file
    bag = rosbag.Bag('Graduate_Rosbag.bag')

    # get topics & types of the topic

    topics = bag.get_type_and_topic_info()[1].keys()

    print topics

    # imu output .m file
    # data variable prepare → write into the .m file
    # timestamp seconds from 1970.1.1 00:00  &   z_angular_speed

    imu_t = []

    imu_z_angular_speed = []

    for topic, msg, t in bag.read_messages(topics=[topics[2]]):
        # not t——bag time , should be the sensor time stamp,msg.header.stamp
        imu_t.append(msg.header.stamp.to_sec())
        imu_z_angular_speed.append(msg.angular_velocity.z)

    print len(imu_t)
    print len(imu_z_angular_speed)

    f_imu = open('Mti_Mtx_Z_Angular_Velocity.m', 'w+')

    f_imu.writelines('MTI_MTX_Z_ANGULAR_VELOCITY = [' + '\n')

    imu_line_num = 2

    for item_t, item_z in zip(imu_t, imu_z_angular_speed):

        # format(item_t,'.7f')可以保留小数精度，而str对大数会截成两位小数，小的数不会

        # second line to last -1 line
        if 1 < imu_line_num < len(imu_t)+1:
            f_imu.writelines(format(item_t, '.7f') + ',' + str(item_z) + ';' +'\n')
            imu_line_num += 1
            continue

        # the last line
        elif imu_line_num == len(imu_t)+1:
            f_imu.writelines(format(item_t, '.7f') + ',' + str(item_z) + '];')
            continue

    f_imu.close()

    # close the bag file

    bag.close()

    # trigger output .m file
    f_trigger = open('Panorama_Camera_Trigger.m', 'w+')

    f_trigger.writelines('PANORAMA_CAMERA_TRIGGER = [' + '\n')

    f_trigger.close()

    # wheel_speed output .m file
    f_wheel_speed_id = open('Wheel_Speed_Id_Velocity.m', 'w+')

    f_wheel_speed_id.writelines('WHEEL_SPEED_ID_VELOCITY = [' + '\n')

    f_wheel_speed_id.close()

    f_wheel_speed_stamp = open('Wheel_Speed_Stamp_Id.m', 'w+')

    f_wheel_speed_stamp.writelines('WHEEL_SPEED_STAMP_ID = [' + '\n')

    f_wheel_speed_stamp.close()

    # yaw_angle output .m file
    f_yaw_angle = open('Yaw_Angle_Filtered.m', 'w+')

    f_yaw_angle.writelines('YAW_ANGLE_FILTERED = [' + '\n')

    f_yaw_angle.close()

    # rtk_fix output .m file
    f_rtk_fix = open('rtk_fix.m', 'w+')

    f_rtk_fix.writelines('RTK_FIX = [' + '\n')

    f_rtk_fix.close()

    # 接受处理好的velocity和trigger话题和yaw angle话题
    listener()


if __name__ == '__main__':
    try:
        fetcher()
    except rospy.ROSInterruptException:
        pass


