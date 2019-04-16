#!/usr/bin/env python
# coding:utf-8

# Simple data_fetcher from .bag record file

import rospy
import std_msgs.msg
# import tiggo_msgs.msg
import sensor_msgs.msg

import utm
import math

import rosbag
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Variable Initialize
# GPSInfoShop = pd.DataFrame(columns=['timestamp', 'lat', 'lon', 'y', 'x'])
GPSInfoShopList = []

# OrientationShop = pd.DataFrame(columns=['timestamp', 'heading', 'pitch', 'roll'])  # yaw,pitch,roll
# YawAngleInfoShop = pd.DataFrame(columns=['timestamp','heading'])
OrientationShopList = []


# GPSHeadingInfoShop = pd.DataFrame(columns=['timestamp', 'lat', 'lon', 'heading', 'heading_imu'])
GPSHeadingInfoShopList = []


iG = 0
iY = 0
iT = 0
iD = 0
iH = 0
iImu = 0
iDentity = -1
initialTriggerId = 0


TriggerCustomer = pd.DataFrame(columns=['triggerId','timestamp'])  # only one triggerID of five
TriggerCustomerLog = pd.DataFrame(columns=['triggerId', 'timestamp'])


def calcRollPitchYaw(w, x, y, z):
    """
    ref: http://blog.csdn.net/u012700322/article/details/52252305
    :param w: q = [w x y z]
    :param x: q = [w x y z]
    :param y: q = [w x y z]
    :param z: q = [w x y z]
    :return: yaw, pitch, roll
    """
    # Inertial z axis is down
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))  # x theta
    pitch = math.asin(2 * (w * y - z * x))  # y pitch
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))  # z sai
    return yaw, pitch, roll


# camera trigger part

def callback_trigger(msg):

    # A new mechanism: trigger once,send five with identity to avoid loss

    global iT
    global iD
    global iDentity

    global initialTriggerId

    # initialize once
    if iT == 0:
        initialTriggerId = msg.seq

    # print int(msg.seq) - initialTriggerId

    msg.stamp = rospy.get_rostime()

    if iDentity != int(msg.seq)- initialTriggerId:
        # cuz iDentity init as -1 ,so the first is already logged

        iDentity = int(msg.seq) - initialTriggerId  # relative id
        # SequenceId may not equal to triggerId


        # TriggerCustomer.loc[iT] = [int(msg.seq), msg.stamp.to_sec()]
        TriggerCustomer.loc[iT] = [iDentity,"{:.9f}".format(msg.stamp.secs+msg.stamp.nsecs*1.0/1e9)]
        iT += 1
        print iDentity

    TriggerCustomerLog.loc[iD] = [iDentity, "{:.9f}".format(msg.stamp.secs + msg.stamp.nsecs * 1.0 / 1e9)]
    iD += 1



# inertial gps fix part

def callback_inertial_gps_fix(msg):
    # print data.longitude.
    # print data.latitude
    # print data.header.stamp.to_sec() # 100HZ

    # put the timestamp, latitude, longitude into the GPSInfoShop dataframe, 用timestamp连接yaw_angle, YawAngleInfoShop
    global iG
    (x, y, _, _) = utm.from_latlon(msg.latitude, msg.longitude)
    # print utm.from_latlon(data.latitude,data.longitude)
    # print y
    # print x
    # temp = ["{:.9f}".format(msg.header.stamp.secs+msg.header.stamp.nsecs*1.0/1e9), msg.latitude, msg.longitude, y, x]
    msg.header.stamp = rospy.get_rostime()
    GPSInfoShopList.append(["{:.9f}".format(msg.header.stamp.secs+msg.header.stamp.nsecs*1.0/1e9), msg.latitude, msg.longitude, y, x])

    # GPSInfoShop.loc[iG] = ["{:.9f}".format(msg.header.stamp.secs+msg.header.stamp.nsecs*1.0/1e9), msg.latitude, msg.longitude, y, x]
    iG += 1

    # print "luck"


# heading
# def callback_heading(data):
#     # print data
#     global iH
#
#     HeadingShop.loc[iH] = [data.header.stamp.to_sec(),data.data, ]
#     iH += 1

def callback_heading(msg):
    """
    / *
    电子罗盘原始数据，以正北为0，从正北逆时针方向到正西再到正南，航向角输出从0 - 180，
    从正南逆时针方向到正东再到正北，航向角输出从 - 180
    到0，我们期望将航向角转化为ENU坐标系，
    即以正东为0，逆时针方向为正，航向角以角度表示，范围从0 - 360。
    角速度数据只用到z轴角速度，其余两个轴的方向还未确定，慎用。
    线性加速度数据基本不使用，这里坐标系不做变换，慎用。
    * /
    """


    global iImu

    # print msg.orientation
    # print '----------'

    yaw = msg.data

    print yaw

    msg.header.stamp = rospy.get_rostime()

    OrientationShopList.append(["{:.9f}".format(msg.header.stamp.secs+msg.header.stamp.nsecs*1.0/1e9), yaw])

    # OrientationShop.loc[iImu] = ["{:.9f}".format(msg.header.stamp.secs+msg.header.stamp.nsecs*1.0/1e9), yaw, pitch, roll]
    iImu += 1

# the data listener
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/Inertial/gps/fix', sensor_msgs.msg.NavSatFix, callback_inertial_gps_fix)
    # rospy.Subscriber('/velocity_stamp', std_msgs.msg.Header, callback_stamp) # xiaowu 计算四元数形成heading角度的话题
    # 最好还是在脚本里面把heading角直接改成正东方向为基准逆时针旋转的一个角度

    rospy.Subscriber('/camera_trigger', std_msgs.msg.Header, callback_trigger)
    # rospy.Subscriber('/Inertial/heading', Heading, callback_heading)
    rospy.Subscriber('/Inertial/imu/data', sensor_msgs.msg.Imu, callback_heading_imu_data)
    # rospy.Subscriber('/Inertial/heading', tiggo_msgs.msg.Heading, callback_heading)

    # /Inertial/gps/fix
    # /Inertial/imu/data
    # /can_frame

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# main data fetcher program
def fetcher():
    # 接受处理好的velocity和trigger话题和yaw angle话题
    listener()

    # shutdown
    rospy.signal_shutdown('the rosbag is over')

    # Variable Initialize
    GPSInfoShop = pd.DataFrame(GPSInfoShopList,columns=['timestamp', 'lat', 'lon', 'y', 'x'])
    OrientationShop = pd.DataFrame(OrientationShopList,columns=['timestamp', 'heading'])  # yaw,pitch,roll
    GPSHeadingInfoShop = pd.DataFrame(columns=['timestamp', 'lat', 'lon', 'heading', 'heading_imu'])

    GPSInfoShop.to_csv('GPSInfoShop.txt', header=None, index=None, sep=' ')
    TriggerCustomer.to_csv('TriggerCustomer.txt', header=None, index=None, sep=' ')
    TriggerCustomerLog.to_csv('TriggerCustomerLog.txt', header=None, index=None, sep=' ')

    # print GPSInfoShop
    # print TriggerCustomer

    # 由GPSInfoShop的下一行和这一行计算这一行的heading
    GPSHeadingInfoShop['timestamp'] = GPSInfoShop['timestamp']
    GPSHeadingInfoShop['lat'] = GPSInfoShop['lat']
    GPSHeadingInfoShop['lon'] = GPSInfoShop['lon']

    # GPSHeadingInfoShop['heading'] = GPSHeadingInfoShop.apply(lambda row:GPSHeadingCalc(row['lat'],row['lon']),axis=1)

    # GPSHeadingInfoShop['heading'] = np.arctan(df['lat'].shift() - df['lat'] / df['ypos'].shift() - df['ypos'])
    # math.atan2(rtk_fix.loc[i + 1]['y'] - rtk_fix.loc[i]['y'], rtk_fix.loc[i + 1]['x'] - rtk_fix.loc[i]['x'])

    # GPSHeadingInfoShop['heading'] = np.arctan2(GPSInfoShop['y'].shift()-GPSInfoShop['y'],GPSInfoShop['x'].shift()-GPSInfoShop['x'])
    GPSHeadingInfoShop['heading'] = (np.arctan2(GPSInfoShop['y'] - GPSInfoShop['y'].shift(), GPSInfoShop['x'] - GPSInfoShop['x'].shift())).shift(-1)


    # actually gps and imu data is all output by Inertial, so the data is equal and sync
    GPSHeadingInfoShop['heading_imu'] = OrientationShop['heading']

    # print GPSHeadingInfoShop

    # 文件输出
    GPSHeadingInfoShop.to_csv('GPSHeadingInfoShop.txt', header=None, index=None, sep=' ')
    OrientationShop.to_csv('OrientationShop.txt', header=None, index=None, sep=' ')


if __name__ == '__main__':
    try:
        fetcher()
    except rospy.ROSInterruptException:
        pass
