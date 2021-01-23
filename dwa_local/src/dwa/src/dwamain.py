#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import dwa
class DWAPlanner():
    def __init__(self,vehicleParam):
        self.sensorAngleMax = vehicleParam['sensorAngleMax']   ##激光传感器的最大测量角度
        self.sensorAngleMin = vehicleParam['sensorAngleMin']   ##激光传感器的最小测量角度
        self.sensorAngleReso = vehicleParam['sensorAngleReso'] ##激光传感器的角分辨率
        self.followAngleS = vehicleParam['followAngleS']       ##提取的有效数据开始角度
        self.followAngleE = vehicleParam['followAngleE']       ##提取的有效数据结束角度
        self.adjacentRange = vehicleParam['adjacentRange']     ##进行计算的传感器探测距离最大值
        self.goal_topic = rospy.get_param('goal_topic')
        initmsg = rospy.wait_for_message(self.goal_topic, PoseStamped)
        self.goal = [initmsg.pose.position.x, initmsg.pose.position.y]

    ##获取激光雷达数据，并将激光雷达数据转化成移动机器人坐标系的坐标。
    #移动机器人坐标系：以移动机器人的前进方向为x轴，垂直于x轴向左为y轴
    def getScan(self):
        temp = []
        self.sensorPoint = []
        ##订阅话题
        scan_topic = rospy.get_param('scan_topic')
        msg = rospy.wait_for_message(scan_topic, LaserScan)
        for i in range(len(msg.ranges)):
            tmp = self.sensorAngleMin + i * self.sensorAngleReso
            ##选取有效角度范围内的激光雷达数据
            if tmp >= self.followAngleS and tmp <= self.followAngleE and msg.ranges[i] <= self.adjacentRange:
                if msg.ranges[i] == float('inf') :
                    continue
                else:
                    ##距离转化成坐标
                    x = msg.ranges[i] * math.cos(tmp)
                    y = msg.ranges[i] * math.sin(tmp)
                    self.sensorPoint.append([x,y])
    
    def getGoal(self): 
        msg = rospy.wait_for_message(self.goal_topic , PoseStamped)
        self.goal = [msg.pose.position.x,msg.pose.position.y]  
    ##获取移动机器人角速度和线速度
    
    def getOdom(self):
        ##订阅话题
        odom_topic = rospy.get_param('odom_topic')
        msgodom = rospy.wait_for_message(odom_topic, Odometry)
        x = msgodom.pose.pose.orientation.x
        y = msgodom.pose.pose.orientation.y
        z = msgodom.pose.pose.orientation.z
        w = msgodom.pose.pose.orientation.w
        self.vehicleX = msgodom.pose.pose.position.x
        self.vehicleY = msgodom.pose.pose.position.y
        self.vehicleYaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        self.linearSpeed = msgodom.twist.twist.linear.x
        self.angularSpeed = msgodom.twist.twist.angular.z

    def localGoal(self):
        relativeX =  (self.goal[0] - self.vehicleX) * math.cos(self.vehicleYaw) + (self.goal[1] - self.vehicleY) * math.sin(self.vehicleYaw)
        relativeY = -(self.goal[0] - self.vehicleX) * math.sin(self.vehicleYaw) + (self.goal[1] - self.vehicleY) * math.cos(self.vehicleYaw)
        return [relativeX, relativeY]

def main():
    ##初始化节点
    rospy.init_node('dwaplanner')
    ##读取launch文件的参数
    sensorAngleMax = rospy.get_param("sensorAngleMax")
    sensorAngleMin = rospy.get_param("sensorAngleMin")
    sensorAngleReso = rospy.get_param("sensorAngleReso")
    followAngleS = rospy.get_param("followAngleS")
    followAngleE = rospy.get_param("followAngleE")
    maxAcc = rospy.get_param("maxAcc")
    adjacentRange = rospy.get_param("adjacentRange")
    maxLinearSpeed = rospy.get_param("maxLinearSpeed")
    minLinearSpeed = rospy.get_param("minLinearSpeed")
    maxAngularSpeed = rospy.get_param("maxAngularSpeed")
    maxAngularAcc = rospy.get_param("maxAngularAcc")
    dt = rospy.get_param("dt")
    speedReso = rospy.get_param("speedReso")
    angularReso = rospy.get_param("angularReso")
    predictTime = rospy.get_param("predictTime")
    headWeight = rospy.get_param("headWeight")
    speedWeight = rospy.get_param("speedWeight")
    odistWeight = rospy.get_param("odistWeight")
    gdistWeight = rospy.get_param("gdistWeight")
    arrivedist = rospy.get_param("arrivedist")

    cml_topic = rospy.get_param("cmd_topic")
    cmd_pub = rospy.Publisher(cml_topic, Twist, queue_size=2) ##控制命令的发布话题

    ##将参数放置到字典中
    vehicleParam = {'maxSpeed':maxLinearSpeed,'minSpeed':minLinearSpeed,'maxAcc':maxAcc,'maxAngularSpeed': maxAngularSpeed,'maxAngularAcc':maxAngularAcc,'dt':dt,'sensorAngleMax':sensorAngleMax,'sensorAngleMin':sensorAngleMin,'sensorAngleReso':sensorAngleReso,
                    'followAngleS':followAngleS,'followAngleE':followAngleE,'adjacentRange':adjacentRange}

    dwaParam = {'speedReso':speedReso,'angularReso':angularReso,'predictTime':predictTime,'headWeight':headWeight,'speedWeight':speedWeight,'odistWeight':odistWeight,'gdistWeight':gdistWeight,'arrivedist':arrivedist}

    ##类参数初始化
    dwaplanner = DWAPlanner(vehicleParam)
    dwaFunc = dwa.DWA(vehicleParam,dwaParam)
    angularSpeed = 0
    while not rospy.is_shutdown():        
        dwaplanner.getScan()
        dwaplanner.getOdom()
        localgoal = dwaplanner.localGoal()
        if math.hypot(localgoal[0], localgoal[1]) < dwaParam['arrivedist']:
            dwaplanner.getGoal()
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            cmd_pub.publish(twist)
        else:
            linearSpeed,angularSpeed = dwaFunc.main(dwaplanner.sensorPoint,localgoal,dwaplanner.linearSpeed,angularSpeed)
            twist = Twist()
            twist.linear.x = linearSpeed
            twist.angular.z = angularSpeed
            cmd_pub.publish(twist)
if __name__ == '__main__':
    main()

