#!/usr/bin/env python3
'''
*File name: rfh_system_monitor.py
*Description: The system monitor of rfusion handheld
              1. if the pose (odom) of kinect is lost sound alarm
*Author: Jian zhang -- test--new v1.0
*Create date: Oct/31/2016
*Note: in order to make alarm sounded, need intall sox by "sudo apt-get install sox"
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,String
from nav_msgs.msg import Odometry

import os
import time

FLOAT_ZERO = 0.00000001
BAD_POSITION = 9999.9

SYS_INITIAL = 0
SYS_NORMAL = 1
SYS_POSE_LOST = -1

ALM_ON = 1
ALM_OFF = -1

    
class rfhSysMonitor(Node):
    def __init__(self):
        super().__init__("rfh_sys_monitor")
        self.nodename = self.get_name()
        self.get_logger().info("%s started" % self.nodename)
        self.rate = 5
        self.r = 1.0 / float(self.rate)
        self.sysState = SYS_INITIAL
        self.almSwitch = ALM_ON
        self.set_alarm_switch_sub = self.create_subscription(
            Bool,
            "/set_alarm_switch",
            self.almSwitchCallback,
            10
        )
        self.initial_pose_done_sub = self.create_subscription(
            Bool,
            "/initial_pose_done",
            self.initialPoseDoneCallback,
            10
        )
        self.pose_lost_sub = self.create_subscription(
            Bool,
            "/pose_lost",
            self.poseLostCallBack,
            10
        )
              


    def poseLostCallBack(self,msg):
        if msg.data:
            self.sysState = SYS_POSE_LOST
        
    def almSwitchCallback(self,msg):
        if msg.data:
            self.almSwitch = ALM_ON
        else:
            self.almSwitch = ALM_OFF

    def initialPoseDoneCallback(self,msg):
        if msg.data:
            #rospy.logwarn("the initial pose is done")
            os.system('play --no-show-progress --null --channels 1 synth %s sine %f' % (0.5, 500)) 
            self.sysState = SYS_NORMAL



        
    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if SYS_POSE_LOST == self.sysState and ALM_ON == self.almSwitch:
                self.soundAlarmOnce()
                
            time.sleep(self.r)

            
    def soundAlarmOnce(self):
        os.system('play --no-show-progress --null --channels 1 synth %s sine %f' % (0.2, 800))

                
if __name__ == '__main__':
    """ main """
    rclpy.init(args=None)
    sysMonitor = rfhSysMonitor()
    try:
        sysMonitor.spin()
    finally:
        sysMonitor.destroy_node()
        rclpy.shutdown()

