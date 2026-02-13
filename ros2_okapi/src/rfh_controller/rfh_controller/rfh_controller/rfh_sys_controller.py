#!/usr/bin/env python3
'''
 * File Name: rfh_sys_controller.py
 *
 * Description: This is the pototype of the system controller
 *
 * Author: Jian Zhang
 *
 * Create Date: 2019-1-25
 *
 * Version: 1.0
 *
 * Remark: for the idea of start this script in the startup, then use it as a sever, to control the
        okapi. This is not reliable, sometimes when ubuntu start the rfh_sys_ctrler may not start
        correctly, and when execute the kill to kill ros node, the sever can not start the okapi.

       Therefore, we need a beter way for a system that onboard system is in a separate computer
       from the workstation, that can work more reliable in the backend to wait the command. Maybe
       apache?

       However, the idea of use python base ros node to controll the start process is working good,
       as long as this node is start correctly, it can control the process.
'''

import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class okpSystemCtrler(Node):
    def __init__(self):
        super().__init__("Okapi_System_controller")
        self.nodename = self.get_name()
        self.get_logger().info("%s started" % self.nodename)
        self.r = 1.0  # 1 Hz loop rate (seconds per loop)

        self.isStarting = False
        self.start_okapi_sub = self.create_subscription(
            Bool,
            'start_okapi',
            self.startOkapiCallBack,
            10
        )

    def startOkapiCallBack(self,msg):
        self.isStarting = msg.data
        os.system("roslaunch rviz_view rviz_view_navigation.launch &")

    def spin(self):
        while rclpy.ok():
            # process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.isStarting:
                os.system("/home/rfid/okapi_ws/okictrlscript/run_handheld_localizer &")
                time.sleep(self.r)
                #we may add some function to check the mode is starting correctly
                os.system("/home/rfid/okapi_ws/okictrlscript/run_tag_localizer &")
                time.sleep(self.r)

                time.sleep(self.r)
                self.isStarting = False

            time.sleep(self.r)



if __name__ == '__main__':
    """ main """
    rclpy.init(args=None)
    console = okpSystemCtrler()
    try:
        console.spin()
    finally:
        console.destroy_node()
        rclpy.shutdown()

