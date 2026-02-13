#!/usr/bin/python3
'''
*File name: rfh_controller_console.py
*Description: The control console of rfusion handhled
*Author: Jian zhang
*Create date: Aug/11/2016
'''

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Bool,String,Int16
from rfidbot_tags_interfaces.srv import TagLocalizing 
from visualization_msgs.msg import Marker

#for loalizing flag seting
LOC_FLAG_NO_RAW_DATA =0   # no raw data, 
LOC_FLAG_ONLY_INV = 1     # only inventory, but can not localize 
LOC_FLAG_LOCALIZED = 2    # localized
RFD8500_START_INV = 1
RFD8500_STOP_INV = -1
RFD8500_STOP_EXIT = 0

msg = """
console of the rfusion handheld:
sr: save the rawdata of handheld
rr: read the rawdata of handheld
lat: localize a tag
relat: retrieve all estimated results of a tag
latall: localize all tag
settx: set reader power
enalm: enable alarm
disalm: disable alarm
exit: exit the cli
startinv: start inventory, this is only working for RFD8500
stopinv: stop inventory,this is only working for RFD8500
setpower: stop inventory power,this is only working for RFD850
showmesh: show the mesh from zed spatial mapping
start: this is the test for okapi system controller
"""


    
class rfhCtrlConsole(Node):
    def __init__(self):
        super().__init__("rfidbot_open_damo")
        self.nodename = self.get_name()
        self.get_logger().info("%s started" % self.nodename)
        self.rate = 50
        self.r = 1.0 / self.rate  # loop period in seconds
        

        self.saverawdata_pub = self.create_publisher(Bool, "/save_raw_data", 1)  
        self.readrawdata_pub = self.create_publisher(String, "/read_raw_data", 1)
        self.localize_all_tag_pub = self.create_publisher(Bool, "/loc_all_tag", 1)
        self.set_tx_pub = self.create_publisher(Bool, "/set_tx", 1)
        self.epccenter_pub = self.create_publisher(Marker, 'epc_center', 10)
        self.alarm_switch_pub = self.create_publisher(Bool, "/set_alarm_switch", 1)
        self.RFD8500_pub = self.create_publisher(Int16, "/set_rfd8500", 1)
        self.RFD8500_power_pub = self.create_publisher(Int16, "/set_rfd8500_power", 1)
        self.retrieve_EstPos_pub = self.create_publisher(String, "/retrieve_est_pos_4_a_tag", 1)

        #jian add for test the test send the mesh obj
        self.mesh_pub = self.create_publisher(Marker, 'mesh_test', 10)

        self.start_pub = self.create_publisher(Bool, 'start_okapi', 10)

        # service client for tag localizing
        self.tag_localizer_client = self.create_client(TagLocalizing, 'localize_a_tag')
        while not self.tag_localizer_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service localize_a_tag not available, waiting...')

    def showMeshMarker(self):
        marker = Marker()
        marker.header.frame_id = "/odom"
        marker.type = marker.MESH_RESOURCE
#        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.mesh_resource = "package://rfh_controller/create_2.dae" # mesh_gen.obj"
        self.mesh_pub.publish(marker)
        
        
    def getTagPosition(self,tagEpc):
        try:
            request = tagLocalizing.Request()
            request.tagEpc = tagEpc
            future = self.tag_localizer_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            resp = future.result()
            if int(resp.localizeFlag) == LOC_FLAG_LOCALIZED:
                #rospy.logwarn('the loaction by service %s',resp)
                posX = float('%.2f' % resp.posX)
                posY = float('%.2f' % resp.posY)
                posZ = float('%.2f' % resp.posZ)
                return resp.tagEpc,int(resp.localizeFlag),posX,posY,posZ
            else:
                return resp.tagEpc,int(resp.localizeFlag),None,None,None

        except Exception as e:
            self.get_logger().warn("Service call failed: %s" % e)
            return None,None,None,None,None

    def setaSingleMarker(self,PosX,PosY,PosZ,color,size,frameId):
        marker = Marker()
        marker.header.frame_id = frameId
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = size 
        marker.scale.y = size 
        marker.scale.z = size 
        if color == 'r':
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif color == 'g':
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif color == 'b':
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = PosX
        marker.pose.position.y = PosY
        marker.pose.position.z = PosZ
        
        return marker
        
    def spin(self):
        self.get_logger().warn(msg)
        while rclpy.ok():
            cmd = input("input cli command:  ")
            if 'sr' == cmd:
                self.get_logger().warn('save rawdata to file')
                self.saverawdata_pub.publish(True)
            elif "rr" == cmd:
                self.get_logger().warn('read rawdata to file, please input the file name(no parents folder)')
                filename = input("please input the file name(no parents folder):  ")
                self.readrawdata_pub.publish(filename)
            elif "lat" == cmd:
                self.get_logger().warn('please input the tag epc')
                epc = input("please input the tag epc:  ")
                epc = epc.lower()
                self.get_logger().warn('localize the tag %s',epc)
                (epc,flag,x,y,z) = self.getTagPosition(epc)
                if (flag == LOC_FLAG_LOCALIZED ):
                    self.get_logger().warn("flag %s,(%f,%f,%f)",flag,x,y,z)
                    marker = self.setaSingleMarker(x,y,z,'r',0.30,"/odom")
                    self.epccenter_pub.publish(marker)
                else:
                    self.get_logger().warn("tag %s, can not be localized:%s",epc,flag)         
            elif "relat" == cmd:
                self.get_logger().warn('please input the tag epc')
                epc = input("please input the tag epc:  ")
                epc = epc.lower()
                self.get_logger().warn('retrieve estimated result of the tag %s',epc)
                self.retrieve_EstPos_pub.publish(epc)
            elif "settx" == cmd:
                self.get_logger().warn("set reader power")
                self.set_tx_pub.publish(True)
            elif "latall" == cmd:
                self.get_logger().warn('localize all tags')
                self.localize_all_tag_pub.publish(True)
            elif "enalm" == cmd:
                self.get_logger().warn('enable the alarm for pose lost')
                self.alarm_switch_pub.publish(True)
            elif "disalm" == cmd:
                self.get_logger().warn('disable the alarm for pose lost')
                self.alarm_switch_pub.publish(False)
            elif "startinv" == cmd:
                self.get_logger().warn('start RFD8500 inventory')
                self.RFD8500_pub.publish(Int16(data=RFD8500_START_INV))
            elif "stopinv" == cmd:
                self.get_logger().warn('stop RFD8500 inventory')
                self.RFD8500_pub.publish(Int16(data=RFD8500_STOP_INV))
            elif "setpower" == cmd:
                self.get_logger().warn('please input new power')
                power = input("please input new power(10~240):  ")
                power = int(power)
                self.get_logger().warn('set power to %d',power)
                self.RFD8500_power_pub.publish(Int16(data=power))
            elif "showmesh"== cmd:
                self.showMeshMarker()
            elif "exit" == cmd:
                self.RFD8500_pub.publish(Int16(data=RFD8500_STOP_EXIT))
                self.get_logger().warn("exit the cli")
                break
            elif "start" == cmd:
                self.start_pub.publish(True)
            else:
                self.get_logger().warn("the input cli command is not exist!")
                self.get_logger().warn(msg)
            time.sleep(self.r)

                
if __name__ == '__main__':
    """ main """
    rclpy.init(args=None)
    console = rfhCtrlConsole()
    try:
        console.spin()
    finally:
        console.destroy_node()
        rclpy.shutdown()

