#!/usr/bin/python3
'''
*File name: rfidbot_tags_reader
_api.py
*Description: to test the control of the system
*Author: Jian zhang
*Create date: June/26/2015
'''

import rospy
from rfidbot_tags_reader.msg import tagReader

from std_msgs.msg import Int16,Bool,String
from std_msgs.msg import Float32

from numpy import array
from math import fabs

from twisted.internet import reactor, defer
import llrp
#from llrp import LLRPClient
from llrp_proto import Modulation_Name2Type, DEFAULT_MODULATION, \
     Modulation_DefaultTari

import pprint
import datetime

import array
import struct
     

'''
class Name rfidbotTagsReader
'''

FILTEREPC= '303404D4C80FF56D07A4B412'
class rfidbotTagsReaderApi():

    def __init__(self):

        
        rospy.init_node("rfid_tags_reader_api") #,log_level=rospy.INFO)
        self.nodename = rospy.get_name()
        rospy.logwarn("%s,%s started" ,"rfid_tags_reader_api", self.nodename)
        self.rate = 1
        
        self.pause_inventory_pub = rospy.Publisher('/resume_inventory_enable_tag_filter', Bool,queue_size=10)
        self.set_tagfilter_pub = rospy.Publisher('/set_tag_filter_pause_inventory', String,queue_size=10)
 
        
    def spin(self):
        rospy.logwarn('send pause_inventory')
        r = rospy.Rate(self.rate)
        #for i in range(0,5):
        while not rospy.is_shutdown():
            state = input('1 set Epc filter, 0 send None as filter,2 pause_inventory:')
            if state == 0:
               self.set_tagfilter_pub.publish('')
            elif state == 1:
               self.set_tagfilter_pub.publish(FILTEREPC)
            elif state ==2:
                self.pause_inventory_pub.publish(True)
            r.sleep()
            
            
if __name__ == '__main__':
    """ main """
    DiffCtrl = rfidbotTagsReaderApi()
    DiffCtrl.spin()
        
