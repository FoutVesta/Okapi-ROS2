#!/usr/bin/python3
'''
*File name: rfidbot_aucc_nav_console.py
*Description: The control console of the autonomous cycle counting navigation
*Author: Jian zhang
*Create date: May/17/2015
'''

import rospy
from std_msgs.msg import Int16,Bool,String,Int32


import sys, select, termios, tty

msg = """
console of the autonomous cycle counting navigaton:
d: delete pos and paulse reader 
r: resume reader
l: set power 10
h: set power 200
a: all antennae
o: only antenna 1
f: set filter "0000"

CTRL-C to quit
"""

STARTING_CYC_COUNTING = 1
BACK_TO_INIT_POINT = 1
STARTING_CYC_MINIC = 1
DEL_A_POINT = 1
SAVE_2_FILE = 1
READ_FROM_FILE = 1
GENERATE_PATH = 1
SET_EREA = 1
    
class rfibotTagsReaderTest:
    def __init__(self):
        rospy.init_node("rfidbot_open_damo")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)

        
        self.rate = 50

        self.pause_Inventory_del_pos_pub = rospy.Publisher("/pause_inventory_delete_pos",
                                                    Bool,queue_size=10)
        self.set_new_filter_2_reader_pub = rospy.Publisher("/set_tag_filter_to_reader", 
                                                    String,queue_size=10)
        self.resume_inventory_pub = rospy.Publisher('/resume_inventory_enable_new_pos',
                                                    Bool,queue_size=10)
        self.set_tx_power_2_reader_pub = rospy.Publisher('/set_tx_power_to_reader',
                                                    Int16,queue_size=10)
        self.set_new_antennas_2_reader_pub = rospy.Publisher('/set_new_antennas',
                                                    Int32,queue_size=10)
                                                    

        #key input init
        self.settings = termios.tcgetattr(sys.stdin)
        

    def setAntennaBits(self,antennas):
        '''
        * description: set antenna bits for antenna config, 
                        every bit of the config parameter is the switch for a antenna, 
                        antennaBits[0]=1 for antenna 1 on, antennaBits[0]=0 off antenna 1, 
                        antennaBits = 0 enable all antennae
        * input: antennas, active antnnas in [1] or [1,2,4], if 0 enable all antennae
        * output: return antennaBits. 
        ''' 
        antennaBits = 0  
        if antennas == 0:  #enable all antennae
            return antennaBits 
        for anId in antennas:
            #BitMask = 1
            BitMask = 1 << (anId -1)
            antennaBits = antennaBits | BitMask
        rospy.logwarn("the antenna bits %d",antennaBits)
        return  antennaBits   
        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key 
        
    def spin(self):
        r = rospy.Rate(self.rate)
        
        rospy.logwarn( msg)
        while not rospy.is_shutdown():
            key = self.getKey()
            
            if key == 'd':
                rospy.logwarn('delete pos and stop the reader')
                self.pause_Inventory_del_pos_pub.publish(True)
                   
            elif key == 'r':
                rospy.logwarn ('resume the reader')
                self.resume_inventory_pub.publish(True)
            elif key == 'l': 
                rospy.logwarn ('set power to 10')
                self.set_tx_power_2_reader_pub.publish(10)
            elif key == 'h':
                rospy.logwarn ('set power to 200')
                self.set_tx_power_2_reader_pub.publish(200)
            elif key == 'a':
                rospy.logwarn ('set all antenna')
                self.set_new_antennas_2_reader_pub.publish(0)
            elif key == 'o':
                rospy.logwarn ('set antenna 1')
                self.set_new_antennas_2_reader_pub.publish(1)
            elif key == 'f':   
                rospy.logwarn ('set filter 0000')
                self.set_new_filter_2_reader_pub.publish('0000')
            elif key == '0':
                rospy.logwarn ('enable all antennae')
                self.set_new_antennas_2_reader_pub.publish(0)
            elif key == '1':
                rospy.logwarn ('enable antenna 1')
                bits = self.setAntennaBits([1])
                self.set_new_antennas_2_reader_pub.publish(bits)
            elif key == '2':
                rospy.logwarn ('enable antenna 1,3')
                bits = self.setAntennaBits([1,3])
                self.set_new_antennas_2_reader_pub.publish(bits)
                    
            elif key == '\x03':
                rospy.logwarn ('exiting')
                break
            r.sleep
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
                
if __name__ == '__main__':
    """ main """
    console = rfibotTagsReaderTest()
    console.spin()