#!/usr/bin/python3
'''
*File name: rfd8500_reader.py
*Description: reader rfid tags from zebera RFID 8500 by ZETI interface
*
*Author: Jian zhang
*Create date: Dec/19/2016
*Modidfy: 
        After updating the firmware of RFD8500 it not working. Then I find the port of the it change from 1 to 9
* note: need install python-bluez: sudo apt-get install python-bluez
'''

from pprint import pprint

import psutil
import serial
import string
import time
import bluetooth

import rclpy
from rclpy.node import Node
import sys,os

from rfidbot_tags_interfaces.msg import TagReader

from std_msgs.msg import Int16
import math

from struct import *

CONNECT_READER ='\n'+'cn'+ '\r\n'+ '\r\n'+ '\r\n'
INV_RFID = 'in' +' '+'.batchmode'+' '+ 'disable'+' '+'.ez .ir .ik .ih '+'.ec '+ '\r\n'
END_INV = 'a' + '\r\n'
GET_BT = 'gd'+' '+ '.bt' + '\r\n'
#GET_GR = 'gr' + '\r\n'
# RFD850015314523021347_ADDR = '00:17:E9:FA:C7:01'
# RFD850015313523020835_ADDR = '00:17:E9:FA:CB:9F'
# RFD850015314523021347_ADDR = '84:C6:92:49:11:A4' # RFD8500 Handheld
RFD850022253520100892_ADDR = '84:C6:92:49:11:A4' 
# RFD850015314523021347_ADDR = '84:C6:92:4B:C7:F8' # Justin's handheld

RFD8500_START_INV = 1
RFD8500_STOP_INV = -1
RFD8500_STOP_EXIT = 0

class tagsRFD8500Reader():
    def __init__(self):
        rclpy.init()
        self.node = Node("rfid_tags_reader_RFD8500")
        self.nodename = self.node.get_name()
        self.node.get_logger().warn(
            "%s, %s started" % ("rfid_tags_reader_RFD8500", self.nodename)
        )

        self.tags_pub = self.node.create_publisher(
            TagReader,
            'rfid_tags',
            10
        )

        self.node.create_subscription(
            Int16,
            '/set_rfd8500',
            self.setRFD8500Callback,
            10
        )

        self.node.create_subscription(
            Int16,
            '/set_rfd8500_power',
            self.setRFD8500PowerCallback,
            10
        )

        #global 
        self.exitflag = False
        self.isInvStart = False
 
        #get default param
        self.defaultSession = int(self.node.declare_parameter('session',0).value)
        self.defaultTxPower = int(self.node.declare_parameter('txpower',130).value)
        self.node.get_logger().warn(
            "default session %d, txpower %d" %        (self.defaultSession,self.defaultTxPower)
        )

        #the buletooth socket object
        self.gaugeSocket = None;
        '''nearby_devices = bluetooth.discover_devices()
        rospy.logwarn("nearby_devices:%s",nearby_devices)
        service = bluetooth.find_service(address=RFD850015314523021347_ADDR)  #use this to check the service port
        pprint(service)
        time.sleep(1)
        self.blueZConnect()
        cmd = input("Enter command:")
        #cmd+='\r\n'
        print(cmd)
        self.gaugeSocket.send(cmd)
        print(self.gaugeSocket.recv(2))'''

        time.sleep(1)
        self.blueZConnect()
        #time.sleep(10)
        self.readerConnect()
        time.sleep(1)
        self.getBattery()
        time.sleep(1)
        self.setInvSession(self.defaultSession)
        #time.sleep(1)
        self.setBeeperVolume(3)
        time.sleep(1)
        self.curTxPower = self.defaultTxPower #record current TX power
        self.startInventory(self.defaultTxPower)


    def setRFD8500Callback(self,msg):   
        cmd = msg.data
        self.node.get_logger().warn('receive cmd %d' % cmd)
        
        if RFD8500_START_INV == cmd:
            self.startInventory()
        elif RFD8500_STOP_INV == cmd:
            self.stopInventory()
        elif RFD8500_STOP_EXIT == cmd:
            self.exitflag = True
            self.blueZdisconnect()
        
    def setRFD8500PowerCallback(self,msg):
        power  = msg.data
        self.node.get_logger().warn('receive power %d' % power)
        if power<=10:
            self.node.get_logger().warn("set power failed!the power is invalid!")
            return
        if self.isInvStart:
            self.stopInventory()
        time.sleep(1)
        self.curTxPower = power
        self.startInventory(power)

    def getBattery(self):
        '''
        * Description: get the RFD8500 handheld power
        * input: none
        * ouput: print the battery information
        '''
        self.node.get_logger().warn("getBattery")
        if self.gaugeSocket == None:
            self.node.get_logger().warn("get battery info failed! the gaugeSocket is  None")
            return
        
        self.gaugeSocket.send(GET_BT)
        time.sleep(1)
        rec = self.gaugeSocket.recv(1024)

        self.node.get_logger().warn("get the battery infor:%s" % rec)

        
    def parseCommandFeedBack(self,recv):
        '''
        * Description: parse the command feed back, most of command feedbacks are:
        *              Command:xxx, Status:xxx
        *              this function onle parse the "Command" and "Status", ignore others
        * input: recv msg
        * ouput: return None, if the recv is not command feed back
                 return dict{"Command":xx, "status":xx}, 
        '''
        recv_str = recv.decode('utf-8')  # Decode bytes to a string
        if "Command" not in recv_str:
            return None
            
        cmdFB= []
        Recordlist = recv_str.split("\r\n")
        for item in Recordlist:
            if "Command" not in item:
                continue
            Cmdlist = item.split(",")
            CmdNameList = Cmdlist[0].split(":")
            statusList = Cmdlist[1].split(":")
            cmdFB.append(dict([('Command',CmdNameList[1]),('Status',statusList[1])]))
        return cmdFB
        
    def blueZConnect(self):
        '''
        * Description: connect the reader by buletooth 
        * input: none
        * ouput: put socket objec to elf.gaugeSocket
        '''
        while(True):  
            self.node.get_logger().warn("please push the trigger of RFD8500")
            try:
                gaugeSocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                gaugeSocket.connect((RFD850022253520100892_ADDR,11)) #RFD850015314523021347_ADDR, 1))
                
                ''' while True:
                    rec = gaugeSocket.recv(1024)
                    if self.IsBluzConnected(rec):
                        break'''
                break;
            except bluetooth.btcommon.BluetoothError as error:
                gaugeSocket.close()
                print("Could not connect: ", error, "; Retrying in 10s...")
                time.sleep(10)
        self.gaugeSocket = gaugeSocket
        self.node.get_logger().warn("connect sucess")

    def blueZdisconnect(self):
        '''
        * Description: disconnect the reader 
        * input: none
        * ouput: none
        '''
        if self.gaugeSocket == None:
            self.node.get_logger().warn("blueZdisconnect failed! the gaugeSocket is None")
            return
        self.stopInventory()
        self.gaugeSocket.close()
        
    def IsConnected(self,rec):
        '''
        * Description: after send 'cn' command to RFD8500, check the feedback to make sure
        *              the bule tooth connection is created
        * input: recv msg
        * ouput: return Ture is connected, otherwise return False
        '''
        self.node.get_logger().warn('%s' % rec)
        cmdFb = self.parseCommandFeedBack(rec)
        if cmdFb == None:
            return False
        self.node.get_logger().warn('%s' % cmdFb)
        for item in cmdFb:
            if 'connect' in item['Command'] and "Connection Successful" in item['Status']:
                return True
        return False
        
    def readerConnect(self):
        '''
        * Description: connect the reader by send command 'cn'
        * input: none
        * ouput: put socket objec to elf.gaugeSocket
        '''
        if self.gaugeSocket == None:
            self.node.get_logger().warn("readerConnect failed! the gaugeSocket is None")
            return
        try:
            
            cmd = CONNECT_READER
            self.gaugeSocket.send(cmd)
            print(cmd)
            while True:
                rec = self.gaugeSocket.recv(1024)
                if self.IsConnected(rec):
                    return          
        except bluetooth.btcommon.BluetoothError as error:
            self.node.get_logger().warn("Caught BluetoothError: %s" % error)
            time.sleep(5)
            pass
            
    def IsSessionIDSet(self,rec):
        '''
        * Description: check the set session is scusess
        * input: recv msg
        * ouput: return Ture is connected, otherwise return False
        '''
        self.node.get_logger().warn('IsSessionIDSet %s' % rec)
        cmdFb = self.parseCommandFeedBack(rec)
        if cmdFb == None:
            return False
        self.node.get_logger().warn('IsSessionIDSet:%s' % cmdFb)
        for item in cmdFb:
            if 'setqueryparams' in item['Command'] and "OK" in item['Status']:
                return True
        return False
        
    def setInvSession(self,sessionID):
        '''
        * Description: set inventory session
        * input: sessionID
        * ouput: None
        '''
        self.node.get_logger().warn("setInvSession")
        if self.gaugeSocket == None:
            self.node.get_logger().warn("setInvSession failed! the gaugeSocket is None")
            return
        cmd = 'qp' + ' '+ '.i'+' '+ str(sessionID)+'\r\n'
        self.node.get_logger().warn("cmd:%s" % cmd)
        try:
            self.gaugeSocket.send(cmd)
            while True:
                rec = self.gaugeSocket.recv(1024)
                if self.IsSessionIDSet(rec):
                    break
        except bluetooth.btcommon.BluetoothError as error:
            self.node.get_logger().warn("Caught BluetoothError: %s" % error)
            time.sleep(5)
            pass

    def isInvStarted(self):
        '''
        * Description: check the start inventory is success
        * input: recv msg
        * ouput: return True if invent start, otherwise return False
        '''
        rec = self.gaugeSocket.recv(1024)
        cmdFb = self.parseCommandFeedBack(rec)
        if cmdFb == None:
            return False
        self.node.get_logger().warn('isInvStart:%s' % cmdFb)
        for item in cmdFb:
            if 'inventory' in item['Command'] :#and "OK" in item['Status']:
                self.node.get_logger().warn('isInvStart:OK')
                return True
        return False
        
    def startInventory(self,power=None):
        '''
        * Description: start inventory
        * input: power, the inventory power
        *               default power is None, then use default setting of RFD8500
        * ouput: send inventory as ROS msg
        '''
        if self.gaugeSocket == None:
            self.node.get_logger().warn("startInventory failed! the gaugeSocket is None")
            return
        if power == None:
            cmd = INV_RFID
        else:
            cmd = 'inventory'+' '+'.batchmode'+' '+ 'disable'+' '+'.ez .ir .ik .ih '+'.ec '+'.power'+' '+ str(power)+'\r\n'
            self.node.get_logger().warn("new command %s" % cmd)
        self.isInvStart = False # to make sure we can got feedback in isInvStart() 
        try:
            self.node.get_logger().warn("start inv new command %s" % cmd)
            self.gaugeSocket.send(cmd)
            while True:
                if self.isInvStarted():
                    break
            self.isInvStart = True
            self.node.get_logger().warn("start inv new command success")
        except bluetooth.btcommon.BluetoothError as error:
            print("Caught BluetoothError: ", error)
            time.sleep(5)
            pass
            
    def isInvStoped(self):
        '''
        * Description: check the stop inventory is success
        * input: recv msg
        * ouput: return True if inventory stop, otherwise return False
        '''
        rec = self.gaugeSocket.recv(1024)
        self.node.get_logger().warn('isInvStoped %s' % rec)
        cmdFb = self.parseCommandFeedBack(rec)
        if cmdFb == None:
            return False
        self.node.get_logger().warn('isInvStoped:%s' % cmdFb)
        for item in cmdFb:
            if 'abort' in item['Command'] and ( "OK" in item['Status'] or "No Radio Operation in Progress" in item['Status']):
                return True
        return False
        
    def stopInventory(self):
        '''
        * Description: stop inventory
        * input: none
        * ouput: none
        '''
        if self.gaugeSocket == None:
            self.node.get_logger().warn("stopInventory failed! the gaugeSocket is None")
            return
        try:
            self.isInvStart = False
            self.gaugeSocket.send(END_INV)
            while True:
                if self.isInvStoped():
                    break
            self.node.get_logger().warn("set end inventory success")
        except bluetooth.btcommon.BluetoothError as error:
            print("Caught BluetoothError: ", error)
            time.sleep(5)
            pass

    def isBeeperVolumeSeted(self):
        '''
        * Description: check set the volume of beeper is success
        * input: recv msg
        * ouput: return True if success, otherwise return False
        '''
        rec = self.gaugeSocket.recv(1024)
        self.node.get_logger().warn('isBeeperVolumeSeted %s' % rec)
        cmdFb = self.parseCommandFeedBack(rec)
        if cmdFb == None:
            return False
        self.node.get_logger().warn('isBeeperVolumeSeted:%s' % cmdFb)
        for item in cmdFb:
            if 'setattr' in item['Command'] and "OK" in item['Status']:
                return True
        return False
        
    def setBeeperVolume(self,volume):
        '''
        * Description: set the volume of beeper
        * input: volume: 0 is high, 1 is medium, 2 is low
        * ouput: none
        '''
        if self.gaugeSocket == None:
            self.node.get_logger().warn("setBeeperVolume failed! the gaugeSocket is None")
            return
        cmd = "setattr" + " " +".attnum 140" +" "+ ".atttype B"+" " + ".attvalue" + " " +str(volume)+ '\r\n'
        try:
            self.gaugeSocket.send(cmd)
            while True:
                if self.isBeeperVolumeSeted():
                    break
            self.node.get_logger().warn("set Beeper Volume success")
        except bluetooth.btcommon.BluetoothError as error:
            print("Caught BluetoothError: ", error)
            time.sleep(5)
            pass
            
    def parseEPC(self,recv):
        '''
        * Description: parse the EPC from the RFD8500 recv
        *               format of a report: na,na,'EPC','pc','RSSI','phase','channel'   
        * input: recv: RFD8500 receive massage
        * ouput: EPC if is EPC, None is not EPC
        '''

        invRes = []
        recv_str = recv.decode('utf-8')  # Decode bytes to a string
        Recordlist = recv_str.split("\r\n")
        #rospy.logwarn("Recordlist: %s",Recordlist)
        for item in Recordlist:
            EPClist = item.split(",")
            #rospy.logwarn("EPClist:%s",EPClist)
            if len(EPClist) > 6:
                if len(EPClist[2]) == 24:
                    invRes.append(dict([('EPC',EPClist[2]),('RSSI',EPClist[3]),('phase',EPClist[4]),('channelId',EPClist[5])]))
        #rospy.logwarn("%s",invRes)
            
        return invRes
    def invReport(self):
        '''
        * Description: reprot the inventory by massge
        * input: None
        * ouput: send inventory as ROS msg
        '''
        
        if self.isInvStart:
            org_report = self.gaugeSocket.recv(1024)
            invlist = self.parseEPC(org_report)
            for item in invlist:
                #rospy.logwarn("%s",item)
                tagmsg = TagReader()
                tagmsg.epc = item.get('EPC', '')
                tagmsg.peak_rssi = str(item.get('RSSI', ''))
                tagmsg.antenna_id = 'RFD8500_1' #all set to antenna RFD8500_1
                tagmsg.phase = str(-1 * float(item.get('phase', 0.0)) / 180.0 * math.pi)  # report in radians
                                                    #for RFD8500 the phase follows: -1* (2pi(2R/lamda)+thea_niose )mod 2pi
                                                    #therefore, we need correct it by multiple -1
                # tagmsg.ChannelID =str(item['channelId']) # wtf? This isnt even there
                tagmsg.channel = str(item.get('channelId', ''))
                #tagmsg.powerLevel = self.curTxPower 
                now = self.node.get_clock().now().to_msg()
                tagmsg.header.stamp = now
                
                self.tags_pub.publish(tagmsg)
                #if str(item['channelId']) == str(20) or str(item['channelId']) == str(15) :
                #    rospy.logwarn("org:%s",org_report)
                
    def run(self):
        '''
        * Description: continiously reading the RFID feedback
        * input: none
        * ouput: send inventory as ROS msg
        '''
        while (not self.exitflag):
            if self.gaugeSocket == None:
                continue
            self.invReport()
            '''invlist = self.parseEPC(self.gaugeSocket.recv(1024))
            for item in invlist:
                tagmsg = tagReader()
                tagmsg.EPC = item['EPC']
                tagmsg.PeakRSSI = str(item['RSSI'])
                tagmsg.AntennaID = '1' #all set to antenna 1
                now = rospy.Time.now()
                tagmsg.header.stamp = now
                self.tags_pub.publish(tagmsg)'''


if __name__ == '__main__':
    """ main """

    RFD8500= tagsRFD8500Reader()
    RFD8500.run()

'''
rospy.init_node("rfid_tags_reader") #,log_level=rospy.INFO)
nodename = rospy.get_name()
rospy.logwarn("%s,%s started" ,"rfid_tags_reader_RFD8500", nodename)
tags_pub = rospy.Publisher('rfid_tags', tagReader,queue_size=10) 




def connect():
    while(True):    
        try:
            gaugeSocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            gaugeSocket.connect(('00:17:E9:FA:C7:01', 1))
            break;
        except bluetooth.btcommon.BluetoothError as error:
            gaugeSocket.close()
            print "Could not connect: ", error, "; Retrying in 10s..."
            time.sleep(10)
    return gaugeSocket;

gaugeSocket = connect()

try:
    gaugeSocket.send(CONNECT_READER)
    print gaugeSocket.recv(1024)
except bluetooth.btcommon.BluetoothError as error:
    print "Caught BluetoothError: ", error
    time.sleep(5)
    gaugeSocket = connect()
    pass
try:
    gaugeSocket.send(INV_RFID)
    for i in range(1000):
        tagmsg = tagReader()
        tagmsg.EPC = gaugeSocket.recv(1024)
        tags_pub.publish(tagmsg)
        print gaugeSocket.recv(1024)
    gaugeSocket.send(END_INV)
except bluetooth.btcommon.BluetoothError as error:
    print "Caught BluetoothError: ", error
    time.sleep(5)
    gaugeSocket = connect()
    pass
    
try:
    gaugeSocket.send(END_INV)
    print gaugeSocket.recv(1024)
except bluetooth.btcommon.BluetoothError as error:
    print "Caught BluetoothError: ", error
    time.sleep(5)
    gaugeSocket = connect()
    pass

gaugeSocket.close() '''
