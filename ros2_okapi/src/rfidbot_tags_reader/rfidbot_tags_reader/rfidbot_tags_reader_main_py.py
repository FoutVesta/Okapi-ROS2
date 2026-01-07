#!/usr/bin/python3
'''
*File name: rfidbot_tags_reader_main.py
*Description: reader rfid tags by LLRP, and send the tags by topic "rfid_tags"
*             it can not kill by ctrl+c,must use /homne/rfid/jiannyctrlscripts/killpython to kill.
*             or you can kill all python process to kill it
*Author: Jian zhang
*Create date: June/26/2015
'''

import rclpy
from rclpy.node import Node
from rfidbot_tags_interfaces.msg import TagReader

from std_msgs.msg import Int16,Bool,String
from std_msgs.msg import Float32

from numpy import array
from math import fabs

from twisted.internet import reactor, defer
from rfidbot_tags_reader import llrp
from rfidbot_tags_reader.llrp_proto import Modulation_Name2Type, DEFAULT_MODULATION, \
     Modulation_DefaultTari

import pprint
import datetime

import array
import struct
     

'''
class Name rfidbotTagsReader
'''

FILTEREPC= '303404D4C80FF56D07A4B412'    

class rfidbotTagsReader(Node):

    def __init__(self):

        rclpy.init()
        super().__init__("rfid_tags_reader") 
        self.nodename = self.get_name()
        self.get_logger().warn(
            f"rfid_tags_reader, {self.nodename} started"
        )
        self.rate = 1

        #jian add for filter setting
        self.tagFilter = ''
        self.isPaused = False
              
        self.create_subscription(Bool, '/pause_inventory_delete_pos', self.pauseInventoryAndDelPosCallback, 10)
        self.create_subscription(Bool, '/resume_inventory_enable_new_pos', self.resumeEnableNewPosCallback, 10)
        self.create_subscription(String, '/set_tag_filter_to_reader', self.setTagFilteCallback, 10)
        self.create_subscription(Int16, '/set_tx_power_to_reader', self.setTxPowerCallback, 10)
        self.create_subscription(String, '/set_new_antennas', self.setActiveAntennasCallback, 10)

        self.tags_pub = self.create_publisher(TagReader, 'rfid_tags', 10) 
        
        self.tagsNum = 0
        
        #initialize the LLRP
        self.get_logger().info('initial LLRP client')
        self.antennas = (1,2,3,4)
        self.host = self.declare_parameter('host', '192.168.115.100').value
        self.port = 5084
        
        d = defer.Deferred()
        d.addCallback(self.finish)
        
        self.session = self.declare_parameter('session', 1).value
        self.time = 0 
        self.tx_power = self.declare_parameter('txpower', 200).value
        self.modulation = 'M4'
        self.tari = 0
        self.population = 1 

        fac = llrp.LLRPClientFactory(onFinish=d,
            duration=self.time,
            report_every_n_tags= 1,
            antennas=self.antennas,
            tx_power=self.tx_power,
            modulation=self.modulation,
            tari=self.tari,
            session=self.session,
            tag_population=self.population,
            start_inventory=True,
            disconnect_when_done=(self.time > 0),
            reconnect= False,
            tag_content_selector={
                'EnableROSpecID': True, 
                'EnableSpecIndex': False,
                'EnableInventoryParameterSpecID': False,
                'EnableAntennaID': True,
                'EnableChannelIndex': True,
                'EnablePeakRRSI': True,
                'EnableFirstSeenTimestamp': True, 
                'EnableLastSeenTimestamp': True,
                'EnableTagSeenCount': True,
                'EnableAccessSpecID': True 
            })
        
        fac.addTagReportCallback(self.tagReportCallback)

        reactor.connectTCP(self.host, self.port, fac, timeout=3)
        self.get_logger().info('the host is %s' % self.host)

        reactor.addSystemEventTrigger('before', 'shutdown', self.politeShutdown, fac)

        self.factory = fac
        self.factory.setTagFilter(self.tagFilter)

        reactor.run()
        
    def finish (self,_):
        self.get_logger().info('total # of tags seen: %s' % self.tagsNum)
        if reactor.running:
            reactor.stop()
            
    def pauseInventoryAndDelPosCallback(self,msg):
        if msg.data:
            self.isPaused = True
            self.factory.pauseAndDeleteRosPECInventory()

    def setTagFilteCallback(self,msg):
        self.tagFilter=msg.data
        self.factory.setTagFilter(self.tagFilter)
        self.get_logger().warn('get the tag filter %s' % self.tagFilter)

    def resumeEnableNewPosCallback(self,msg):
        if msg.data:
            self.isPaused = False
            self.factory.resumeAndStartNewRosPECInventory()

    def setTxPowerCallback(self,msg):   
        txPower = msg.data
        self.tx_power = txPower
        self.get_logger().warn('receive the new power %s' % self.tx_power)
        self.factory.setNewTxPower(self.tx_power)

    def setActiveAntennasCallback(self,msg):
        newAntennas = msg.data
        self.antennas = newAntennas
        self.factory.setNewActiveAntennas(self.antennas)
        
    def setTagMsg(self, tag):
        tagmsg = TagReader()

        AntennaId = tag['AntennaID'][0]
        tagmsg.epc = tag['EPC-96']

        tagmsg.antenna_id = str(AntennaId)

        fstime = datetime.datetime.fromtimestamp(
            tag['FirstSeenTimestampUTC'][0]/1000000.0).strftime('%Y-%m-%d %H:%M:%S:%f')
        tagmsg.frist_seen_timestamputc = str(fstime)

        lstime = datetime.datetime.fromtimestamp(
            tag['LastSeenTimestampUTC'][0]/1000000.0).strftime('%Y-%m-%d %H:%M:%S:%f')
        tagmsg.last_seen_timestamputc = str(lstime)

        tagmsg.peakrssi = str(tag['PeakRSSI'][0])

        now = self.get_clock().now().to_msg()
        tagmsg.header.stamp = now
        tagmsg.header.frame_id = 'tag_antenna_' + tagmsg.antenna_id + '_link'

        return tagmsg

    
    def tagReportCallback (self,llrpMsg):
        if self.isPaused:
            self.get_logger().debug('reading is paused ')
            return
        tags = llrpMsg.msgdict['RO_ACCESS_REPORT']['TagReportData']
        if len(tags):
            self.get_logger().debug('saw tag(s): %s' % pprint.pformat(tags))
        else:
            self.get_logger().debug('no tags seen')
            return
        for tag in tags:
            self.tagsNum += tag['TagSeenCount'][0]

            try:
                tagmsg = self.setTagMsg(tag)
                self.tags_pub.publish(tagmsg)
            except:
                self.get_logger().warn("error while get tag inform")
            
    def politeShutdown (self,factory):
        self.get_logger().info('polite shut down!')
        return factory.politeShutdown()

    def spin(self):
        rate = self.create_rate(self.rate)
        while rclpy.ok():
            rate.sleep()
            
if __name__ == '__main__':
    DiffCtrl = rfidbotTagsReader()

