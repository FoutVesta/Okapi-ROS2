#!/usr/bin/python3
'''
*File name: rfidbot_tags_reader
_main.py
*Description: reader rfid tags by LLRP, and send the tags by topic "rfid_tags"
*             it can not kill by ctrl+c,must use /homne/rfid/jiannyctrlscripts/killpython to kill.
*             or you can kill all python process to kill it
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
class rfidbotTagsReader():

    def __init__(self):

        rospy.init_node("rfid_tags_reader") #,log_level=rospy.INFO)
        self.nodename = rospy.get_name()
        rospy.logwarn("%s,%s started" ,"rfid_tags_reader", self.nodename)
        self.rate = 1

        #jian add for filter setting
        self.tagFilter = ''
        self.isPaused = False
              
        rospy.Subscriber('/pause_inventory_delete_pos', Bool, self.pauseInventoryAndDelPosCallback) 
        rospy.Subscriber('/resume_inventory_enable_new_pos', Bool, self.resumeEnableNewPosCallback) 
        rospy.Subscriber('/set_tag_filter_to_reader', String, self.setTagFilteCallback) 
        rospy.Subscriber('/set_tx_power_to_reader', Int16, self.setTxPowerCallback)
        rospy.Subscriber('/set_new_antennas',String,self.setActiveAntennasCallback)
        self.tags_pub = rospy.Publisher('rfid_tags', tagReader,queue_size=10) 
        
        self.tagsNum = 0
        
        #initialize the LLRP
        rospy.loginfo('initial LLRP client')
        self.antennas = (1,2,3,4)
        self.host = rospy.get_param('~host','192.168.115.100')
        #self.host = '192.168.1.136'
        self.port = 5084
        
        # d.callback will be called when all connections have terminated normally.
        # use d.addCallback(<callable>) to define end-of-program behavior.
        d = defer.Deferred()
        d.addCallback(self.finish)
        
           
        self.session = rospy.get_param('~session',1)
        self.time = 0 # was None before
        self.tx_power = rospy.get_param('~txpower',200) #200 #170 is for read rate #150  #maxmium 200
        self.modulation = 'M4'
        self.tari = 0
        self.population = 1 #4
        fac = llrp.LLRPClientFactory(onFinish=d,
            duration=self.time,
            report_every_n_tags= 1,#args.every_n, jian modified
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
        
        # tagReportCallback will be called every time the reader sends a TagReport
        # message (i.e., when it has "seen" tags).
        fac.addTagReportCallback(self.tagReportCallback)

        reactor.connectTCP(self.host, self.port, fac, timeout=3)
        rospy.loginfo('the host is %s',self.host)

        # catch ctrl-C and stop inventory before disconnecting
        reactor.addSystemEventTrigger('before', 'shutdown', self.politeShutdown, fac)

        self.factory = fac
        self.factory.setTagFilter(self.tagFilter)
        #self.fac.addStateCallback(llrp.LLRPClient.STATE_CONNECTED, self.shutdownReader)
        #After this, the control is handed to reactor loop
        reactor.run()
        
    def finish (self,_):
        rospy.loginfo('total # of tags seen: %s',self.tagsNum)
        if reactor.running:
            reactor.stop()
            
    #jian add for filter setting
    def pauseInventoryAndDelPosCallback(self,msg):
        if msg.data:
            self.isPaused = True
            self.factory.pauseAndDeleteRosPECInventory()

    def setTagFilteCallback(self,msg):
        self.tagFilter=msg.data
        self.factory.setTagFilter(self.tagFilter)
        rospy.logwarn('get the tag filter %s',self.tagFilter)

    def resumeEnableNewPosCallback(self,msg):
        if msg.data:
            self.isPaused = False
            self.factory.resumeAndStartNewRosPECInventory()

    def setTxPowerCallback(self,msg):   
        txPower = msg.data
        self.tx_power = txPower
        rospy.logwarn('receive the new power %s',self.tx_power)
        self.factory.setNewTxPower(self.tx_power)

    def setActiveAntennasCallback(self,msg):
        newAntennas = msg.data
        self.antennas = newAntennas
        self.factory.setNewActiveAntennas(self.antennas)
        
    #now the just use the time of received as the time stamp, 
    #can update to use FirstSeenTimestampUTC as the time stamp
    def setTagMsg(self, tag):
        tagmsg = tagReader()
        #try:
        #rospy.logwarn("lengh of tag %s",len(tag))
        AntennaId = tag['AntennaID'][0]
        tagmsg.EPC = tag['EPC-96']
        #except Exception:
        #    rospy.logwarn( "Got Exception %s" ,str(traceback.format_exc()) )
        #    return None
        tagmsg.AntennaID = str(AntennaId)
        #convert unix time into date + hours,min,seconds 
        fstime = datetime.datetime.fromtimestamp(
            tag['FirstSeenTimestampUTC'][0]/1000000.0).strftime('%Y-%m-%d %H:%M:%S:%f')
        tagmsg.FristSeenTimestampUTC = str(fstime)#tag['FirstSeenTimestampUTC'][0])
        #convert unix time into date + hours,min,seconds 
        lstime = datetime.datetime.fromtimestamp(
            tag['LastSeenTimestampUTC'][0]/1000000.0).strftime('%Y-%m-%d %H:%M:%S:%f')
        tagmsg.LastSeenTimestampUTC = str(lstime)#tag['LastSeenTimestampUTC'][0])
        tagmsg.PeakRSSI = str(tag['PeakRSSI'][0])
        
        now = rospy.Time.now()
        #rospy.logwarn('tiem %s, utc %s',now,tag['FirstSeenTimestampUTC'][0])
        tagmsg.header.stamp = now
        #tagmsg.header.stamp.secs = tag['FirstSeenTimestampUTC'][0]/1000000
        #tagmsg.header.stamp.nsecs = tag['FirstSeenTimestampUTC'][0]%1000000 * 1000
        tagmsg.header.frame_id = 'tag_antenna_' + tagmsg.AntennaID + '_link'

        '''if AntennaId == 1:
            tagmsg.header.frame_id = 'tag_antenna_1_link'
        elif AntennaId == 2:
            tagmsg.header.frame_id = 'tag_antenna_2_link'
        elif AntennaId == 3:
            tagmsg.header.frame_id = 'tag_antenna_3_link'
        elif AntennaId == 4:
            tagmsg.header.frame_id = 'tag_antenna_4_link'''

        return tagmsg

    
    def tagReportCallback (self,llrpMsg):
        """Function to run each time the reader reports seeing tags."""
        if self.isPaused:
            rospy.logdebug('reading is paused ')
            return
        tags = llrpMsg.msgdict['RO_ACCESS_REPORT']['TagReportData']
        if len(tags):
            rospy.logdebug('saw tag(s): %s',pprint.pformat(tags))
        else:
            rospy.logdebug('no tags seen')
            return
        for tag in tags:
            self.tagsNum += tag['TagSeenCount'][0]
            
            #rospy.loginfo('send tag msg: %s, total tag: %s',tag['EPC-96'],self.tagsNum)
            #rospy.logwarn('tag %s',tag)

            #rospy.logwarn("the tag %s",tag['EPC-96'])
            try:
                tagmsg = self.setTagMsg(tag)
                self.tags_pub.publish(tagmsg)
            except:
                rospy.logwarn("error while get tag inform")
            #self.tags_pub.publish(tagmsg)
        #rospy.logwarn("the tag num %s",self.tagsNum)
            
    def politeShutdown (self,factory):
        rospy.loginfo('polite shut down!')
        return factory.politeShutdown()

    def spin(self):
        self.r = rospy.Rate(self.rate) 

        i = 0
        while not rospy.is_shutdown():
            #rospy.loginfo('tags reading')
            self.r.sleep()
            '''rospy.loginfo('spinning :%s', i)
            i += 1
            if i > 1:
                reactor.run()
            elif i > 15:
                self.politeShutdown(self.fac)
                reactor.stop()'''
            
if __name__ == '__main__':
    """ main """
    DiffCtrl = rfidbotTagsReader()
    #DiffCtrl.spin()
        
