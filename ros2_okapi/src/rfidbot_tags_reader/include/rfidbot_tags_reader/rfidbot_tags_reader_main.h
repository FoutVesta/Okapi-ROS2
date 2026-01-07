/**********************************************************************************************
* file name: rfidbot_tags_reader_main.h
* description: provide the rfid reader main function, it is a header file
* 
*----------------------------------------------------------------------------------------------
* author: Jian Zhang
* date:  13 May
**********************************************************************************************/


#ifndef _RFIDBOT_TAGS_READER_MAIN_H
#define _RFIDBOT_TAGS_READER_MAIN_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>

#include "rfidbot_tags_interfaces/msg/tag_reader.hpp"   

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <rfidbot_tags_reader/ltkcpp.h>

using namespace LLRP;
//using namespace rfidbot_tags_reader;

class CRFIDBotTagsReader
{
    public:
        /** Verbose level, incremented by each -v on command line */
        int m_Verbose;

        /** Connection to the LLRP reader */
        CConnection *m_pConnectionToReader;

        /** Node handle (ROS2 shared pointer) */
        explicit CRFIDBotTagsReader(rclcpp::Node::SharedPtr node);
     
        int run(char *pReaderHostName);

        int checkConnectionStatus(void);

        int scrubConfiguration(void);

        int resetConfigurationToFactoryDefaults(void);

        int deleteAllROSpecs (void);
        
        int deleteAllAccessSpecs (void);

        int addROSpec (void);

        int enableROSpec (void);

        int disableROSpec (void);

        int startROSpec (void);

        int stopROSpec (void);

        int awaitAndPrintReport (void);

        void printTagReportData(CRO_ACCESS_REPORT *pRO_ACCESS_REPORT);

        void printOneTagReportData (CTagReportData *pTagReportData);

        void handleReaderEventNotification (CReaderEventNotificationData *pNtfData);

        void handleAntennaEvent(CAntennaEvent *pAntennaEvent);

        void handleReaderExceptionEvent (CReaderExceptionEvent *pReaderExceptionEvent);

        int checkLLRPStatus (CLLRPStatus *pLLRPStatus, const char *pWhatStr);

        CMessage *transact (CMessage *pSendMsg);

        CMessage *recvMessage (int nMaxMS);

        int sendMessage (CMessage *pSendMsg);

        void printXMLMessage (CMessage *pMessage);

        void updateTagReaderMsgHeader(
            const rfidbot_tags_interfaces::msg::TagReader::SharedPtr rfid_msg,
            int iAntennaID);

        void setActiveAntennae(unsigned int uAntenae);
        void setTransmitPower(unsigned int uTransPower);
        void setTagFilter(std::string sTagFilter);
        void deletePosPuaseInventory(void);
        void resumeInventorywithNewPos(void);
        void setSession(unsigned int uSession);

        /* ROS2 subscription callbacks */
        void deletePosPuaseInventoryCallBack(const std_msgs::msg::Bool::SharedPtr msg);
        void resumeInventorywithNewPosCallBack(const std_msgs::msg::Bool::SharedPtr msg);
        void setTagFilterCallBack(const std_msgs::msg::String::SharedPtr msg);
        void setTransmitPowerCallBack(const std_msgs::msg::Int16::SharedPtr msg);
        void setActiveAntennaeCallBack(const std_msgs::msg::Int32::SharedPtr msg);

    private:
        rclcpp::Publisher<rfidbot_tags_interfaces::msg::TagReader>::SharedPtr rfid_tags_pub_;  // Publisher for output RFID tags
        unsigned int m_uAntennaBits;     /* every bit is the switch for an antenna.
                                        m_uAntennaBits[0]=1 for antenna 1 on, 
                                        m_uAntennaBits[0]=0 off antenna 1. 
                                        If uAntennaBits == 0, enable all antennas. */
        unsigned int m_uTransmitPower;   // all antennas set with same transmit power
        std::string m_sTagFilter;        // tag filter for all antennas      
        char* m_ReaderHost;
        bool m_isPause = false;
        bool m_isPauseRecovery = false;
        bool m_isStateChange = false;
        unsigned int m_uSession;

        CAntennaConfiguration *setAntennaConfig(int iAntennaID,
                            unsigned int uTransmitPower, unsigned int uSession);

        unsigned int getAntennaeNum(unsigned int uAntennBits);

        void setAntennaIDS(llrp_u16v_t* antennaIDsPtr, unsigned int uAntennBits);

        rclcpp::Node::SharedPtr node_; // ROS2 node reference
};


/* BEGIN forward declarations */
int main (int ac, char *av[]);
void usage (char *pProgName);
/* END forward declarations */

#endif /* !_RFIDBOT_TAGS_READER_MAIN_H */



