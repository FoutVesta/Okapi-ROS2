
/*
 ***************************************************************************
 *  Copyright 2007,2008 Impinj, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ***************************************************************************
 */

/**
 *****************************************************************************
 **
 ** @file  example1.cpp
 **
 ** @brief Simple example of LTKCPP that talks to an actual reader
 **
 ** This is sometimes called the "LLRP Hello World" program
 **
 ** The steps:
 **     - Connect to the reader (TCP)
 **     - Make sure the connection status is good
 **     - Clear (scrub) the reader configuration
 **     - Add and enable a ROSpec that does a simple operation
 **         - Uses mostly default settings except for ROReportSpec
 **         - Uses all antennas
 **         - Starts on command
 **         - Runs for 5 seconds
 **         - Reports all accumulated tag observations
 **     - Run the ROSpec 5 times
 **     - Clear (scrub) the reader configuration
 **
 ** This program can be run with zero, one, or two verbose options (-v).
 **     no -v -- Only prints the tag report and errors
 **     -v    -- Also prints one line progress messages
 **     -vv   -- Also prints all LLRP messages as XML text
 **
 *****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string>
#include <boost/thread.hpp>

#include "rfidbot_tags_reader/ltkcpp.h"
#include "rfidbot_tags_reader/rfidbot_tags_reader_main.h"

using namespace LLRP;

std::shared_ptr<CRFIDBotTagsReader> rfidReaerPtr;

void pauseCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (NULL == rfidReaerPtr)
    {
        RCLCPP_WARN(rclcpp::get_logger("rfidbot_tags_reader"), 
            "the rfid tags reader not initialized! can not processs");
        return;
    }
    rfidReaerPtr->deletePosPuaseInventoryCallBack(msg);
}

void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (NULL == rfidReaerPtr)
    {
        RCLCPP_WARN(rclcpp::get_logger("rfidbot_tags_reader"), 
            "the rfid tags reader not initialized! can not processs");
        return;
    }
    rfidReaerPtr->resumeInventorywithNewPosCallBack(msg);
}

void setTxPowerCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
    if (NULL == rfidReaerPtr)
    {
        RCLCPP_WARN(rclcpp::get_logger("rfidbot_tags_reader"), 
            "the rfid tags reader not initialized! can not processs");
        return;
    }
    
    rfidReaerPtr->setTransmitPowerCallBack(msg);
}

void setAntennaeCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (NULL == rfidReaerPtr)
    {
        RCLCPP_WARN(rclcpp::get_logger("rfidbot_tags_reader"), 
            "the rfid tags reader not initialized! can not processs");
        return;
    }

    rfidReaerPtr->setActiveAntennaeCallBack(msg);
}

void setFilterCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (NULL == rfidReaerPtr)
    {
        RCLCPP_WARN(rclcpp::get_logger("rfidbot_tags_reader"), 
            "the rfid tags reader not initialized! can not processs");
        return;
    }
    rfidReaerPtr->setTagFilterCallBack(msg);

}
void runReader(char *pReaderHostName)
{
    rfidReaerPtr->run(pReaderHostName);
}

/*******************************************************************************
 **
 ** @brief  Command main routine
 **
 ** Command synopsis:
 **
 **     example1 [-v[v]] READERHOSTNAME
 **
 ** @exitcode   0               Everything *seemed* to work.
 **             1               Bad usage
 **             2               Run failed
 **
 *****************************************************************************/
int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);

    /* ROS2 node replaces NodeHandle */
    auto node = std::make_shared<rclcpp::Node>("rfidbot_tags_reader_main");

    std::string readerHost;
    char *pReaderHostName;
    int rc,isessionId=-1,iTxPower=0,iAntennaIds = 0;
    boost::thread *workThread;

    /* In ROS2, parameters must be declared first */
    node->declare_parameter<std::string>("host", "");
    node->declare_parameter<int>("session", -1);
    node->declare_parameter<int>("txpower", 0);
    node->declare_parameter<int>("antennaIds", 0);

    /* Create reader object (pass node instead of nh) */
    rfidReaerPtr = std::make_shared<CRFIDBotTagsReader>(node);

    /* ROS2 subscriptions */
    auto sub_pause  = node->create_subscription<std_msgs::msg::Bool>(
        "/pause_inventory_delete_pos", 20, pauseCallback);

    auto sub_resume = node->create_subscription<std_msgs::msg::Bool>(
        "/resume_inventory_enable_new_pos", 20, resumeCallback);

    auto sub_txpow  = node->create_subscription<std_msgs::msg::Int16>(
        "/set_tx_power_to_reader", 20, setTxPowerCallback);

    auto sub_anten  = node->create_subscription<std_msgs::msg::Int32>(
        "/set_new_antennas", 20, setAntennaeCallback);

    auto sub_filter = node->create_subscription<std_msgs::msg::String>(
        "/set_tag_filter_to_reader", 20, setFilterCallback);

    /*
    // Old ROS1 subscriber example
    ros::Subscriber sub_pause = n.subscribe<std_msgs::Bool>("/pause_inventory_delete_pos", 1, 
        boost::bind(&testCallback, _1));
    */

    /* get parameters */

    if(!node->get_parameter("host", readerHost))
    {
        RCLCPP_ERROR(node->get_logger(), "get host error,please check related launch file, %s!", readerHost.c_str());
        exit(0);
    }
    pReaderHostName = (char*)readerHost.c_str();
    RCLCPP_INFO(node->get_logger(), " reader host %s", pReaderHostName);

    if(!node->get_parameter("session", isessionId))
    {
        RCLCPP_ERROR(node->get_logger(), "get session error,please check related launch file, %d!", isessionId);
        exit(0);
    }
    RCLCPP_INFO(node->get_logger(), "get session %d", isessionId);
    rfidReaerPtr->setSession(isessionId);

    if(!node->get_parameter("txpower", iTxPower))
    {
        RCLCPP_ERROR(node->get_logger(), "get txpower error,please check related launch file, %d!", iTxPower);
        exit(0);
    }
    RCLCPP_INFO(node->get_logger(), "get tx power %d", iTxPower);
    rfidReaerPtr->setTransmitPower(iTxPower);

    if(!node->get_parameter("antennaIds", iAntennaIds))
    {
        RCLCPP_ERROR(node->get_logger(), "get antenna id error %d, set to default all antenna!", iAntennaIds);
        iAntennaIds = 0;
    }
    RCLCPP_INFO(node->get_logger(), "get AntennaIds %d", iAntennaIds);
    rfidReaerPtr->setActiveAntennae(iAntennaIds);

    /* Run application, capture return value for exit status */
    workThread = new boost::thread(runReader, pReaderHostName);

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "INFO: Done\n");

    /* Exit with the right status */
    rfidReaerPtr.reset();
    if(0 == rc)
    {
        exit(0);
    }
    else
    {
        exit(2);
    }
    /* NOTREACHED */
}

/***************************************************************************
*function name: CRFIDBotTagsReader
*description: construction function for CRFIDBotTagsReader
*             it will initial the message publisher
*             provide message based interface by subscriber
* return: Void
****************************************************************************/
CRFIDBotTagsReader::CRFIDBotTagsReader(std::shared_ptr<rclcpp::Node> node) :
    m_Verbose(0), m_pConnectionToReader(nullptr), node_(node)
{
    rfid_tags_pub_ = node_->create_publisher<rfidbot_tags_interfaces::msg::TagReader>(
        "rfid_tags",
        20
    );
}


/**
 *****************************************************************************
 **
 ** @brief  Run the application
 **
 ** The steps:
 **     - Instantiate connection
 **     - Connect to LLRP reader (TCP)
 **     - Make sure the connection status is good
 **     - Clear (scrub) the reader configuration
 **     - Configure for what we want to do
 **     - Run inventory operation 5 times
 **     - Again, clear (scrub) the reader configuration
 **     - Disconnect from reader
 **     - Destruct connection
 **
 ** @param[in]  pReaderHostName String with reader name
 **
 ** @return      0              Everything worked.
 **             -1              Failed allocation of type registry
 **             -2              Failed construction of connection
 **             -3              Could not connect to reader
 **              1              Reader connection status bad
 **              2              Cleaning reader config failed
 **              3              Adding ROSpec failed
 **              4              Enabling ROSpec failed
 **              5              Something went wrong running the ROSpec
 **
 *****************************************************************************/
int CRFIDBotTagsReader::run (char *pReaderHostName)
{
    CTypeRegistry *pTypeRegistry;
    CConnection *pConn;
    int retc;
    rclcpp::Rate r(20.0);

    /*Allocate the type registry. This is needed by the connection to decode.*/
    pTypeRegistry = getTheTypeRegistry();
    if(NULL == pTypeRegistry)
    {
        printf("ERROR: getTheTypeRegistry failed\n");
        return -1;
    }

    /* Construct a connection (LLRP::CConnection).Using a 32kb max frame size for send/recv.
     * The connection object is ready for business but not actually connected to the reader yet. */
    pConn = new CConnection(pTypeRegistry, 1024u*1024u); //32u*1024u);
    if(NULL == pConn)
    {
        printf("ERROR: new CConnection failed\n");
        return -2;
    }

    /*Open the connection to the reader*/
    retc = pConn->openConnectionToReader(pReaderHostName);
    if(0 != retc)
    {
        printf("ERROR: connect: %s (%d)\n", pConn->getConnectError(), retc);
        delete pConn;
        return -3;
    }
    m_ReaderHost = pReaderHostName;

    /*Record the pointer to the connection object so other routines can use it.*/
    m_pConnectionToReader = pConn;
    //ROS_INFO("INFO: Connected, checking status....\n");

    /* Commence the sequence and check for errors as we go.
     * See comments for each routine for details. Each routine prints messages.*/
    retc = 1;
    if(0 == checkConnectionStatus())
    {
        retc = 2;
        if(0 == scrubConfiguration())
        {
            retc = 3;
            if(0 == addROSpec())
            {
                retc = 4;
                if(0 == enableROSpec())
                {
                    if(0 != startROSpec())
                    {
                        return 5;
                    } 
                    retc = 0;  //all previous step is OK set retc to 0
                    
                    while (rclcpp::ok()) //continue reading
                    {
                        if(m_isStateChange)
                        {
                            if(m_isPause)
                            {
                                deletePosPuaseInventory();
                            }
                            else
                            {
                                resumeInventorywithNewPos();
                            }

                            m_isStateChange = false;
                        }

                        if(m_isPause)
                        {
                            continue;
                        }
                        
                        if(0 != awaitAndPrintReport())
                        { 
                            RCLCPP_WARN(node_->get_logger(), "the report is failed!");
                            //break;
                        }
                    }

                }
            }

            /*After we're done, try to leave the reader in a clean state for next use. 
             *This is best effort and no checking of the result is done.*/
            
            RCLCPP_INFO(node_->get_logger(), "INFO: Clean up reader configuration...\n");
           
            //before the clean up deletePosPuaseInventory this will guaranee next time use
            deletePosPuaseInventory();
            scrubConfiguration();
        }
    }


    RCLCPP_INFO(node_->get_logger(), "INFO: Finished\n");
  
    //Close the connection and release its resources
    pConn->closeConnectionToReader();
    delete pConn;
    m_pConnectionToReader = NULL;

    //Done with the registry.
    delete pTypeRegistry;

    //When we get here all allocated memory should have been deallocated.
    return retc;
}


/**
 *****************************************************************************
 **
 ** @brief  Await and check the connection status message from the reader
 **
 ** We are expecting a READER_EVENT_NOTIFICATION message that
 ** tells us the connection is OK. The reader is suppose to
 ** send the message promptly upon connection.
 **
 ** If there is already another LLRP connection to the
 ** reader we'll get a bad Status.
 **
 ** The message should be something like:
 **
 **     <READER_EVENT_NOTIFICATION MessageID='0'>
 **       <ReaderEventNotificationData>
 **         <UTCTimestamp>
 **           <Microseconds>1184491439614224</Microseconds>
 **         </UTCTimestamp>
 **         <ConnectionAttemptEvent>
 **           <Status>Success</Status>
 **         </ConnectionAttemptEvent>
 **       </ReaderEventNotificationData>
 **     </READER_EVENT_NOTIFICATION>
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong
 **
 *****************************************************************************/

int
CRFIDBotTagsReader::checkConnectionStatus (void)
{
    CMessage *                  pMessage;
    CREADER_EVENT_NOTIFICATION *pNtf;
    CReaderEventNotificationData *pNtfData;
    CConnectionAttemptEvent *   pEvent;

    /*
     * Expect the notification within 10 seconds.
     * It is suppose to be the very first message sent.
     */
    pMessage = recvMessage(10000);

    /*
     * recvMessage() returns NULL if something went wrong.
     */
    if(NULL == pMessage)
    {
        /* recvMessage already tattled */
        goto fail;
    }

    /*
     * Check to make sure the message is of the right type.
     * The type label (pointer) in the message should be
     * the type descriptor for READER_EVENT_NOTIFICATION.
     */
    if(&CREADER_EVENT_NOTIFICATION::s_typeDescriptor != pMessage->m_pType)
    {
        goto fail;
    }

    /*
     * Now that we are sure it is a READER_EVENT_NOTIFICATION,
     * traverse to the ReaderEventNotificationData parameter.
     */
    pNtf = (CREADER_EVENT_NOTIFICATION *) pMessage;
    pNtfData = pNtf->getReaderEventNotificationData();
    if(NULL == pNtfData)
    {
        goto fail;
    }

    /*
     * The ConnectionAttemptEvent parameter must be present.
     */
    pEvent = pNtfData->getConnectionAttemptEvent();
    if(NULL == pEvent)
    {
        goto fail;
    }

    /*
     * The status in the ConnectionAttemptEvent parameter
     * must indicate connection success.
     */
    if(ConnectionAttemptStatusType_Success != pEvent->getStatus())
    {
        goto fail;
    }

    /*
     * Done with the message
     */
    delete pMessage;

    if(m_Verbose)
    {
        printf("INFO: Connection status OK\n");
    }

    /*
     * Victory.
     */
    return 0;

  fail:
    /*
     * Something went wrong. Tattle. Clean up. Return error.
     */
    printf("ERROR: checkConnectionStatus failed\n");
    delete pMessage;
    return -1;
}


/**
 *****************************************************************************
 **
 ** @brief  Scrub the reader configuration
 **
 ** The steps:
 **     - Try to reset configuration to factory defaults,
 **       this feature is optional and may not be supported
 **       by the reader.
 **     - Delete all ROSpecs
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong
 **
 *****************************************************************************/

int CRFIDBotTagsReader::scrubConfiguration (void)
{
    if(0 != resetConfigurationToFactoryDefaults())
    {
        return -1;
    }

    if(0 != deleteAllROSpecs())
    {
        return -2;
    }

    if (0 != deleteAllAccessSpecs())
    {
        return -3;
    }
    
    return 0;
}


/**
 *****************************************************************************
 **
 ** @brief  Send a SET_READER_CONFIG message that resets the
 **         reader to factory defaults.
 **
 ** NB: The ResetToFactoryDefault semantics vary between readers.
 **     It might have no effect because it is optional.
 **
 ** The message is:
 **
 **     <SET_READER_CONFIG MessageID='101'>
 **       <ResetToFactoryDefault>1</ResetToFactoryDefault>
 **     </SET_READER_CONFIG>
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong
 **
 *****************************************************************************/

int
CRFIDBotTagsReader::resetConfigurationToFactoryDefaults (void)
{
    CMessage *pRspMsg;
    CSET_READER_CONFIG *pCmd;
    CSET_READER_CONFIG_RESPONSE *pRsp;

    /*
     * Compose the command message
     */
    pCmd = new CSET_READER_CONFIG();
    pCmd->setMessageID(101);
    pCmd->setResetToFactoryDefault(1);

    /*
     * Send the message, expect the response of certain type
     */
    pRspMsg = transact(pCmd);

    /*
     * Done with the command message
     */
    delete pCmd;

    /*
     * transact() returns NULL if something went wrong.
     */
    if(NULL == pRspMsg)
    {
        /* transact already tattled */
        return -1;
    }

    /*
     * Cast to a SET_READER_CONFIG_RESPONSE message.
     */
    pRsp = (CSET_READER_CONFIG_RESPONSE *) pRspMsg;

    /*
     * Check the LLRPStatus parameter.
     */
    if(0 != checkLLRPStatus(pRsp->getLLRPStatus(),
                        "resetConfigurationToFactoryDefaults"))
    {
        /* checkLLRPStatus already tattled */
        delete pRspMsg;
        return -1;
    }

    /*
     * Done with the response message.
     */
    delete pRspMsg;

    /*
     * Tattle progress, maybe
     */
    if(m_Verbose)
    {
        printf("INFO: Configuration reset to factory defaults\n");
    }

    /*
     * Victory.
     */
    return 0;
}


/**
 *****************************************************************************
 **
 ** @brief  Delete all ROSpecs using DELETE_ROSPEC message
 **
 ** Per the spec, the DELETE_ROSPEC message contains an ROSpecID
 ** of 0 to indicate we want all ROSpecs deleted.
 **
 ** The message is
 **
 **     <DELETE_ROSPEC MessageID='102'>
 **       <ROSpecID>0</ROSpecID>
 **     </DELETE_ROSPEC>
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong
 **
 *****************************************************************************/

int CRFIDBotTagsReader::deleteAllROSpecs (void)
{
    CDELETE_ROSPEC *            pCmd;
    CMessage *                  pRspMsg;
    CDELETE_ROSPEC_RESPONSE *   pRsp;

    /*
     * Compose the command message
     */
    pCmd = new CDELETE_ROSPEC();
    pCmd->setMessageID(102);
    pCmd->setROSpecID(0);               /* All */

    /*
     * Send the message, expect the response of certain type
     */
    pRspMsg = transact(pCmd);

    /*
     * Done with the command message
     */
    delete pCmd;

    /*
     * transact() returns NULL if something went wrong.
     */
    if(NULL == pRspMsg)
    {
        /* transact already tattled */
        return -1;
    }

    /*
     * Cast to a DELETE_ROSPEC_RESPONSE message.
     */
    pRsp = (CDELETE_ROSPEC_RESPONSE *) pRspMsg;

    /*
     * Check the LLRPStatus parameter.
     */
    if(0 != checkLLRPStatus(pRsp->getLLRPStatus(), "deleteAllROSpecs"))
    {
        /* checkLLRPStatus already tattled */
        delete pRspMsg;
        return -2;
    }

    /*
     * Done with the response message.
     */
    delete pRspMsg;

    /*
     * Tattle progress, maybe
     */
    if(m_Verbose)
    {
        printf("INFO: All ROSpecs are deleted\n");
    }

    /*
     * Victory.
     */
    return 0;
}


/**
 *****************************************************************************
 **
 ** @brief  Delete all AccessSpecs using DELETE_ACCESSSPEC message
 **
 ** Per the spec, the DELETE_ACCESSSPEC message contains an AccessSpecID
 ** of 0 to indicate we want all AccessSpecs deleted.
 **
 ** The message is
 **
 **     <DELETE_ACCESSSPEC MessageID='103'>
 **       <AccessSpecID>0</AccessSpecID>
 **     </DELETE_ACCESSSPEC>
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong
 **
 *****************************************************************************/

int CRFIDBotTagsReader::deleteAllAccessSpecs (void)
{
    CDELETE_ACCESSSPEC *        pCmd;
    CMessage *                  pRspMsg;
    CDELETE_ACCESSSPEC_RESPONSE *pRsp;

    /*
     * Compose the command message
     */
    pCmd = new CDELETE_ACCESSSPEC();
    pCmd->setMessageID(103);
    pCmd->setAccessSpecID(0);           /* All */

    /*
     * Send the message, expect the response of certain type
     */
    pRspMsg = transact(pCmd);

    /*
     * Done with the command message
     */
    delete pCmd;

    /*
     * transact() returns NULL if something went wrong.
     */
    if(NULL == pRspMsg)
    {
        /* transact already tattled */
        return -1;
    }

    /*
     * Cast to a DELETE_ACCESSSPEC_RESPONSE message.
     */
    pRsp = (CDELETE_ACCESSSPEC_RESPONSE *) pRspMsg;

    /*
     * Check the LLRPStatus parameter.
     */
    if(0 != checkLLRPStatus(pRsp->getLLRPStatus(), "deleteAllAccessSpecs"))
    {
        /* checkLLRPStatus already tattled */
        delete pRspMsg;
        return -1;
    }

    /*
     * Done with the response message.
     */
    delete pRspMsg;

    /*
     * Tattle progress, maybe
     */
    if(m_Verbose)
    {
        printf("INFO: All AccessSpecs are deleted\n");
    }

    /*
     * Victory.
     */
    return 0;
}


/**
 *****************************************************************************
 **
 ** @brief  Add AntennaConfig message
 **
 ** The message is
 **             <AntennaConfiguration>
 **                 <AntennaID>1</AntennaID>
 **                 <C1G2InventoryCommand>
 **                     <TagInventoryStateAware>false</TagInventoryStateAware>
 ** BEGIN C1G2RFControl only if mode specified on command line
 **                     <C1G2RFControl>
 **                         <ModeIndex>$MODE</ModeIndex>
 **                         <Tari>0</Tari>
 **                     </C1G2RFControl>
 ** END   C1G2RFControl only if mode specified on command line
 **                      <C1G2SingulationControl>
 **                          <Session>2</Session>
 **                          <TagPopulation>32</TagPopulation>
 **                          <TagTransitTime>0</TagTransitTime>
 **                      </C1G2SingulationControl>
 **                 </C1G2InventoryCommand>
 **                 <RFTransmitter>
 **                     <TransmitPower>200</TransmitPower>
 **                 </RFTransmitter>
 **             </AntennaConfiguration>
 **     
 ** @inpute: iAntennaID, the antenna ID, if 0 for all antennae
 **          uTransmitPower, the transmit power
 **          uSession, session id
 ** @return point of antenna config
 *****************************************************************************/

CAntennaConfiguration * CRFIDBotTagsReader::setAntennaConfig(int iAntennaID,
        unsigned int uTransmitPower,unsigned int uSession)
{
    CC1G2SingulationControl *pC1G2SingulationControl = new CC1G2SingulationControl();
    pC1G2SingulationControl->setSession(uSession);
    pC1G2SingulationControl->setTagPopulation(32);

    CC1G2InventoryCommand *pC1G2InventoryCommand = new CC1G2InventoryCommand();
    pC1G2InventoryCommand->setC1G2SingulationControl(pC1G2SingulationControl);
    //add filter: CC1G2Filter
    //pC1G2InventoryCommand->addC1G2Filter(CC1G2Filter)

    CRFTransmitter *pTransmitter = new CRFTransmitter();
    pTransmitter->setHopTableID(1);
    pTransmitter->setChannelIndex(1);
    pTransmitter->setTransmitPower(uTransmitPower);

    CAntennaConfiguration *pAntennaConfiguration = new CAntennaConfiguration();
    pAntennaConfiguration->addAirProtocolInventoryCommandSettings(pC1G2InventoryCommand);
    pAntennaConfiguration->setAntennaID(iAntennaID);//1);
    pAntennaConfiguration->setRFTransmitter(pTransmitter);

    return pAntennaConfiguration;
}


unsigned int CRFIDBotTagsReader::getAntennaeNum(unsigned int uAntennBits)
{
    unsigned int uantennaNum = 0;
    if(0 == uAntennBits)  // enable all antenna return antenna num with 1
    {     
        return 1;
    }
    while(uAntennBits) 
    {
        if (uAntennBits & 1)
        {
            uantennaNum++;
        }
        uAntennBits >>= 1;
    }

    return uantennaNum;
}

void CRFIDBotTagsReader::setAntennaIDS(llrp_u16v_t* antennaIDsPtr,unsigned int uAntennBits)
{
    int id = 0, Antennaid = 0;
    if (0 == uAntennBits) // enable all antenna
    {
        antennaIDsPtr->m_pValue[0] = 0;  
    }
    while(uAntennBits) 
    {   
        Antennaid ++;
        if (uAntennBits & 1)
        {
            antennaIDsPtr->m_pValue[id++] = Antennaid;
        }
        uAntennBits >>= 1;
    }
}


/**
 *****************************************************************************
 **
 ** @brief  Add our ROSpec using ADD_ROSPEC message
 **
 ** This ROSpec waits for a START_ROSPEC message,
 ** then takes inventory on seting antennae.
 ** The tag report at every new tag
 **

 **
 ** The message is
 **
 **     <ADD_ROSPEC MessageID='201'>
 **       <ROSpec>
 **         <ROSpecID>123</ROSpecID>
 **         <Priority>0</Priority>
 **         <CurrentState>Disabled</CurrentState>
 **         <ROBoundarySpec>
 **           <ROSpecStartTrigger>
 **             <ROSpecStartTriggerType>Null</ROSpecStartTriggerType>
 **           </ROSpecStartTrigger>
 **           <ROSpecStopTrigger>
 **             <ROSpecStopTriggerType>Null</ROSpecStopTriggerType>
 **             <DurationTriggerValue>0</DurationTriggerValue>
 **           </ROSpecStopTrigger>
 **         </ROBoundarySpec>
 **         <AISpec>
 **           <AntennaIDs>0</AntennaIDs>    //0 for all antenna
 **           <AISpecStopTrigger>
 **             <AISpecStopTriggerType>Duration</AISpecStopTriggerType>
 **             <DurationTrigger>5000</DurationTrigger>
 **           </AISpecStopTrigger>
 **           <InventoryParameterSpec>
 **             <InventoryParameterSpecID>1234</InventoryParameterSpecID>
 **             <ProtocolID>EPCGlobalClass1Gen2</ProtocolID>
 ** if more antenna add a AntennaConfiguration
 **             <AntennaConfiguration>
 **                 <AntennaID>1</AntennaID>
 **                 <C1G2InventoryCommand>
 **                     <TagInventoryStateAware>false</TagInventoryStateAware>
 ** BEGIN C1G2RFControl only if mode specified on command line
 **                     <C1G2RFControl>
 **                         <ModeIndex>$MODE</ModeIndex>
 **                         <Tari>0</Tari>
 **                         </C1G2RFControl>
 ** END   C1G2RFControl only if mode specified on command line
 **                      <C1G2SingulationControl>
 **                          <Session>2</Session>
 **                          <TagPopulation>32</TagPopulation>
 **                          <TagTransitTime>0</TagTransitTime>
 **                      </C1G2SingulationControl>
 **                 </C1G2InventoryCommand>
 **                 <RFTransmitter>
 **                     <TransmitPower>200</TransmitPower>
 **                 </RFTransmitter>
 **             </AntennaConfiguration>
 **           </InventoryParameterSpec>
 **         </AISpec>
 **         <ROReportSpec>
 **           <ROReportTrigger>Upon_N_Tags_Or_End_Of_ROSpec</ROReportTrigger>
 **           <N>0</N>
 **           <TagReportContentSelector>
 **             <EnableROSpecID>0</EnableROSpecID>
 **             <EnableSpecIndex>0</EnableSpecIndex>
 **             <EnableInventoryParameterSpecID>0</EnableInventoryParameterSpecID>
 **             <EnableAntennaID>0</EnableAntennaID>
 **             <EnableChannelIndex>0</EnableChannelIndex>
 **             <EnablePeakRSSI>0</EnablePeakRSSI>
 **             <EnableFirstSeenTimestamp>0</EnableFirstSeenTimestamp>
 **             <EnableLastSeenTimestamp>0</EnableLastSeenTimestamp>
 **             <EnableTagSeenCount>0</EnableTagSeenCount>
 **             <EnableAccessSpecID>0</EnableAccessSpecID>
 **           </TagReportContentSelector>
 **         </ROReportSpec>
 **       </ROSpec>
 **     </ADD_ROSPEC>
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong
 **
 *****************************************************************************/
int CRFIDBotTagsReader::addROSpec (void)
{
    CROSpecStartTrigger *pROSpecStartTrigger = new CROSpecStartTrigger();
    pROSpecStartTrigger->setROSpecStartTriggerType(ROSpecStartTriggerType_Null);

    CROSpecStopTrigger *pROSpecStopTrigger = new CROSpecStopTrigger();
    pROSpecStopTrigger->setROSpecStopTriggerType(ROSpecStopTriggerType_Null);
    pROSpecStopTrigger->setDurationTriggerValue(0);     /* n/a */

    CROBoundarySpec *pROBoundarySpec = new CROBoundarySpec();
    pROBoundarySpec->setROSpecStartTrigger(pROSpecStartTrigger);
    pROBoundarySpec->setROSpecStopTrigger(pROSpecStopTrigger);

    CAISpecStopTrigger *pAISpecStopTrigger = new CAISpecStopTrigger();
    pAISpecStopTrigger->setAISpecStopTriggerType(AISpecStopTriggerType_Null);
    pAISpecStopTrigger->setDurationTrigger(0);
    //pAISpecStopTrigger->setTagObservationTrigger(pTagObservationTrigger);

    CInventoryParameterSpec *pInventoryParameterSpec = new CInventoryParameterSpec();
    pInventoryParameterSpec->setInventoryParameterSpecID(1234); //21); //
    pInventoryParameterSpec->setProtocolID(AirProtocols_EPCGlobalClass1Gen2);

    //set antennaIDs
    int antennaNum = getAntennaeNum(m_uAntennaBits);
    llrp_u16v_t AntennaIDs = llrp_u16v_t(antennaNum);
    setAntennaIDS(&AntennaIDs,m_uAntennaBits);
    RCLCPP_INFO(node_->get_logger(), "antennae num %d", antennaNum);
    for(int i=0; i<antennaNum;i++)
    {
        RCLCPP_INFO(node_->get_logger(), "[%d] antenna id is %d", i, AntennaIDs.m_pValue[i]);
    }

    //add antena config
    CAntennaConfiguration *pAntennaConfiguration = NULL;
    unsigned int uAntennaId;
    for (int i=0; i<antennaNum; i++)
    {
        uAntennaId = AntennaIDs.m_pValue[i];
        pAntennaConfiguration = setAntennaConfig(uAntennaId,m_uTransmitPower,m_uSession);
        pInventoryParameterSpec->addAntennaConfiguration(pAntennaConfiguration);
    }
     

    CAISpec *pAISpec = new CAISpec();
    pAISpec->setAntennaIDs(AntennaIDs);
    pAISpec->setAISpecStopTrigger(pAISpecStopTrigger);
    pAISpec->addInventoryParameterSpec(pInventoryParameterSpec);

    CTagReportContentSelector *pTagReportContentSelector = new CTagReportContentSelector();
    pTagReportContentSelector->setEnableROSpecID(FALSE);
    pTagReportContentSelector->setEnableSpecIndex(FALSE);
    pTagReportContentSelector->setEnableInventoryParameterSpecID(FALSE);
    pTagReportContentSelector->setEnableAntennaID(TRUE);
    pTagReportContentSelector->setEnableChannelIndex(FALSE);
    pTagReportContentSelector->setEnablePeakRSSI(FALSE);
    pTagReportContentSelector->setEnableFirstSeenTimestamp(FALSE);
    pTagReportContentSelector->setEnableLastSeenTimestamp(FALSE);
    pTagReportContentSelector->setEnableTagSeenCount(FALSE);
    pTagReportContentSelector->setEnableAccessSpecID(FALSE);

    CROReportSpec *pROReportSpec = new CROReportSpec();
    pROReportSpec->setROReportTrigger(ROReportTriggerType_Upon_N_Tags_Or_End_Of_ROSpec);
    pROReportSpec->setN(1);         /* 0 is Unlimited,1 is report every one tag */
    pROReportSpec->setTagReportContentSelector(pTagReportContentSelector);

    CROSpec *pROSpec = new CROSpec();
    pROSpec->setROSpecID(123);
    pROSpec->setPriority(0);
    pROSpec->setCurrentState(ROSpecState_Disabled);
    pROSpec->setROBoundarySpec(pROBoundarySpec);
    pROSpec->addSpecParameter(pAISpec);
    pROSpec->setROReportSpec(pROReportSpec);

    CADD_ROSPEC *pCmd;
    CMessage *pRspMsg;
    CADD_ROSPEC_RESPONSE *pRsp;

    /*Compose the command message.
     * N.B.: After the message is composed, all the parametersconstructed, 
            immediately above, are considered "owned" by the command message. 
            When it is destructed so too will the parameters be. */
    pCmd = new CADD_ROSPEC();
    pCmd->setMessageID(201);
    pCmd->setROSpec(pROSpec);

    /*Send the message, expect the response of certain type*/
    pRspMsg = transact(pCmd);

    /*Done with the command message.
     * N.B.: And the parameters */
    delete pCmd;

    /*transact() returns NULL if something went wrong.*/
    if(NULL == pRspMsg)
    {
        /* transact already tattled */
        return -1;
    }

    /* Cast to a ADD_ROSPEC_RESPONSE message.*/
    pRsp = (CADD_ROSPEC_RESPONSE *) pRspMsg;

    /*Check the LLRPStatus parameter.*/
    if(0 != checkLLRPStatus(pRsp->getLLRPStatus(), "addROSpec"))
    {
        /* checkLLRPStatus already tattled */
        delete pRspMsg;
        return -1;
    }

    /* Done with the response message.*/
    delete pRspMsg;

    /*Tattle progress, maybe*/
    RCLCPP_INFO(node_->get_logger(), "INFO: ROSpec added.");
   
    return 0;
}


/**
 *****************************************************************************
 **
 ** @brief  Enable our ROSpec using ENABLE_ROSPEC message
 **
 ** Enable the ROSpec that was added above.
 **
 ** The message we send is:
 **     <ENABLE_ROSPEC MessageID='202'>
 **       <ROSpecID>123</ROSpecID>
 **     </ENABLE_ROSPEC>
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong
 **
 *****************************************************************************/

int
CRFIDBotTagsReader::enableROSpec (void)
{
    CENABLE_ROSPEC *            pCmd;
    CMessage *                  pRspMsg;
    CENABLE_ROSPEC_RESPONSE *   pRsp;

    /*
     * Compose the command message
     */
    pCmd = new CENABLE_ROSPEC();
    pCmd->setMessageID(202);
    pCmd->setROSpecID(123);

    /*
     * Send the message, expect the response of certain type
     */
    pRspMsg = transact(pCmd);

    /*
     * Done with the command message
     */
    delete pCmd;

    /*
     * transact() returns NULL if something went wrong.
     */
    if(NULL == pRspMsg)
    {
        /* transact already tattled */
        return -1;
    }

    /*
     * Cast to a ENABLE_ROSPEC_RESPONSE message.
     */
    pRsp = (CENABLE_ROSPEC_RESPONSE *) pRspMsg;

    /*
     * Check the LLRPStatus parameter.
     */
    if(0 != checkLLRPStatus(pRsp->getLLRPStatus(), "enableROSpec"))
    {
        /* checkLLRPStatus already tattled */
        delete pRspMsg;
        return -1;
    }

    /*
     * Done with the response message.
     */
    delete pRspMsg;

    /*
     * Tattle progress, maybe
     */
    if(m_Verbose)
    {
        printf("INFO: ROSpec enabled\n");
    }

    /*
     * Victory.
     */
    return 0;
}

/**
 *****************************************************************************
 **
 ** @brief  disable all ROSpec using DISABLE_ROSPEC message
 **
 ** Enable the ROSpec that was added above.
 **
 ** The message we send is:
 **     <DISABLE_ROSPEC MessageID='205'>
 **       <ROSpecID>0</ROSpecID>
 **     </ENABLE_ROSPEC>
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong
 **
 *****************************************************************************/

int CRFIDBotTagsReader::disableROSpec (void)
{
    CDISABLE_ROSPEC *            pCmd;
    CMessage *                  pRspMsg;
    CDISABLE_ROSPEC_RESPONSE *   pRsp;

    /*
     * Compose the command message
     */
    pCmd = new CDISABLE_ROSPEC();
    pCmd->setMessageID(205);
    pCmd->setROSpecID(0);   //0 disable all ROSpecs

    /*
     * Send the message, expect the response of certain type
     */
    pRspMsg = transact(pCmd);

    /*
     * Done with the command message
     */
    delete pCmd;

    /*
     * transact() returns NULL if something went wrong.
     */
    if(NULL == pRspMsg)
    {
        /* transact already tattled */
        return -1;
    }

    /*
     * Cast to a ENABLE_ROSPEC_RESPONSE message.
     */
    pRsp = (CDISABLE_ROSPEC_RESPONSE *) pRspMsg;

    /*
     * Check the LLRPStatus parameter.
     */
    if(0 != checkLLRPStatus(pRsp->getLLRPStatus(), "disableROSpec"))
    {
        /* checkLLRPStatus already tattled */
        delete pRspMsg;
        return -1;
    }

    /*
     * Done with the response message.
     */
    delete pRspMsg;

    /*
     * Tattle progress, maybe
     */
    if(m_Verbose)
    {
        printf("INFO: ROSpec enabled\n");
    }

    /*
     * Victory.
     */
    return 0;
}


/*******************************************************************************
 **
 ** @brief  Start our ROSpec using START_ROSPEC message
 **
 ** Start the ROSpec that was added above.
 **
 ** The message we send is:
 **     <START_ROSPEC MessageID='203'>
 **       <ROSpecID>123</ROSpecID>
 **     </START_ROSPEC>
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong
 **
 *****************************************************************************/

int CRFIDBotTagsReader::startROSpec (void)
{
    CSTART_ROSPEC *             pCmd;
    CMessage *                  pRspMsg;
    CSTART_ROSPEC_RESPONSE *    pRsp;

    /*
     * Compose the command message
     */
    pCmd = new CSTART_ROSPEC();
    pCmd->setMessageID(203);
    pCmd->setROSpecID(123);

    /*
     * Send the message, expect the response of certain type
     */
    pRspMsg = transact(pCmd);

    /*
     * Done with the command message
     */
    delete pCmd;

    /*
     * transact() returns NULL if something went wrong.
     */
    if(NULL == pRspMsg)
    {
        /* transact already tattled */
        return -1;
    }

    /*
     * Cast to a START_ROSPEC_RESPONSE message.
     */
    pRsp = (CSTART_ROSPEC_RESPONSE *) pRspMsg;

    /*
     * Check the LLRPStatus parameter.
     */
    if(0 != checkLLRPStatus(pRsp->getLLRPStatus(), "startROSpec"))
    {
        /* checkLLRPStatus already tattled */
        delete pRspMsg;
        return -1;
    }

    /*
     * Done with the response message.
     */
    delete pRspMsg;

    /*
     * Tattle progress
     */
    if(m_Verbose)
    {
        printf("INFO: ROSpec started\n");
    }

    /*
     * Victory.
     */
    return 0;
}


/*******************************************************************************
 **
 ** @brief  stop  ROSpec using STOP_ROSPEC message
 **
 ** STOP the ROSpec that was added above.
 **
 ** The message we send is:
 **     <STOP_ROSPEC MessageID='204'>
 **       <ROSpecID>123</ROSpecID>
 **     </START_ROSPEC>
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong
 **
 *****************************************************************************/

int CRFIDBotTagsReader::stopROSpec (void)
{
    CSTOP_ROSPEC *pCmd;
    CMessage *pRspMsg;
    CSTOP_ROSPEC_RESPONSE *pRsp;

    /*
     * Compose the command message
     */
    pCmd = new CSTOP_ROSPEC();
    pCmd->setMessageID(203);
    pCmd->setROSpecID(123);

    /*
     * Send the message, expect the response of certain type
     */
    pRspMsg = transact(pCmd);

    /*
     * Done with the command message
     */
    delete pCmd;

    /*
     * transact() returns NULL if something went wrong.
     */
    if(NULL == pRspMsg)
    {
        /* transact already tattled */
        return -1;
    }

    /*
     * Cast to a START_ROSPEC_RESPONSE message.
     */
    pRsp = (CSTOP_ROSPEC_RESPONSE *) pRspMsg;

    /*
     * Check the LLRPStatus parameter.
     */
    if(0 != checkLLRPStatus(pRsp->getLLRPStatus(), "stopROSpec"))
    {
        /* checkLLRPStatus already tattled */
        delete pRspMsg;
        return -1;
    }

    /*
     * Done with the response message.
     */
    delete pRspMsg;

    /*
     * Tattle progress
     */
    if(m_Verbose)
    {
        printf("INFO: ROSpec started\n");
    }

    /*
     * Victory.
     */
    return 0;
}



/**
 *****************************************************************************
 **
 ** @brief  Receive and print the RO_ACCESS_REPORT
 **
 ** Receive messages until an RO_ACCESS_REPORT is received.
 ** Time limit is 7 seconds. We expect a report within 5 seconds.
 **
 ** This shows how to determine the type of a received message.
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong
 **
 *****************************************************************************/

int CRFIDBotTagsReader::awaitAndPrintReport (void)
{
    int bDone = 0;
    int retVal = 0;

    /*
     * Keep receiving messages until done or until
     * something bad happens.
     */
    while(!bDone)
    {
        CMessage *              pMessage;
        const CTypeDescriptor * pType;

        if(m_isPause)
        {
            break;
        }

        /*
         * Wait up to 7 seconds for a message. The report
         * should occur within 5 seconds. 
         */
        pMessage = recvMessage(7000);
        if(NULL == pMessage)
        {
            /*
             * Did not receive a message within a reasonable
             * amount of time. recvMessage() already tattled
             */
            retVal = -2;
            bDone = 1;
            continue;
        }

        /*
         * What happens depends on what kind of message
         * received. Use the type label (m_pType) to
         * discriminate message types.
         */
        pType = pMessage->m_pType;

        /*
         * Is it a tag report? If so, print it out.
         */
        if(&CRO_ACCESS_REPORT::s_typeDescriptor == pType)
        {
            CRO_ACCESS_REPORT * pNtf;

            pNtf = (CRO_ACCESS_REPORT *) pMessage;

            printTagReportData(pNtf);
            bDone = 1;
            retVal = 0;
        }

        /*
         * Is it a reader event? This example only recognizes
         * AntennaEvents.
         */
        else if(&CREADER_EVENT_NOTIFICATION::s_typeDescriptor == pType)
        {
            CREADER_EVENT_NOTIFICATION *pNtf;
            CReaderEventNotificationData *pNtfData;

            pNtf = (CREADER_EVENT_NOTIFICATION *) pMessage;

            pNtfData = pNtf->getReaderEventNotificationData();
            if(NULL != pNtfData)
            {
                handleReaderEventNotification(pNtfData);
            }
            else
            {
                /*
                 * This should never happen. Using continue
                 * to keep indent depth down.
                 */
                printf("WARNING: READER_EVENT_NOTIFICATION without data\n");
            }
        }

        /*
         * Hmmm. Something unexpected. Just tattle and keep going.
         */
        else
        {
            printf("WARNING: Ignored unexpected message during monitor: %s\n",
                pType->m_pName);
        }

        /*
         * Done with the received message
         */
        delete pMessage;
    }

    return retVal;
}


/**
 *****************************************************************************
 **
 ** @brief  Helper routine to print a tag report
 **
 ** The report is printed in list order, which is arbitrary.
 **
 ** TODO: It would be cool to sort the list by EPC and antenna,
 **       then print it.
 **
 ** @return     void
 **
 *****************************************************************************/

void
CRFIDBotTagsReader::printTagReportData (
  CRO_ACCESS_REPORT *           pRO_ACCESS_REPORT)
{
    std::list<CTagReportData *>::iterator Cur;
    unsigned int                nEntry = 0;

    /*
     * Loop through and count the number of entries
     */
    for(
        Cur = pRO_ACCESS_REPORT->beginTagReportData();
        Cur != pRO_ACCESS_REPORT->endTagReportData();
        Cur++)
    {
        nEntry++;
    }

    //printf("INFO: %u tag report entries\n", nEntry);

    /*
     * Loop through again and print each entry.
     */
    for(
        Cur = pRO_ACCESS_REPORT->beginTagReportData();
        Cur != pRO_ACCESS_REPORT->endTagReportData();
        Cur++)
    {
        printOneTagReportData(*Cur);
    }
}


/*******************************************************************************
 **
 ** @brief  Helper routine to print one tag report into a rfid_tags msg
 **
 ** @return     void
 **
 *****************************************************************************/

void CRFIDBotTagsReader::printOneTagReportData (CTagReportData *pTagReportData)
{
    const CTypeDescriptor * pType;
    int antennaId;
    char  aBuf[64];

    /*
     * Print the EPC. It could be an 96-bit EPC_96 parameter
     * or an variable length EPCData parameter.
     */

    CParameter *pEPCParameter = pTagReportData->getEPCParameter();
    auto tmpMsg = std::make_shared<rfidbot_tags_interfaces::msg::TagReader>();

    if(NULL != pEPCParameter)
    {
        char *p = aBuf;
        llrp_u96_t my_u96;
        llrp_u1v_t my_u1v;
        llrp_u8_t *pValue = NULL;
        unsigned int n, i;

        pType = pEPCParameter->m_pType;
        if(&CEPC_96::s_typeDescriptor == pType)
        {
            CEPC_96 *pEPC_96;

            pEPC_96 = (CEPC_96 *) pEPCParameter;
            my_u96 = pEPC_96->getEPC();
            pValue = my_u96.m_aValue;
            n = 12u;
        }
        else if(&CEPCData::s_typeDescriptor == pType)
        {
            CEPCData *pEPCData;

            pEPCData = (CEPCData *) pEPCParameter;
            my_u1v = pEPCData->getEPC();
            pValue = my_u1v.m_pValue;
            n = (my_u1v.m_nBit + 7u) / 8u;
        }

        if(NULL != pValue)
        {
            for(i = 0; i < n; i++)
            {
                /*if(0 < i && i%2 == 0)
                {
                    *p++ = '-';
                }*/
                sprintf(p, "%02X", pValue[i]);
                while(*p) p++;
            }
        }
        else
        {
            strcpy(aBuf, "---unknown-epc-data-type---");
        }
    }
    else
    {
        strcpy(aBuf, "---missing-epc-data---");
    }
    //printf("%-32s", aBuf);
    tmpMsg->epc = std::string(aBuf);

    /* Print the antenna ID number */
    if(NULL != pTagReportData->getAntennaID())
    {
        antennaId = pTagReportData->getAntennaID()->getAntennaID();
        sprintf(aBuf, "%d",antennaId);
        tmpMsg->antenna_id = std::string(aBuf);
        //printf("the ant id %d",pTagReportData->getAntennaID()->getAntennaID());
    }
    else
    {
        tmpMsg->antenna_id = std::string("NA");
    }

    /*
     * End of line
     */
    //printf("\n");
    //publish the tagmsg
    updateTagReaderMsgHeader(tmpMsg,antennaId);
    rfid_tags_pub_->publish(*tmpMsg);
}


/*******************************************************************************
 ** funtion name: updateTagReaderMsgHeader
 ** description: update the message header for "rfid_tags"
 **
 ** @return     void
 **
 *****************************************************************************/
void CRFIDBotTagsReader::updateTagReaderMsgHeader(
        const rfidbot_tags_interfaces::msg::TagReader::SharedPtr rfid_msg,
int iAntennaID)
{
    rfid_msg->header.stamp = node_->get_clock()->now();

    switch (iAntennaID)
    {
        case 1:
            rfid_msg->header.frame_id =  "tag_antenna_1_link";
            break;
        case 2:
            rfid_msg->header.frame_id =  "tag_antenna_2_link";
            break;
        case 3:
            rfid_msg->header.frame_id =  "tag_antenna_3_link";
            break;
        case 4:
            rfid_msg->header.frame_id =  "tag_antenna_4_link";
            break;
        case 5:
            rfid_msg->header.frame_id =  "tag_antenna_5_link";
            break;
        case 6:
            rfid_msg->header.frame_id =  "tag_antenna_6_link";
            break;
        case 7:
            rfid_msg->header.frame_id =  "tag_antenna_7_link";
            break;
        case 8:
            rfid_msg->header.frame_id =  "tag_antenna_8_link";
            break;
        default:
            rfid_msg->header.frame_id =  "unknow";
            break;      
    }

}


/**
 *****************************************************************************
 **
 ** @brief  Handle a ReaderEventNotification
 **
 ** Handle the payload of a READER_EVENT_NOTIFICATION message.
 ** This routine simply dispatches to handlers of specific
 ** event types.
 **
 ** @return     void
 **
 *****************************************************************************/

void
CRFIDBotTagsReader::handleReaderEventNotification (
  CReaderEventNotificationData *pNtfData)
{
    CAntennaEvent *             pAntennaEvent;
    CReaderExceptionEvent *     pReaderExceptionEvent;
    int                         nReported = 0;

    pAntennaEvent = pNtfData->getAntennaEvent();
    if(NULL != pAntennaEvent)
    {
        handleAntennaEvent(pAntennaEvent);
        nReported++;
    }

    pReaderExceptionEvent = pNtfData->getReaderExceptionEvent();
    if(NULL != pReaderExceptionEvent)
    {
        handleReaderExceptionEvent(pReaderExceptionEvent);
        nReported++;
    }

    /*
     * Similarly handle other events here:
     *      HoppingEvent
     *      GPIEvent
     *      ROSpecEvent
     *      ReportBufferLevelWarningEvent
     *      ReportBufferOverflowErrorEvent
     *      RFSurveyEvent
     *      AISpecEvent
     *      ConnectionAttemptEvent
     *      ConnectionCloseEvent
     *      Custom
     */

    if(0 == nReported)
    {
        printf("NOTICE: Unexpected (unhandled) ReaderEvent\n");
    }
}


/**
 *****************************************************************************
 **
 ** @brief  Handle an AntennaEvent
 **
 ** An antenna was disconnected or (re)connected. Tattle.
 **
 ** @return     void
 **
 *****************************************************************************/

void
CRFIDBotTagsReader::handleAntennaEvent (
  CAntennaEvent *               pAntennaEvent)
{
    EAntennaEventType           eEventType;
    llrp_u16_t                  AntennaID;
    const char *                      pStateStr;

    eEventType = pAntennaEvent->getEventType();
    AntennaID = pAntennaEvent->getAntennaID();

    switch(eEventType)
    {
    case AntennaEventType_Antenna_Disconnected:
        pStateStr = "disconnected";
        break;

    case AntennaEventType_Antenna_Connected:
        pStateStr = "connected";
        break;

    default:
        pStateStr = "?unknown-event?";
        break;
    }

    printf("NOTICE: Antenna %d is %s\n", AntennaID, pStateStr);
}


/**
 *****************************************************************************
 **
 ** @brief  Handle a ReaderExceptionEvent
 **
 ** Something has gone wrong. There are lots of details but
 ** all this does is print the message, if one.
 **
 ** @return     void
 **
 *****************************************************************************/

void
CRFIDBotTagsReader::handleReaderExceptionEvent (
  CReaderExceptionEvent *       pReaderExceptionEvent)
{
    llrp_utf8v_t                Message;

    Message = pReaderExceptionEvent->getMessage();

    if(0 < Message.m_nValue && NULL != Message.m_pValue)
    {
        printf("NOTICE: ReaderException '%.*s'\n",
             Message.m_nValue, Message.m_pValue);
    }
    else
    {
        printf("NOTICE: ReaderException but no message\n");
    }
}


/**
 *****************************************************************************
 **
 ** @brief  Helper routine to check an LLRPStatus parameter
 **         and tattle on errors
 **
 ** Helper routine to interpret the LLRPStatus subparameter
 ** that is in all responses. It tattles on an error, if one,
 ** and tries to safely provide details.
 **
 ** This simplifies the code, above, for common check/tattle
 ** sequences.
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong, already tattled
 **
 *****************************************************************************/

int
CRFIDBotTagsReader::checkLLRPStatus(
    CLLRPStatus *pLLRPStatus,
    const char *pWhatStr)
{
    /*
     * The LLRPStatus parameter is mandatory in all responses.
     * If it is missing there should have been a decode error.
     * This just makes sure (remember, this program is a
     * diagnostic and suppose to catch LTKC mistakes).
     */
    if(NULL == pLLRPStatus)
    {
        printf("ERROR: %s missing LLRP status\n", pWhatStr);
        return -1;
    }

    /*
     * Make sure the status is M_Success.
     * If it isn't, print the error string if one.
     * This does not try to pretty-print the status
     * code. To get that, run this program with -vv
     * and examine the XML output.
     */
    if(StatusCode_M_Success != pLLRPStatus->getStatusCode())
    {
        llrp_utf8v_t            ErrorDesc;

        ErrorDesc = pLLRPStatus->getErrorDescription();

        if(0 == ErrorDesc.m_nValue)
        {
            printf("ERROR: %s failed, no error description given\n",
                pWhatStr);
        }
        else
        {
            printf("ERROR: %s failed, %.*s\n",
                pWhatStr, ErrorDesc.m_nValue, ErrorDesc.m_pValue);
        }
        return -2;
    }

    /*
     * Victory. Everything is fine.
     */
    return 0;
}


/**
 *****************************************************************************
 **
 ** @brief  Wrapper routine to do an LLRP transaction
 **
 ** Wrapper to transact a request/resposne.
 **     - Print the outbound message in XML if verbose level is at least 2
 **     - Send it using the LLRP_Conn_transact()
 **     - LLRP_Conn_transact() receives the response or recognizes an error
 **     - Tattle on errors, if any
 **     - Print the received message in XML if verbose level is at least 2
 **     - If the response is ERROR_MESSAGE, the request was sufficiently
 **       misunderstood that the reader could not send a proper reply.
 **       Deem this an error, free the message.
 **
 ** The message returned resides in allocated memory. It is the
 ** caller's obligtation to free it.
 **
 ** @return     ==NULL          Something went wrong, already tattled
 **             !=NULL          Pointer to a message
 **
 *****************************************************************************/

CMessage *
CRFIDBotTagsReader::transact (
  CMessage *                    pSendMsg)
{
    CConnection *               pConn = m_pConnectionToReader;
    CMessage *                  pRspMsg;

    /*
     * Print the XML text for the outbound message if
     * verbosity is 2 or higher.
     */
    if(1 < m_Verbose)
    {
        printf("\n===================================\n");
        printf("INFO: Transact sending\n");
        printXMLMessage(pSendMsg);
    }

    /*
     * Send the message, expect the response of certain type.
     * If LLRP::CConnection::transact() returns NULL then there was
     * an error. In that case we try to print the error details.
     */
    pRspMsg = pConn->transact(pSendMsg, 5000);

    if(NULL == pRspMsg)
    {
        const CErrorDetails *   pError = pConn->getTransactError();

        printf("ERROR: %s transact failed, %s\n",
            pSendMsg->m_pType->m_pName,
            pError->m_pWhatStr ? pError->m_pWhatStr : "no reason given");

        if(NULL != pError->m_pRefType)
        {
            printf("ERROR: ... reference type %s\n",
                pError->m_pRefType->m_pName);
        }

        if(NULL != pError->m_pRefField)
        {
            printf("ERROR: ... reference field %s\n",
                pError->m_pRefField->m_pName);
        }

        return NULL;
    }

    /*
     * Print the XML text for the inbound message if
     * verbosity is 2 or higher.
     */
    if(1 < m_Verbose)
    {
        printf("\n- - - - - - - - - - - - - - - - - -\n");
        printf("INFO: Transact received response\n");
        printXMLMessage(pRspMsg);
    }

    /*
     * If it is an ERROR_MESSAGE (response from reader
     * when it can't understand the request), tattle
     * and declare defeat.
     */
    if(&CERROR_MESSAGE::s_typeDescriptor == pRspMsg->m_pType)
    {
        const CTypeDescriptor * pResponseType;

        pResponseType = pSendMsg->m_pType->m_pResponseType;

        printf("ERROR: Received ERROR_MESSAGE instead of %s\n",
            pResponseType->m_pName);
        delete pRspMsg;
        pRspMsg = NULL;
    }

    return pRspMsg;
}


/**
 *****************************************************************************
 **
 ** @brief  Wrapper routine to receive a message
 **
 ** This can receive notifications as well as responses.
 **     - Recv a message using the LLRP_Conn_recvMessage()
 **     - Tattle on errors, if any
 **     - Print the message in XML if verbose level is at least 2
 **
 ** The message returned resides in allocated memory. It is the
 ** caller's obligtation to free it.
 **
 ** @param[in]  nMaxMS          -1 => block indefinitely
 **                              0 => just peek at input queue and
 **                                   socket queue, return immediately
 **                                   no matter what
 **                             >0 => ms to await complete frame
 **
 ** @return     ==NULL          Something went wrong, already tattled
 **             !=NULL          Pointer to a message
 **
 *****************************************************************************/

CMessage *
CRFIDBotTagsReader::recvMessage (int nMaxMS)
{
    CConnection *               pConn = m_pConnectionToReader;
    CMessage *                  pMessage;

    /*
    if(m_isPause)
    {
        ROS_WARN("the reader is pause!");
        return NULL;
    }*/
    /*
     * Receive the message subject to a time limit
     */
    pMessage = pConn->recvMessage(nMaxMS);

    /*
     * If LLRP::CConnection::recvMessage() returns NULL then there was
     * an error. In that case we try to print the error details.
     */
    if(NULL == pMessage)
    {
        const CErrorDetails *   pError = pConn->getRecvError();

        printf("ERROR: recvMessage failed, %s\n",
            pError->m_pWhatStr ? pError->m_pWhatStr : "no reason given");

        if(NULL != pError->m_pRefType)
        {
            printf("ERROR: ... reference type %s\n",
                pError->m_pRefType->m_pName);
        }

        if(NULL != pError->m_pRefField)
        {
            printf("ERROR: ... reference field %s\n",
                pError->m_pRefField->m_pName);
        }

        return NULL;
    }

    /*
     * Print the XML text for the inbound message if
     * verbosity is 2 or higher.
     */
    if(1 < m_Verbose)
    {
        printf("\n===================================\n");
        printf("INFO: Message received\n");
        printXMLMessage(pMessage);
    }

    return pMessage;
}


/**
 *****************************************************************************
 **
 ** @brief  Wrapper routine to send a message
 **
 ** Wrapper to send a message.
 **     - Print the message in XML if verbose level is at least 2
 **     - Send it using the LLRP_Conn_sendMessage()
 **     - Tattle on errors, if any
 **
 ** @param[in]  pSendMsg        Pointer to message to send
 **
 ** @return     ==0             Everything OK
 **             !=0             Something went wrong, already tattled
 **
 *****************************************************************************/

int
CRFIDBotTagsReader::sendMessage (
  CMessage *                    pSendMsg)
{
    CConnection *               pConn = m_pConnectionToReader;

    /*
     * Print the XML text for the outbound message if
     * verbosity is 2 or higher.
     */
    if(1 < m_Verbose)
    {
        printf("\n===================================\n");
        printf("INFO: Sending\n");
        printXMLMessage(pSendMsg);
    }

    /*
     * If LLRP::CConnection::sendMessage() returns other than RC_OK
     * then there was an error. In that case we try to print
     * the error details.
     */
    if(RC_OK != pConn->sendMessage(pSendMsg))
    {
        const CErrorDetails *   pError = pConn->getSendError();

        printf("ERROR: %s sendMessage failed, %s\n",
            pSendMsg->m_pType->m_pName,
            pError->m_pWhatStr ? pError->m_pWhatStr : "no reason given");

        if(NULL != pError->m_pRefType)
        {
            printf("ERROR: ... reference type %s\n",
                pError->m_pRefType->m_pName);
        }

        if(NULL != pError->m_pRefField)
        {
            printf("ERROR: ... reference field %s\n",
                pError->m_pRefField->m_pName);
        }

        return -1;
    }

    /*
     * Victory
     */
    return 0;
}


/**
 *****************************************************************************
 **
 ** @brief  Helper to print a message as XML text
 **
 ** Print a LLRP message as XML text
 **
 ** @param[in]  pMessage        Pointer to message to print
 **
 ** @return     void
 **
 *****************************************************************************/

void
CRFIDBotTagsReader::printXMLMessage (
  CMessage *                    pMessage)
{
    char                        aBuf[100*1024];

    /*
     * Convert the message to an XML string.
     * This fills the buffer with either the XML string
     * or an error message. The return value could
     * be checked.
     */

    pMessage->toXMLString(aBuf, sizeof aBuf);

    /*
     * Print the XML Text to the standard output.
     */
    printf("%s", aBuf);
}

void CRFIDBotTagsReader::deletePosPuaseInventoryCallBack(
        const std_msgs::msg::Bool::SharedPtr msg )
{
    if(msg->data)
    {
        RCLCPP_WARN(node_->get_logger(), "receive msg: delete pos and puase inventory! ");
        //deletePosPuaseInventory();
        m_isPause = true;
        m_isStateChange = true;
        RCLCPP_WARN(node_->get_logger(), "done: delete pos and puase inventory! ");
        
    }

}

void CRFIDBotTagsReader::deletePosPuaseInventory(void)
{   
    int iRect = 0; 
    //TODO: add the function
    if (NULL == m_pConnectionToReader)
    {
        RCLCPP_WARN(node_->get_logger(), "m_pConnectionToReader is NULL! "); 
        return;
    }
    
    //m_isPause = true;
    //RCLCPP_WARN("set m_ispause! ");
    
    RCLCPP_WARN(node_->get_logger(), "delete pos and puase inventory! ");
   
    iRect = disableROSpec();
    if(0 != iRect)
    {
        RCLCPP_WARN(node_->get_logger(), "disable ROSpec failed! return %d ",iRect);
        return;
    }

    iRect = deleteAllROSpecs();
    if(0 != iRect)
    {
        RCLCPP_WARN(node_->get_logger(), "delete all ROSpec failed! return %d ",iRect);
        return;
    }
    /*
    iRect = stopROSpec();
    if(0 != iRect)
    {
        RCLCPP_WARN("stopROSpec failed! return %d ",iRect);
    }
    
    iRect = scrubConfiguration();
    if(0 != iRect)
    {
        RCLCPP_WARN("scrubConfiguration failed! return %d ",iRect);
    }
    
    
    iRect = m_pConnectionToReader->closeConnectionToReader();
    if(0 != iRect)
    {
        RCLCPP_WARN("closeConnectionToReader failed! return %d ",iRect);
    }
    */
   
    
    //RCLCPP_WARN("closeConnectionToReader! ");
    //m_pConnectionToReader->openConnectionToReader(m_ReaderHost);
    //RCLCPP_WARN("openConnectionToReader! ");
    //scrubConfiguration();
    //RCLCPP_WARN("scrubConfiguration! ");

    //m_isPause = true;
    //RCLCPP_WARN("set m_ispause! ");

}

void CRFIDBotTagsReader::resumeInventorywithNewPosCallBack(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data)
    {
        RCLCPP_WARN(node_->get_logger(), "receive msg: resume inventory! ");  
        //resumeInventorywithNewPos();
        m_isPauseRecovery = true;
        m_isPause = false;
        m_isStateChange = true;
    }
    
}

void CRFIDBotTagsReader::resumeInventorywithNewPos(void)
{
    //TODO: add the function
    //m_pConnectionToReader->closeConnectionToReader(); 
    //m_pConnectionToReader->openConnectionToReader(m_ReaderHost);
    //checkConnectionStatus();
    //scrubConfiguration();
    addROSpec();
    enableROSpec();
    startROSpec();
    
}

void CRFIDBotTagsReader::setTagFilterCallBack(
    const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node_->get_logger(), "receive msg: set filter! "); 
    setTagFilter((std::string)msg->data);
}

void CRFIDBotTagsReader::setTagFilter(std::string sTagFilter)
{
    m_sTagFilter = sTagFilter;
    RCLCPP_INFO(node_->get_logger(), "set filter %s",(char*)m_sTagFilter.c_str());
}


void CRFIDBotTagsReader::setTransmitPowerCallBack(
    const std_msgs::msg::Int16::SharedPtr msg)
{
    RCLCPP_WARN(node_->get_logger(), "receive msg: set tx power! "); 
    setTransmitPower((unsigned int)msg->data);
}

void CRFIDBotTagsReader::setTransmitPower(unsigned int uTransPower)
{
    m_uTransmitPower = uTransPower;
    RCLCPP_INFO(node_->get_logger(), "set tx power to %d",m_uTransmitPower);
}

void CRFIDBotTagsReader::setActiveAntennaeCallBack(
    const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(node_->get_logger(), "receive msg: set antennae! "); 
    setActiveAntennae((unsigned int)msg->data);
}

void CRFIDBotTagsReader::setActiveAntennae(unsigned int uAntenae)
{
    m_uAntennaBits = uAntenae;
    RCLCPP_INFO(node_->get_logger(), "set antennae to %x",m_uAntennaBits);
}


void CRFIDBotTagsReader::setSession(unsigned int uSession)
{
    m_uSession = uSession;
    RCLCPP_INFO(node_->get_logger(), "set session to %d",m_uSession);
}


