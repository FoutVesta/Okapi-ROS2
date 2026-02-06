#!/usr/bin/env python3
"""
Main entry point for Okapi handheld tag localization (ROS 2 Humble).
Ports the former ROS 1 node to rclpy conventions without changing behaviour.
"""

import os
import datetime
from timeit import default_timer as timer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray

from rfidbot_tags_interfaces.srv import TagLocalizing
from rfidbot_tags_localization.data_recorders.rfh_tags_localization_fix_power_rawdata_recorder import (
    rfhFixPowerRecoder,
)
from rfidbot_tags_localization.localizers.rfh_3d_recursive_bayes_localizer import (
    rfh3DRBTagLoclizer,
)
from rfidbot_tags_localization.localizers.phase_differ_localizer import phDiffTagLoclizer
from rfidbot_tags_localization.localizers.recursive_bayes_localizer_gpu_3d import (
    GPU3DRBTagLoclizer,
)
from rfidbot_tags_localization.libs.rfidbot_set_tags_location_marker import (
    setaSingle3DMarker,
    setaSingleMarkerText,
)

# Localization flags
LOC_FLAG_NO_RAW_DATA = 0   # no raw data
LOC_FLAG_ONLY_INV = 1      # only inventory, but can not localize
LOC_FLAG_LOCALIZED = 2     # localized


class rfhTagsLocalization(Node):
    def __init__(self):
        # Keep ROS2 node name consistent with launch file and legacy ROS1 name
        super().__init__("rfh_tags_localization_main")

        # Parameters
        self.declare_parameter("localizer_type", "3DRB")
        self.declare_parameter("map_frameid", "/map")
        self.declare_parameter("spin_rate", 1.0)           # Hz for main loop
        self.declare_parameter("recorder_rate", 10.0)      # Hz for recorder waits
        self.declare_parameter("recorder_rate_ratio", 10)  # multiplier for waits

        self.localizerType = self.get_parameter("localizer_type").value
        self.mapFrameId = self.get_parameter("map_frameid").value
        self.spin_rate_hz = float(self.get_parameter("spin_rate").value)
        recorder_rate_hz = float(self.get_parameter("recorder_rate").value)
        recorder_rate_ratio = int(self.get_parameter("recorder_rate_ratio").value)

        self.get_logger().info(f"{self.get_name()} started")
        self.get_logger().info(f"Localizer type: {self.localizerType}")

        # Paths for raw data
        path = os.path.dirname(os.path.abspath(__file__))
        pardir = os.path.abspath(os.path.join(path, os.pardir))
        self.rfhFixPathRawDataFile = os.path.join(pardir, "rawdata", "rfh_fixpower_rawdata.txt")
        self.rfhFixRawDataPardir = os.path.join(pardir, "rawdata")

        # Subscriptions
        self.create_subscription(String, "/loc_a_tag", self.localizeATagCallBack, 10)
        self.create_subscription(Bool, "/save_raw_data", self.saveRawData, 10)
        self.create_subscription(String, "/read_raw_data", self.readRawData, 10)
        self.create_subscription(Bool, "/loc_all_tag", self.start2LocalizeAllTags, 10)
        self.localizeAllScannedTagsFlag = False

        # Service
        self.create_service(TagLocalizing, "localize_a_tag", self.localizeATagHandler)

        # Data recorder
        self.fixPowerRecorder = rfhFixPowerRecoder(
            self, recorder_rate_ratio, recorder_rate_hz
        )
        self.fixPowerRecorder.setRawDataFileAdr(self.rfhFixPathRawDataFile)

        # Localizer selection
        if self.localizerType == "3DRB":
            self.RFIDLocalizer = rfh3DRBTagLoclizer()
        elif self.localizerType == "phDiff":
            self.RFIDLocalizer = phDiffTagLoclizer()
        elif self.localizerType == "3DRBGPU":
            self.RFIDLocalizer = GPU3DRBTagLoclizer()
        else:
            self.get_logger().warn(
                f"Localizer '{self.localizerType}' not found, defaulting to 3DRB"
            )
            self.RFIDLocalizer = rfh3DRBTagLoclizer()

        # Publishers
        self.allTagsShower_pub = self.create_publisher(MarkerArray, "all_scanned_tags", 10)
        self.allTagsLabeler_pub = self.create_publisher(
            MarkerArray, "all_scanned_tags_text", 10
        )

        # Main loop timer (replaces rospy.Rate loop)
        period = 1.0 / self.spin_rate_hz if self.spin_rate_hz > 0 else 1.0
        self.timer = self.create_timer(period, self.spin_once)

    def generateRawDataFileName(self):
        """
        Generate a new file name for raw data based on current time up to minutes.
        """
        filenamePrefix = "rfh_fp_rawdata"
        ymd = datetime.datetime.now()
        filenameSuffix = f"_{ymd.year}_{ymd.month}_{ymd.day}_{ymd.hour}_{ymd.minute}.txt"
        rawDataName = os.path.join(self.rfhFixRawDataPardir, filenamePrefix + filenameSuffix)
        return rawDataName

    def start2LocalizeAllTags(self, msg):
        self.localizeAllScannedTagsFlag = True
        self.get_logger().warn("Received /loc_all_tag trigger; localizing all scanned tags.")

    def saveRawData(self, msg):
        if not msg.data:
            return
        fileName = self.generateRawDataFileName()
        self.get_logger().warn(f"Saving raw data to {fileName}")
        self.fixPowerRecorder.saveRawData2File(fileName)

    def readRawData(self, msg):
        fileName = msg.data
        if not fileName:
            return
        fullAdrFileName = os.path.join(self.rfhFixRawDataPardir, fileName)
        self.get_logger().warn(f"Read raw data from: {fullAdrFileName}")
        self.fixPowerRecorder.readRawDataFromFile(fullAdrFileName)

    def localizeATagHandler(self, request, response):
        tagEPC = request.tag_epc
        self.get_logger().warn(f"Receive localize a tag {tagEPC}")
        (localizingFlag, posx, posy, posz) = self.localizeAtag(tagEPC)
        response.tag_epc = tagEPC
        response.localizing_flag = int(localizingFlag)
        response.posx = float(posx) if posx is not None else 0.0
        response.posy = float(posy) if posy is not None else 0.0
        response.posz = float(posz) if posz is not None else 0.0
        return response

    def localizeATagCallBack(self, msg):
        """
        Callback for localizing a single tag by EPC; shows the tag on the map.
        """
        tagEPC = msg.data
        markerArray = MarkerArray()

        start = timer()
        (flag, posx, posy, posz) = self.localizeAtag(tagEPC)
        dt = timer() - start

        if flag == LOC_FLAG_LOCALIZED:
            self.get_logger().warn(
                f"Localization success: tag:{tagEPC}, flag:{flag}, pos({posx:.3f}, {posy:.3f}, {posz:.3f}) in {dt:.3f} s"
            )
            marker = setaSingle3DMarker(posx, posy, posz, "r", 0.10, self.mapFrameId)
            markerArray.markers.append(marker)

            # Assign unique ids
            for idx, m in enumerate(markerArray.markers):
                m.id = idx

            self.allTagsShower_pub.publish(markerArray)
        else:
            self.get_logger().warn(
                f"Localization failed: tag:{tagEPC}, flag:{flag}, in {dt:.3f} s"
            )

    def localizeAtag(self, tagEPC):
        """
        Localize a tag by given EPC.
        """
        localizingFlag = LOC_FLAG_NO_RAW_DATA
        rawData = self.fixPowerRecorder.getRawDataForATag(tagEPC)

        if rawData is None:
            self.get_logger().warn("No raw data found for the tag.")
            return (localizingFlag, None, None, None)

        (localizingFlag, posx, posy, posz) = self.RFIDLocalizer.localizesATag(tagEPC, rawData)
        return (localizingFlag, posx, posy, posz)

    def localizeAllScannedTags(self):
        """
        Localize all scanned tags and show them in the map.
        """
        markerArray = MarkerArray()
        textMarkerArray = MarkerArray()
        self.fixPowerRecorder.getUniqueTags()

        for idx, tagEpc in enumerate(self.fixPowerRecorder.uniqueTagsEPC):
            start = timer()
            (flag, posx, posy, posz) = self.localizeAtag(tagEpc)
            dt = timer() - start

            if flag == LOC_FLAG_LOCALIZED:
                self.get_logger().warn(
                    f"Localization success: {idx} in {len(self.fixPowerRecorder.uniqueTagsEPC)}, tag:{tagEpc}, flag:{flag}, pos({posx:.3f}, {posy:.3f}, {posz:.3f}) in {dt:.3f} s"
                )
                marker = setaSingle3DMarker(posx, posy, posz, "r", 0.10, self.mapFrameId)
                markerArray.markers.append(marker)

                textmarker = setaSingleMarkerText(posx, posy, posz, tagEpc, self.mapFrameId)
                textMarkerArray.markers.append(textmarker)
            else:
                self.get_logger().warn(
                    f"Localization failed: {idx} in {len(self.fixPowerRecorder.uniqueTagsEPC)}, tag:{tagEpc}, flag:{flag}, in {dt:.3f} s"
                )

        # Assign ids and publish
        for idx, m in enumerate(markerArray.markers):
            m.id = idx
        for idx, text_m in enumerate(textMarkerArray.markers):
            text_m.id = idx

        self.allTagsShower_pub.publish(markerArray)
        self.allTagsLabeler_pub.publish(textMarkerArray)

        self.get_logger().warn("=======================================================")
        self.get_logger().warn(f"===============Total {len(markerArray.markers)} tag localized=================")
        self.get_logger().warn("=======================================================")

    def spin_once(self):
        # Execute periodic tasks (replaces while not rospy.is_shutdown())
        if self.localizeAllScannedTagsFlag:
            self.localizeAllScannedTags()
            self.localizeAllScannedTagsFlag = False


def main(args=None):
    rclpy.init(args=args)
    node = rfhTagsLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        fileName = node.generateRawDataFileName()
        node.fixPowerRecorder.saveRawData2File(fileName)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
