#!/usr/bin/env python3
"""
* File name: rfh_tags_localization_main.py
* Description: ROS2 version of Okapi handheld RFID tag localization node.
*              Records RFID tags with related Kinect pose and estimates tag positions.
* Author: Jian Zhang
* Create date: Aug/10/2016
* Modified date: Nov/10/2025 by Justin Palm
* Version 2.0 for ROS2 Humble
"""

import traceback
import os
import sys
import datetime
from timeit import default_timer as timer

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from rfidbot_tags_interfaces.srv import TagLocalizing  # Rename srv file for ROS2 naming conventions

# Import custom modules (ROS2 absolute imports)
from rfidbot_tags_localization.data_recorders.rfh_tags_localization_fix_power_rawdata_recorder import rfhFixPowerRecoder
from rfidbot_tags_localization.localizers.rfh_3d_recursive_bayes_localizer import rfh3DRBTagLoclizer
from rfidbot_tags_localization.localizers.phase_differ_localizer import phDiffTagLoclizer
from rfidbot_tags_localization.localizers.recursive_bayes_localizer_gpu_3d import GPU3DRBTagLoclizer
from rfidbot_tags_localization.libs.rfidbot_set_tags_location_marker import setaSingle3DMarker, setaSingleMarkerText


# Localization flags
LOC_FLAG_NO_RAW_DATA = 0
LOC_FLAG_ONLY_INV = 1
LOC_FLAG_LOCALIZED = 2


class RFHTagsLocalization(Node):
    """
    Main node class for RFID tag localization using recorded data and localization algorithms.
    """

    def __init__(self):
        super().__init__("rfh_rfid_tags_localization")
        self.get_logger().info("RFH RFID Tags Localization Node started")

        # ===== Parameters =====
        self.declare_parameter("localizer_type", "3DRB")
        self.declare_parameter("map_frameid", "/map")

        self.localizer_type = self.get_parameter("localizer_type").value
        self.map_frameid = self.get_parameter("map_frameid").value

        path = sys.path[0]
        pardir = os.path.abspath(os.path.join(path, os.pardir))
        self.rfhFixPathRawDataFile = os.path.join(pardir, "rawdata", "rfh_fixpower_rawdata.txt")
        self.rfhFixRawDataPardir = os.path.join(pardir, "rawdata")

        # ===== Publishers =====
        self.all_tags_pub = self.create_publisher(MarkerArray, "all_scanned_tags", 10)
        self.all_tags_text_pub = self.create_publisher(MarkerArray, "all_scanned_tags_text", 10)

        # ===== Subscribers =====
        self.create_subscription(String, "/loc_a_tag", self.localize_a_tag_callback, 10)
        self.create_subscription(Bool, "/save_raw_data", self.save_raw_data, 10)
        self.create_subscription(String, "/read_raw_data", self.read_raw_data, 10)
        self.create_subscription(Bool, "/loc_all_tag", self.start_localize_all_tags, 10)

        # ===== Service =====
        self.create_service(TagLocalizing, "localize_a_tag", self.localize_a_tag_handler)

        # ===== Recorder and Localizer =====
        self.fixPowerRecorder = rfhFixPowerRecoder(self, 1, 10.0)
        self.fixPowerRecorder.setRawDataFileAdr(self.rfhFixPathRawDataFile)
        self.RFIDLocalizer = self.initialize_localizer()

        # ===== Flags and Timer =====
        self.localize_all_scanned_tags_flag = False
        self.create_timer(1.0, self.timer_callback)  # replaces rospy.Rate + spin loop

    # ------------------------------------------------------------------
    # INITIALIZATION
    # ------------------------------------------------------------------

    def initialize_localizer(self):
        """Initialize the correct localizer based on parameter."""
        if self.localizer_type == "3DRB":
            self.get_logger().warn("Initializing 3DRB localizer")
            return rfh3DRBTagLoclizer()
        elif self.localizer_type == "phDiff":
            self.get_logger().warn("Initializing phDiff localizer")
            return phDiffTagLoclizer()
        elif self.localizer_type == "3DRBGPU":
            self.get_logger().warn("Initializing GPU3DRB localizer")
            return GPU3DRBTagLoclizer()
        else:
            self.get_logger().warn(f"Unknown localizer '{self.localizer_type}', defaulting to 3DRB")
            return rfh3DRBTagLoclizer()

    # ------------------------------------------------------------------
    # FILE MANAGEMENT
    # ------------------------------------------------------------------

    def generate_raw_data_filename(self):
        """Generate unique filename for raw data output."""
        ymd = datetime.datetime.now()
        suffix = f"{ymd.year}_{ymd.month}_{ymd.day}_{ymd.hour}_{ymd.minute}.txt"
        return os.path.join(self.rfhFixRawDataPardir, f"rfh_fp_rawdata_{suffix}")

    # ------------------------------------------------------------------
    # SUBSCRIBER CALLBACKS
    # ------------------------------------------------------------------

    def start_localize_all_tags(self, msg: Bool):
        if msg.data:
            self.localize_all_scanned_tags_flag = True

    def save_raw_data(self, msg: Bool):
        if msg.data:
            filename = self.generate_raw_data_filename()
            self.get_logger().warn(f"Saving raw data to {filename}")
            self.fixPowerRecorder.saveRawData2File(filename)

    def read_raw_data(self, msg: String):
        if msg.data:
            full_path = os.path.join(self.rfhFixRawDataPardir, msg.data)
            self.get_logger().warn(f"Reading raw data from {full_path}")
            self.fixPowerRecorder.readRawDataFromFile(full_path)

    # ------------------------------------------------------------------
    # SERVICE CALLBACK
    # ------------------------------------------------------------------

    def localize_a_tag_handler(self, request, response=None):
        tag_epc = request.tag_epc
        self.get_logger().warn(f"Service call: localize tag {tag_epc}")
        localizing_flag, x, y, z = self.localize_a_tag(tag_epc)
        response.tag_epc = tag_epc
        response.localizing_flag = localizing_flag
        response.posx = x
        response.posy = y
        response.posz = z
        return response

    # ------------------------------------------------------------------
    # LOCALIZATION LOGIC
    # ------------------------------------------------------------------

    def localize_a_tag_callback(self, msg: String):
        tag_epc = msg.data
        marker_array = MarkerArray()

        start = timer()
        flag, x, y, z = self.localize_a_tag(tag_epc)
        dt = timer() - start

        if flag == LOC_FLAG_LOCALIZED:
            self.get_logger().info(
                f"Localization success: tag={tag_epc}, pos=({x:.3f}, {y:.3f}, {z:.3f}), time={dt:.2f}s"
            )
            marker = setaSingle3DMarker(x, y, z, "r", 0.10, self.map_frameid)
            marker_array.markers.append(marker)

            for idx, m in enumerate(marker_array.markers):
                m.id = idx
            self.all_tags_pub.publish(marker_array)
        else:
            self.get_logger().warn(f"Localization failed for {tag_epc}, flag={flag}, time={dt:.2f}s")

    def localize_a_tag(self, tag_epc):
        """Run localization for a single tag EPC."""
        localizing_flag = LOC_FLAG_NO_RAW_DATA
        raw_data = self.fixPowerRecorder.getRawDataForATag(tag_epc)

        if raw_data is None:
            self.get_logger().warn("No raw data found for tag.")
            return localizing_flag, None, None, None

        return self.RFIDLocalizer.localizesATag(tag_epc, raw_data)

    def localize_all_scanned_tags(self):
        """Localize all scanned tags and publish as markers."""
        marker_array = MarkerArray()
        text_marker_array = MarkerArray()
        self.fixPowerRecorder.getUuiqueTags()

        for idx, tag_epc in enumerate(self.fixPowerRecorder.uniqueTagsEPC):
            start = timer()
            flag, x, y, z = self.localize_a_tag(tag_epc)
            dt = timer() - start

            if flag == LOC_FLAG_LOCALIZED:
                self.get_logger().info(
                    f"[{idx+1}/{len(self.fixPowerRecorder.uniqueTagsEPC)}] Tag {tag_epc} localized at ({x:.3f}, {y:.3f}, {z:.3f}) in {dt:.2f}s"
                )
                marker = setaSingle3DMarker(x, y, z, "r", 0.10, self.map_frameid)
                text_marker = setaSingleMarkerText(x, y, z, tag_epc, self.map_frameid)
                marker_array.markers.append(marker)
                text_marker_array.markers.append(text_marker)
            else:
                self.get_logger().warn(f"Failed to localize tag {tag_epc}")

        for i, m in enumerate(marker_array.markers):
            m.id = i
        for j, t in enumerate(text_marker_array.markers):
            t.id = j

        self.all_tags_pub.publish(marker_array)
        self.all_tags_text_pub.publish(text_marker_array)
        self.get_logger().info(f"Total localized tags: {len(marker_array.markers)}")

    # ------------------------------------------------------------------
    # TIMER-BASED MAIN LOOP
    # ------------------------------------------------------------------

    def timer_callback(self):
        if self.localize_all_scanned_tags_flag:
            self.localize_all_scanned_tags()
            self.localize_all_scanned_tags_flag = False

    # ------------------------------------------------------------------
    # CLEANUP
    # ------------------------------------------------------------------

    def destroy_node(self):
        filename = self.generate_raw_data_filename()
        self.fixPowerRecorder.saveRawData2File(filename)
        self.get_logger().warn(f"Node shutdown — saved data to {filename}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RFHTagsLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
