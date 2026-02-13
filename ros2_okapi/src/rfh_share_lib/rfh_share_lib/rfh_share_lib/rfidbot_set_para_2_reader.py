#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Int16, Bool, String


class rfidbotReaderFilterSetter:
    def __init__(self, node, rate_ratio, rate):
        self.node = node
        self.logger = node.get_logger()

        self.rate_ratio = rate_ratio
        self.rate_hz = rate
        self.rater = self.node.create_rate(self.rate_hz)

        # Publishers (ALL must come from node)
        self.pause_Inventory_del_pos_pub = self.node.create_publisher(
            Bool, '/pause_inventory_delete_pos', 10)

        self.set_new_filter_2_reader_pub = self.node.create_publisher(
            String, '/set_tag_filter_to_reader', 10)

        self.resume_inventory_pub = self.node.create_publisher(
            Bool, '/resume_inventory_enable_new_pos', 10)

        self.set_tx_power_2_reader_pub = self.node.create_publisher(
            Int16, '/set_tx_power_to_reader', 10)

        self.set_new_antennas_2_reader_pub = self.node.create_publisher(
            String, '/set_new_antennas', 10)

    def waitForaSecond(self):
        """Wait for roughly 1 second (or rate cycles)."""
        count = 0
        while rclpy.ok():
            self.rater.sleep()
            count += 1
            if count >= self.rate_hz:
                break

    # -------------------------------------------------------------------------------------------------
    # Core functions
    # -------------------------------------------------------------------------------------------------

    def setANewFilter2RestartRos(self, tagEPC):
        self.pauseInven2DelRos()
        self.waitForaSecond()
        self.setANewFilter(tagEPC)
        self.resumeInvenWithNewRos()

    def setTxPower(self, txPower):
        msg = Int16()
        msg.data = txPower
        self.set_tx_power_2_reader_pub.publish(msg)
        self.logger.warn(f"Set new TX power: {txPower}")

    def setANewFilter(self, tagEPC):
        msg = String()
        msg.data = tagEPC
        self.set_new_filter_2_reader_pub.publish(msg)
        self.logger.info(f"Set new filter: {tagEPC}")

    def setAntennaBits(self, antennas):
        antennaBits = 0
        if antennas == 0:
            return antennaBits
        for anId in antennas:
            antennaBits |= 1 << (anId - 1)
        self.logger.warn(f"The antenna bits: {antennaBits}")
        return antennaBits

    def setNewAntennas2Reader(self, newAtennas):
        antennaBits = self.setAntennaBits(newAtennas)
        msg = String()
        msg.data = str(antennaBits)
        self.set_new_antennas_2_reader_pub.publish(msg)
        self.logger.info(f"Published new antenna bits: {antennaBits}")

    # -------------------------------------------------------------------------------------------------
    # Pause/resume commands
    # -------------------------------------------------------------------------------------------------

    def pauseInven2DelRos(self):
        msg = Bool()
        msg.data = True
        self.pause_Inventory_del_pos_pub.publish(msg)
        self.logger.info("Paused inventory to delete ROS process")

    def resumeInvenWithNewRos(self):
        msg = Bool()
        msg.data = True
        self.resume_inventory_pub.publish(msg)
        self.logger.info("Resumed inventory with new ROS process")


