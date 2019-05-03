#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import serial
import crcmod.predefined
import threading

import cv2
from cv_bridge import CvBridge
import rospy

from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

from teraranger.cfg import Evo64pxConfig
from dynamic_reconfigure.server import Server


class Evo64px(object):
    def __init__(self):
        # ROS initialisation
        rospy.init_node("evo_64px")
        self.publisher = rospy.Publisher("teraranger_evo_64px/point_cloud", PointCloud2,
                                         queue_size=1)
        self.depth_publisher = rospy.Publisher("teraranger_evo_64px/depth_image", Image,
                                               queue_size=1)
        self.window_size = rospy.get_param("~window_size", 5)
        self.portname = rospy.get_param("~portname", "/dev/ttyACM0")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.evo_64px_frame = "evo_64px_frame"
        self.bridge = CvBridge()

        # Initialize parameters
        self.depth_image_invert = False
        self.depth_image_interpolate = False
        self.depth_image_colormap = cv2.COLORMAP_JET
        self.max_distance = 5000.0
        self.min_distance = 100.0

        # Parameters of the camera
        self.focal_length = 30.0
        self.scaling_factor = 1000.0
        self.height = 8
        self.width = 8

        # Initialize reconfiguration server
        self.evo_64px_cfg_server = Server(Evo64pxConfig, self.evo_64px_cfg_callback)

        # Configure the serial connections (the parameters differs on the device you are connecting to)
        self.port = serial.Serial(
            port=self.portname,
            baudrate=self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.port.isOpen()
        self.crc32 = crcmod.predefined.mkPredefinedCrcFun('crc-32-mpeg')
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8')
        self.serial_lock = threading.Lock()

    def to_point_cloud(self, depth_array):
        # Initializing output structure
        header = Header()
        header.frame_id = self.evo_64px_frame
        header.stamp = rospy.Time.now()

        #  Build a Point Cloud from a depth image
        points = []
        for v in range(self.height):
            for u in range(self.width):
                z = depth_array[u][v] / self.scaling_factor
                if z <= self.min_distance / self.scaling_factor:
                    z = self.min_distance / self.scaling_factor
                x = (u - self.height / 2) * z / self.focal_length
                y = (v - self.width / 2) * z / self.focal_length

                point = np.array([[x], [y], [z]])
                points.append(point)

        # Create pcl from points
        output_cloud = point_cloud2.create_cloud_xyz32(header, points)

        return output_cloud

    def evo_64px_cfg_callback(self, config, level):
        rospy.logdebug("Evo 64px parameters reconfigure request".format(**config))
        if level == -1:
            return config
        elif level == 0:
            self.depth_image_invert = config["depth_image_invert"]
            self.depth_image_interpolate = config["depth_image_interpolate"]
        elif level == 1:
            if config["max_distance_mm"] <= config["min_distance_mm"]:
                config["max_distance_mm"] = config["min_distance_mm"] + 1
            self.max_distance = config["max_distance_mm"]
            self.min_distance = config["min_distance_mm"]
        elif level == 2:
            if config["max_distance_mm"] <= config["min_distance_mm"]:
                config["min_distance_mm"] = config["max_distance_mm"] - 1
            self.max_distance = config["max_distance_mm"]
            self.min_distance = config["min_distance_mm"]
        elif level == 3:
            self.reconfigure_mode(config)
        else:
            rospy.loginfo("Invalid reconfiguration level")
        return config

    def reconfigure_mode(self, config):
        if config["Mode"] == Evo64pxConfig.Evo64px_Close_Range:
            self.send_command("\x00\x21\x01\xBC")
            rospy.loginfo("Changing mode to Close Range")
        elif config["Mode"] == Evo64pxConfig.Evo64px_Fast:
            self.send_command("\x00\x21\x02\xB5")
            rospy.loginfo("Changing mode to Fast")
        else:
            rospy.logerr("Unknown sensor mode")

    def publish(self, msg):
        self.publisher.publish(msg)

    def get_depth_array(self):
        '''
        This function reads the data from the serial port and returns it as
        an array of 12 bit values with the shape 8x8
        '''
        got_frame = False
        while not got_frame:
            with self.serial_lock:
                frame = self.port.readline()
            if len(frame) in (269, 141):
                if ord(frame[0]) == 0x11 and self.crc_check(frame):  # Check for range frame header and crc
                    dec_out = []
                    for i in range(1, 65):
                        rng = ord(frame[2 * i - 1]) << 7
                        rng = rng | (ord(frame[2 * i]) & 0x7F)
                        dec_out.append(rng & 0x3FFF)
                    depth_array = [dec_out[i:i + 8] for i in range(0, len(dec_out), 8)]
                    depth_array = np.array(depth_array)
                    got_frame = True
            else:
                rospy.logwarn("Invalid frame length: {}".format(len(frame)))

        depth_array.astype(np.uint16)
        return depth_array

    def crc_check(self, frame):
        index = len(frame) - 9  # Start of CRC
        crc_value = (ord(frame[index]) & 0x0F) << 28
        crc_value |= (ord(frame[index + 1]) & 0x0F) << 24
        crc_value |= (ord(frame[index + 2]) & 0x0F) << 20
        crc_value |= (ord(frame[index + 3]) & 0x0F) << 16
        crc_value |= (ord(frame[index + 4]) & 0x0F) << 12
        crc_value |= (ord(frame[index + 5]) & 0x0F) << 8
        crc_value |= (ord(frame[index + 6]) & 0x0F) << 4
        crc_value |= (ord(frame[index + 7]) & 0x0F)
        crc_value = crc_value & 0xFFFFFFFF
        crc = self.crc32(frame[:index])

        if crc == crc_value:
            return True
        else:
            rospy.logwarn("Discarding current buffer because of bad CRC")
            return False

    def send_command(self, command):
        with self.serial_lock:
            self.port.write(command)
            ack = self.port.read(1)
            # This loop discards buffered frames until an ACK header is reached
            while ord(ack) != 20:
                self.port.readline()
                ack = self.port.read(1)
            else:
                ack += self.port.read(3)

            # Check ACK crc8
            crc8 = self.crc8(ack[:3])
            if crc8 == ord(ack[3]):
                # Check if ACK or NACK
                if ord(ack[2]) == 0:
                    return True
                else:
                    rospy.logerr("Command refused by device")
                    return False
            else:
                rospy.logerr("Invalid sensor answer to command")
                return False

    def start_sensor(self):
        rospy.loginfo("Configuring sensor...")
        res = self.send_command("\x00\x11\x03\x4B")
        if res:
            rospy.loginfo("Distance and ambient mode activated successfully")
        else:
            rospy.logerr("Failed to activate distance and ambient mode")

        rospy.loginfo("Starting sensor output...")
        res = self.send_command("\x00\x52\x02\x01\xDF")
        if res:
            rospy.loginfo("Sensor output started successfully")
            return True
        else:
            rospy.logerr("Failed to start sensor output")
            return False

    def stop_sensor(self):
        rospy.loginfo("Stopping sensor output ...")
        res = self.send_command("\x00\x52\x02\x00\xD8")
        if res:
            rospy.loginfo("Sensor output stopped successfully")
            return True
        else:
            rospy.logerr("Failed to stop sensor output")
            return False

    def run(self):
        self.port.flushInput()
        if self.baudrate == 115200: # Sending VCP start when connected via USB
            if not self.start_sensor():
                return

        while not rospy.is_shutdown():
            depth_array = self.get_depth_array()

            # Trimming depth_array
            out_of_range = depth_array > self.max_distance
            too_close_range = depth_array < self.min_distance
            depth_array[out_of_range] = self.max_distance
            depth_array[too_close_range] = self.min_distance

            # Creating point cloud
            point_cloud_array = np.empty((8, 8))
            point_cloud_array[:] = depth_array
            point_cloud = self.to_point_cloud(point_cloud_array)
            self.publisher.publish(point_cloud)

            # Creating and scaling depth image
            depth_image_array = np.empty((8, 8))
            depth_image_array[:] = depth_array
            depth_scale_factor = 255.0 / (self.max_distance - self.min_distance)
            depth_scale_offset = -(self.min_distance * depth_scale_factor)
            depth_image_array = depth_image_array * depth_scale_factor + depth_scale_offset
            depth_image_array = depth_image_array.astype(np.uint8)

            if not self.depth_image_invert:
                depth_image_array = 255 - depth_image_array

            # Converting image to RGB with interpolation
            rgb = cv2.cvtColor(depth_image_array, cv2.COLOR_GRAY2RGB)
            rgb = cv2.applyColorMap(rgb.astype(np.uint8), self.depth_image_colormap)
            if self.depth_image_interpolate:
                rgb = cv2.resize(rgb, (512, 512), interpolation=cv2.INTER_LINEAR)
            else:
                rgb = cv2.resize(rgb, (512, 512), interpolation=cv2.INTER_AREA)

            # Publishing depth image
            img_msg = self.bridge.cv2_to_imgmsg(rgb)
            img_msg.header.stamp = rospy.Time.now()

            self.depth_publisher.publish(img_msg)

        else:
            if self.baudrate == 115200:
                self.stop_sensor()  # Sending VCP stop when connected via USB
            rospy.logwarn("Node shutting down")


if __name__ == '__main__':
    evo_64px = Evo64px()
    try:
        evo_64px.run()
    except rospy.ROSInterruptException:
        exit()
