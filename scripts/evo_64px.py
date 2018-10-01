#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import time
import serial
import crcmod.predefined
import threading

import cv2
from cv_bridge import CvBridge
import rospy

from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

from teraranger.cfg import Evo_64pxConfig
from dynamic_reconfigure.server import Server


class Evo_64px(object):

    def __init__(self):
        # ROS initialisation
        rospy.init_node("evo_64px")
        self.publisher = rospy.Publisher("teraranger_evo_64px/point_cloud", PointCloud2,
                                         queue_size=1)
        self.depth_publisher = rospy.Publisher("teraranger_evo_64px/depth_image", Image,
                                               queue_size=1)
        self.window_size = rospy.get_param("~window_size", 5)
        self.portname = rospy.get_param("~portname", "/dev/ttyACM0")
        self.baudrate = rospy.get_param("~baudrate", "115200")
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
        self.evo_64px_cfg_server = Server(Evo_64pxConfig, self.evo_64px_cfg_callback)

        # Variables used for fps estimation
        self.last_fps_timestamp = time.time()
        self.fps_window = 4
        self.fps_frame_count = 0
        self.fps = 30.0

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
        rospy.logdebug("Evo_64px parameters reconfigure request".format(**config))
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
        if config["Mode"] == Evo_64pxConfig.Evo_64px_Close_Range:
            self.send_command("\x00\x21\x01\xBC")
            rospy.loginfo("Changing mode to Close Range")
        if config["Mode"] == Evo_64pxConfig.Evo_64px_Fast:
            self.send_command("\x00\x21\x02\xB5")
            rospy.loginfo("Changing mode to Fast")

    def publish(self, msg):
        self.publisher.publish(msg)

    def compute_fps(self):
        """
        This function compute the FPS of the processing
        """
        if self.fps_window <= 0:  # Bypass fps computation
            return
        if self.fps_frame_count < self.fps_window:
            self.fps_frame_count += 1
        else:
            current_time = time.time()
            diff = current_time - self.last_fps_timestamp
            self.last_fps_timestamp = current_time

            self.fps = self.fps_frame_count / diff
            self.fps_frame_count = 1

    def get_depth_array(self):
        '''
        This function reads the data from the serial port and returns it as
        an array of 12 bit values with the shape 8x8
        '''
        got_frame = False
        while not got_frame:
            with self.serial_lock:
                frame = self.port.readline()
            if len(frame) == 269:
                if ord(frame[0]) == 0x11 and self.crc_check(frame):  # Check for range frame header and crc
                    dec_out = []
                    for i in range(1, 65):
                        rng = ord(frame[2 * i - 1]) << 7
                        rng = rng | (ord(frame[2 * i]) & 0x7F)
                        dec_out.append(rng & 0x0FFF)
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
                    print "Command not acknowledged"
                    return False
            else:
                print "Error in ACK checksum"
                return False

    def start_sensor(self):
        if self.send_command("\x00\x52\x02\x01\xDF"):
            rospy.loginfo("Sensor started successfully")

    def stop_sensor(self):
        if self.send_command("\x00\x52\x02\x00\xD8"):
            rospy.loginfo("Sensor stopped successfully")

    def run(self):
        self.port.flushInput()
        self.start_sensor()

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
            self.compute_fps()

        else:
            self.stop_sensor()
            rospy.logwarn("Node shutting down")


if __name__ == '__main__':
    evo_64px = Evo_64px()
    try:
        evo_64px.run()
    except rospy.ROSInterruptException:
        exit()
