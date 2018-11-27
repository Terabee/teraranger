#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
import serial
import cv2
import crcmod.predefined
import os
from struct import unpack

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from teraranger.cfg import EvoThermalConfig
from dynamic_reconfigure.server import Server


class EvoThermal(object):
    def __init__(self):
        # ROS INIT
        rospy.init_node("evo_thermal")
        self.rgb_publisher = rospy.Publisher("teraranger_evo_thermal/rgb_image", Image, queue_size=1)
        self.raw_publisher = rospy.Publisher("teraranger_evo_thermal/raw_temp_array", Float64MultiArray, queue_size=1)
        self.ptat_publisher = rospy.Publisher("teraranger_evo_thermal/ptat", Float64, queue_size=1)

        self.window_size = rospy.get_param("~window_size", 5)
        self.portname = rospy.get_param("~portname", "/dev/ttyACM0")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.evo_thermal_frame = "evo_thermal_frame"
        self.bridge = CvBridge()

        # Init variables
        self.cmap_number = 0
        self.minima_list = []
        self.maxima_list = []

        # Get colormap from files
        colormap_files = (
            "resources/colormap/dave.txt",
            "resources/colormap/ice.txt",
            "resources/colormap/ironbow.txt",
            "resources/colormap/high_contrast.txt",
            "resources/colormap/whot.txt",
        )
        self.cmap_list = self.load_colormap_files(colormap_files)
        # We also append opencv colormaps
        self.cmap_list.append(cv2.COLORMAP_HOT)
        self.cmap_list.append(cv2.COLORMAP_JET)
        self.selected_cmap = self.cmap_list[self.cmap_number]

        # Converter frame param
        self.scaling = 10.0
        self.celsius_offset = 273.15

        # Initialize reconfiguration server
        self.evo_thermal_cfg_server = Server(EvoThermalConfig, self.evo_thermal_callback)

        # Reconfigure parameters initialization
        self.thermal_image_flip_v = False
        self.thermal_image_flip_h = False
        self.thermal_image_interpolate = False
        self.thermal_image_autoscale = True
        self.manual_min_scaling = 20.0
        self.manual_max_scaling = 30.0

        self.port = serial.Serial(
            port=self.portname,
            baudrate=self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1.0
        )

        self.port.isOpen()
        self.crc32 = crcmod.predefined.mkPredefinedCrcFun('crc-32-mpeg')
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8')

    def get_frame(self):
        got_frame = False
        while not got_frame:
            # Polls for header
            temp = self.port.read(2)
            got_header = False
            while not got_header:
                if len(temp) < 2:  # Check for serial timeout
                    return None
                if ord(temp[0]) == 13 and ord(temp[1]) == 0:
                    got_header = True
                else:
                    temp = temp[1:]
                    temp += self.port.read(1)

            # Header received, now read rest of frame
            data = self.port.read(2068)
            if len(data) < 2068:  # Check for serial timeout
                return None

            # Calculate CRC for frame (except CRC value and header)
            calculatedCRC = self.crc32(data[:2064])
            data = unpack("H" * 1034, str(data))
            receivedCRC = (data[1032] & 0xFFFF) << 16
            receivedCRC |= data[1033] & 0xFFFF
            self.ptat = data[1024]
            data = data[:1024]
            data = np.reshape(data, (32, 32))
            # Compare calculated CRC to received CRC
            if calculatedCRC == receivedCRC:
                got_frame = True
            else:
                rospy.logwarn("Bad CRC")
                return None

        self.port.flushInput()
        return data

    def convert_frame(self, array):
        # Data is sent in dK, need to convert to celsius
        array = (array / self.scaling) - self.celsius_offset

        # Mirror frame if necessary
        if self.thermal_image_flip_h:
            array = np.flip(array, 0)  # flip the image horizontally
        if self.thermal_image_flip_v:
            array = np.flip(array, 1)  # flip the image vertically

        return array

    def auto_scaling(self, data):
        # Get min/maxfor averaging
        frame_min, frame_max = data.min(), data.max()
        self.minima_list.append(frame_min)
        self.maxima_list.append(frame_max)

        # Need at least 10 frames for better average
        if len(self.maxima_list) >= 10:
            avg_max = sum(self.maxima_list) / len(self.maxima_list)
            avg_min = sum(self.minima_list) / len(self.minima_list)
            # Delete oldest insertions
            self.maxima_list.pop(0)
            self.minima_list.pop(0)
        else:
            # Until list fills, use current frame min/max/ptat
            avg_max = frame_max
            avg_min = frame_min

        # Scale based on boolean toggled by event #
        if self.thermal_image_autoscale:
            # Auto scale based on avg min/max values for last 10 frames
            data[data <= avg_min] = avg_min
            data[data >= avg_max] = avg_max
            multiplier = 255 / (avg_max - avg_min)
            data = data - avg_min
            data = data * multiplier
        else:
            # Manual scale based on user setting
            data[data <= self.manual_min_scaling] = self.manual_min_scaling
            data[data >= self.manual_max_scaling] = self.manual_max_scaling
            multiplier = 255 / (self.manual_max_scaling - self.manual_min_scaling)
            data = data - self.manual_min_scaling
            data = data * multiplier
        frame = data.astype(np.uint8)

        # Check if interpolation is toggled on
        if self.thermal_image_interpolate:
            # If on, interpolate and blur
            frame = cv2.resize(frame, (128, 128), interpolation=cv2.INTER_LINEAR)
            #frame = cv2.GaussianBlur(frame, (9, 9), 9)

        # Resize the frame
        frame = cv2.resize(frame, (512, 512), interpolation=cv2.INTER_NEAREST)

        # Apply selected colormap and stamp FPS
        frame = cv2.applyColorMap(frame, self.selected_cmap)

        return frame

    def load_colormap_files(self, files):
        # Get colormap from files
        # Created using http://jdherman.github.io/colormap/
        colormap_list = []
        for rel_file_path in files:
            full_path = os.path.join(os.path.dirname(__file__), os.pardir, rel_file_path)
            with open(full_path, 'r') as f:
                r = []
                g = []
                b = []
                for i in range(256):
                    x, y, z = f.readline().split(',')
                    r.append(x)
                    g.append(y)
                    b.append(z.replace(";\n", ""))
            colormap = np.zeros((256, 1, 3), dtype=np.uint8)
            # We use BGR because that's default for openCV
            colormap[:, 0, 0] = b
            colormap[:, 0, 1] = g
            colormap[:, 0, 2] = r
            colormap_list.append(colormap)

        return colormap_list

    def evo_thermal_callback(self, config, level):
        rospy.logdebug("Evo Thermal parameters reconfigure request".format(**config))
        if level == -1:
            return config
        if level == 0:
            self.thermal_image_flip_h = config["thermal_image_flip_h"]
            self.thermal_image_flip_v = config["thermal_image_flip_v"]
            self.thermal_image_interpolate = config["thermal_image_interpolate"]
        elif level == 1:
            self.thermal_image_autoscale = config["thermal_image_autoscale"]
        elif level == 2:
            if config["manual_max_scaling"] <= config["manual_min_scaling"]:
                config["manual_max_scaling"] = config["manual_min_scaling"] + 1
            if config["manual_max_scaling"] <= config["manual_min_scaling"]:
                config["manual_min_scaling"] = config["manual_max_scaling"] - 1
            self.manual_min_scaling = config["manual_min_scaling"]
            self.manual_max_scaling = config["manual_max_scaling"]
        elif level == 3:
            self.reconfigure_color_map(config)
        else:
            rospy.loginfo("Invalid reconfiguration level")
        return config

    def reconfigure_color_map(self, config):
        if config["Map"] == EvoThermalConfig.EvoThermal_Dave:
            self.selected_cmap = self.cmap_list[0]
            rospy.loginfo("Changed colormap to Dave")
        elif config["Map"] == EvoThermalConfig.EvoThermal_Ice:
            self.selected_cmap = self.cmap_list[1]
            rospy.loginfo("Changed colormap to Ice")
        elif config["Map"] == EvoThermalConfig.EvoThermal_Ironbow:
            self.selected_cmap = self.cmap_list[2]
            rospy.loginfo("Changed colormap to Ironbow")
        elif config["Map"] == EvoThermalConfig.EvoThermal_High_contrast:
            self.selected_cmap = self.cmap_list[3]
            rospy.loginfo("Changed colormap to High Contrast")
        elif config["Map"] == EvoThermalConfig.EvoThermal_Whot:
            self.selected_cmap = self.cmap_list[4]
            rospy.loginfo("Changed colormap to White Hot")
        else:
            rospy.logerr("Unknown colormap index")

    def publish(self, rgb, temp_array):
        # Publish RGB image
        rgb_img_msg = self.bridge.cv2_to_imgmsg(rgb)
        rgb_img_msg.header.frame_id = self.evo_thermal_frame
        rgb_img_msg.header.stamp = rospy.Time.now()
        self.rgb_publisher.publish(rgb_img_msg)

        # Publish temperatures array
        raw_temp_array = Float64MultiArray()
        raw_temp_array.layout.dim.append(MultiArrayDimension())
        raw_temp_array.layout.dim[0].size = 32
        raw_temp_array.layout.dim[0].stride = 1
        raw_temp_array.layout.dim[0].label = "x-axis"
        raw_temp_array.layout.dim.append(MultiArrayDimension())
        raw_temp_array.layout.dim[1].size = 32
        raw_temp_array.layout.dim[1].stride = 1
        raw_temp_array.layout.dim[1].label = "y-axis"
        data = np.reshape(temp_array, (1024))
        np.round(data, 2)
        raw_temp_array.data = data
        self.raw_publisher.publish(raw_temp_array)

        # Publish internal sensor temperature
        self.ptat_publisher.publish(self.ptat/self.scaling - self.celsius_offset)

    def send_command(self, command):
        self.port.flushInput()
        self.port.write(command)

        # This loop discards buffered frames until a valid ACK frame is reached
        temp_ack = self.port.read(4)
        got_ack = False
        while not got_ack:
            if len(temp_ack) < 4:  # Check for serial timeout
                rospy.logwarn("Timeout when reading ACK")
                return False
            if ord(temp_ack[0]) == 20:
                if self.crc8(temp_ack[:3]) == ord(temp_ack[3]):
                    got_ack = True
                    break
            temp_ack = temp_ack[1:]
            temp_ack += self.port.read(1)

        # Check if ACK or NACK
        if ord(temp_ack[2]) == 0:
            return True
        else:
            rospy.logerr("Command not acknowledged")
            return False

    def start_sensor(self):
        rospy.loginfo("Starting sensor...")
        res = self.send_command("\x00\x52\x02\x01\xDF")
        if res:
            rospy.loginfo("Sensor started successfully")
            return True
        else:
            rospy.logerr("Failed to start sensor")
            return False

    def stop_sensor(self):
        rospy.loginfo("Stopping sensor...")
        res = self.send_command("\x00\x52\x02\x00\xD8")
        if res:
            rospy.loginfo("Sensor stopped successfully")
            return True
        else:
            rospy.logerr("Failed to stop sensor")
            return False

    def run(self):
        self.port.flushInput()
        if self.baudrate == 115200:  # Sending VCP start when connected via USB
            if not self.start_sensor():
                return

        while not rospy.is_shutdown():
            frame = self.get_frame()
            if frame is not None:
                temp_array = self.convert_frame(frame)
                thermal_image = self.auto_scaling(temp_array)

                self.publish(thermal_image, temp_array)
        else:
            if self.baudrate == 115200:  # Sending VCP stop when connected via USB
                self.stop_sensor()
            rospy.logwarn("Node shutting down")


if __name__ == "__main__":
    evo_thermal = EvoThermal()
    try:
        evo_thermal.run()
    except rospy.ROSInterruptException:
        exit()
