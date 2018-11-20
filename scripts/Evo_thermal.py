#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
import serial
import time
from struct import unpack
import cv2
from cv_bridge import CvBridge
from time import time
from sensor_msgs.msg import Image
import crcmod.predefined
import serial.tools.list_ports
import os

from teraranger.cfg import Evo_ThermalConfig
from dynamic_reconfigure.server import Server


class Evo_Thermal(object):
    def __init__(self):
        # ROS INIT
        rospy.init_node("evo_thermal")
        self.publisher = rospy.Publisher("evo_thermal/thermal_image", Image, queue_size=1)

        self.window_size = rospy.get_param("~window_size", 5)
        self.portname = rospy.get_param("~portname", "/dev/ttyACM0")
        self.baudrate = rospy.get_param("~baudrate", "115200")
        self.evo_thermal_frame = "evo_thermal_frame"
        self.bridge = CvBridge()

        # Init variables
        self.cmap_number = 0
        self.minima_list = []
        self.maxima_list = []
        self.ptat_list = []

        # Get colormap from files
        colormap_files = (
            "resources/colormap/dev_kit_cmap.txt",
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
        self.evo_thermal_cfg_server = Server(Evo_ThermalConfig, self.evo_thermal_callback)

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
            timeout=0.1
        )

        self.port.isOpen()
        self.crc32 = crcmod.predefined.mkPredefinedCrcFun('crc-32-mpeg')

    def get_frame(self):
        got_frame = False
        while not got_frame:
            # Polls for header
            header = self.port.read(2)
            # This prevents hanging in this function
            if len(header) < 2:
                return None
            header = unpack('H', str(header))
            if header[0] == 13:
                # Header received, now read rest of frame
                data = self.port.read(2068)
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
                    print "Bad CRC"
                    return None
            # This prevents hanging in this function
            else:
                return None

        self.port.flushInput()

        return data

    def convert_frame(self, array):
        # Data is sent in dK, need to convert to celsius
        return (array / self.scaling) - self.celsius_offset

    def auto_scaling(self, data):
        # Get min/max/ptat for averaging
        frame_min, frame_max = data.min(), data.max()
        self.minima_list.append(frame_min)
        self.maxima_list.append(frame_max)
        self.ptat_list.append(self.ptat)

        # Need at least 10 frames for better average
        if len(self.maxima_list) >= 10:
            avg_max = sum(self.maxima_list) / len(self.maxima_list)
            avg_min = sum(self.minima_list) / len(self.minima_list)
            avg_ptat = sum(self.ptat_list) / len(self.ptat_list)
            # Delete oldest insertions
            self.ptat_list.pop(0)
            self.maxima_list.pop(0)
            self.minima_list.pop(0)
        else:
            # Until list fills, use current frame min/max/ptat
            avg_max = frame_max
            avg_min = frame_min
            avg_ptat = self.ptat

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
            frame = cv2.GaussianBlur(frame, (9, 9), 9)

        # Resize the frame
        frame = cv2.resize(frame, (512, 512), interpolation=cv2.INTER_NEAREST)

        # Mirror frame if necessary
        if self.thermal_image_flip_h:
            frame = cv2.flip(frame, 0)  # flip the image horizontally
        if self.thermal_image_flip_v:
            frame = cv2.flip(frame, 1)  # flip the image vertically

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
        rospy.logdebug("Evo_Thermal parameters reconfigure request".format(**config))
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
        if config["Map"] == Evo_ThermalConfig.Evo_Thermal_Dev:
            self.selected_cmap = self.cmap_list[0]
            rospy.loginfo("Change colormap to Dev format")
        elif config["Map"] == Evo_ThermalConfig.Evo_Thermal_Ice:
            self.selected_cmap = self.cmap_list[1]
            rospy.loginfo("Change colormap to Ice format")
        elif config["Map"] == Evo_ThermalConfig.Evo_Thermal_Ironbow:
            self.selected_cmap = self.cmap_list[2]
            rospy.loginfo("Change colormap to Ironbow format")
        elif config["Map"] == Evo_ThermalConfig.Evo_Thermal_High_contrast:
            self.selected_cmap = self.cmap_list[3]
            rospy.loginfo("Change colormap to High Contrast format")
        elif config["Map"] == Evo_ThermalConfig.Evo_Thermal_Whot:
            self.selected_cmap = self.cmap_list[4]
            rospy.loginfo("Change colormap to White Hot format")
        else:
            rospy.logerr("Unknown colormap index")

    def publish(self, msg):
        self.publisher.publish(msg)

    def send_command(self, command):
        self.port.write(command)
        ack = self.port.read(1)
        # This loop discards buffered frames until an ACK header is reached
        while ord(ack) != 20:
            self.port.read()
            ack = self.port.read(1)
        else:
            ack += self.port.read(3)

        # Check if ACK or NACK
        if ord(ack[2]) == 0:
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
        if not self.start_sensor():
            return

        while not rospy.is_shutdown():
            frame = self.get_frame()
            if frame is not None:
                converted_frame = self.convert_frame(frame)
                thermal_image, averages = self.auto_scaling(converted_frame)

                # Publishing thermal image
                img_msg = self.bridge.cv2_to_imgmsg(thermal_image)
                img_msg.header.stamp = rospy.Time.now()

                self.publisher.publish(img_msg)

        else:
            self.stop_sensor()
            rospy.logwarn("Node shutting down")


if __name__ == "__main__":
    evo_thermal = Evo_Thermal()
    try:
        evo_thermal.run()
    except rospy.ROSInterruptException:
        exit()
