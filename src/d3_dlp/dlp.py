#!/usr/bin/env python3

########
# DLP Controller - Manages SPI interface and incoming
# DLP commands and issues the necessary commands to display the
# correct message based on dlp_command topic
#
# This class has a constructor + a single entry point: `update_dlp`
# that entry point will call the correct update method based on
# what command string was provided.
#
# The remaining functions are just arranged to make sure the
# correct video address is set, with the correct flip settings
# with the correct number of total frames.
########

import rospy
from pyftdi.spi import SpiController, SpiIOError
from d3_dlp import dlp_spi_generator as dlpgen

# Video Name Offset offset (hex) framerate (hz) framecount (int)
# ------------------------------------------------------------
# D3_Logo    0x85958     20hz        108
# Scanning   0x17DA51C   20hz        27
# Turn       0x1A7A60C   30hz        39
# Straight   0x1F861C4   30hz        45
# Go         0x245A79C   30hz        31
# Slow       0x27748AC   30hz        93
# Stop       0x345C594   30hz        18
# ------------------------------------------------------------
# datasheet: https://www.ti.com/lit/ug/dlpu100/dlpu100.pdf?ts=1665158603108&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FDLP3021-Q1
class DLPDemo:
    def __init__(self):
        self.ctrl = SpiController()
        # Configure the first interface (IF/1) of the FTDI device as a SPI master
        self.ctrl.configure('ftdi://ftdi:232h/1')

        # Get a port to a SPI slave
        self.spi = self.ctrl.get_port(0)
        self.spi.set_frequency(5E6)

    def message_dlp_go(self):
        self.spi.exchange(dlpgen.VCM_START_ADDR1(0x245A79C))
        self.spi.exchange(dlpgen.FMT_FLIP(long_flip=False, short_flip=True))
        self.spi.exchange(dlpgen.VCM_CONFIG1(0, 31))
        self.spi.exchange(dlpgen.VCM_CONTROL(True))

    def message_dlp_slow(self):
        self.spi.exchange(dlpgen.VCM_START_ADDR1(0x27748AC))
        self.spi.exchange(dlpgen.FMT_FLIP(long_flip=False, short_flip=True))
        self.spi.exchange(dlpgen.VCM_CONFIG1(0, 93))
        self.spi.exchange(dlpgen.VCM_CONTROL(True))

    def message_dlp_stop(self):
        self.spi.exchange(dlpgen.VCM_START_ADDR1(0x345C594))
        self.spi.exchange(dlpgen.FMT_FLIP(long_flip=False, short_flip=True))
        self.spi.exchange(dlpgen.VCM_CONFIG1(0, 18))
        self.spi.exchange(dlpgen.VCM_CONTROL(True))

    def message_dlp_turn(self, v_flip = False):
        self.spi.exchange(dlpgen.VCM_START_ADDR1(0x1A7A60C))
        self.spi.exchange(dlpgen.FMT_FLIP(long_flip=v_flip, short_flip=True))
        self.spi.exchange(dlpgen.VCM_CONFIG1(0, 39))
        self.spi.exchange(dlpgen.VCM_CONTROL(True))

    # Disable the video
    def disable_dlp_message(self):
        self.spi.exchange(dlpgen.VCM_CONTROL(False))

    def message_dlp_logo(self):
        self.spi.exchange(dlpgen.VCM_START_ADDR1(0x85958))
        self.spi.exchange(dlpgen.FMT_FLIP(long_flip=False, short_flip=True))
        self.spi.exchange(dlpgen.VCM_CONFIG1(0, 108))
        self.spi.exchange(dlpgen.VCM_CONTROL(True))

    def message_dlp_scan(self):
        self.spi.exchange(dlpgen.VCM_START_ADDR1(0x17DA51C))
        self.spi.exchange(dlpgen.FMT_FLIP(long_flip=False, short_flip=True))
        self.spi.exchange(dlpgen.VCM_CONFIG1(0, 27))
        self.spi.exchange(dlpgen.VCM_CONTROL(True))

    def message_dlp_straight(self, h_flip):
        self.spi.exchange(dlpgen.VCM_START_ADDR1(0x1F861C4))
        self.spi.exchange(dlpgen.FMT_FLIP(long_flip=False, short_flip=h_flip))
        self.spi.exchange(dlpgen.VCM_CONFIG1(0, 45))
        self.spi.exchange(dlpgen.VCM_CONTROL(True))

    def update_dlp(self, dlp_cmd):
        rospy.logdebug("DLP command: " + dlp_cmd)
        self.disable_dlp_message()
        rospy.sleep(0.05)
        if(dlp_cmd == 'logo'):
            self.message_dlp_logo()
        elif(dlp_cmd == 'scan'):
            self.message_dlp_scan()
        elif(dlp_cmd == 'stop'):
            self.message_dlp_stop()
        elif(dlp_cmd == 'slow'):
            self.message_dlp_slow()
        elif(dlp_cmd == 'go'):
            self.message_dlp_go()
        elif(dlp_cmd == 'forward'):
            self.message_dlp_straight(h_flip=True)
        elif(dlp_cmd == 'backward'):
            self.message_dlp_straight(h_flip=False)
        elif(dlp_cmd == 'turn_left'):
            self.message_dlp_turn(v_flip=False)
        elif(dlp_cmd == 'turn_right'):
            self.message_dlp_turn(v_flip=True)
        else:
            self.disable_dlp_message()
