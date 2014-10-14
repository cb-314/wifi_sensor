#!/usr/bin/env python

import rospy
import time
import numpy as np
from msg import *
import thread
import subprocess
import struct

class WifiSensor():
  def __init__(self):
    # setup rospy and get parameters
    rospy.init_node("wifisensor")
    self.adapter = rospy.get_param("~adapter", "wlan0")
    self.channel = rospy.get_param("~channel", 9)
    self.rate = rospy.get_param("~rate", 5)
    # setup wifi adapter
    subprocess.call(["ifconfig", self.adapter, "down"])
    subprocess.call(["iwconfig", self.adapter, "mode", "monitor"])
    subprocess.call(["ifconfig", self.adapter, "up"])
    subprocess.call(["iwconfig", self.adapter, "channel", str(self.channel)])
    # find your own mac addr
    output = subprocess.check_output(["ifconfig", "-a", self.adapter])
    self.my_addr = "-".join([x.lower() for x in output.split(" ")[output.split(" ").index("HWaddr")+1].split("-")[:6]])
    # Radiotap field specifications
    self.radiotap_formats = {"TSFT":"Q", "Flags":"B", "Rate":"B",
      "Channel":"HH", "FHSS":"BB", "dBm_AntSignal":"b", "dBm_AntNoise":"b",
      "Lock_Quality":"H", "TX_Attenuation":"H", "dB_TX_Attenuation":"H",
      "dBm_TX_Power":"b", "Antenna":"B",  "dB_AntSignal":"B",
      "dB_AntNoise":"B", "b14":"H", "b15":"B", "b16":"B", "b17":"B", "b18":"B",
      "b19":"BBB", "b20":"LHBB", "b21":"HBBBBBH", "b22":"B", "b23":"B",
      "b24":"B", "b25":"B", "b26":"B", "b27":"B", "b28":"B", "b29":"B",
      "b30":"B", "Ext":"B"}
    # setup shared data for threads and start
    self.data = {}
    self.dataMutex = thread.allocate_lock()
    thread.start_new_thread(self.mesRaw, ())
    # setup main loop
    self.pub = rospy.Publisher("rssi", RssiMulti, queue_size=10)
    r = rospy.Rate(self.rate)
    # main loop
    while not rospy.is_shutdown():
      data = {}
      with self.dataMutex:
        data = dict(self.data)
        self.data = {}
      msg = RssiMulti()
      msg.header.stamp = rospy.Time.now()
      for addr in data.keys():
        submsg = Rssi()
        submsg.header.stamp = rospy.Time.now()
        submsg.my_mac_addr = self.my_addr
        submsg.their_mac_addr = addr
        submsg.rssi = data[addr]
        msg.data.append(submsg)
      self.pub.publish(msg)
      r.sleep()
  def mesRaw(self):
    p = subprocess.Popen(("tcpdump", "-l", "-e", "-i", self.adapter), stdout=subprocess.PIPE)
    while not rospy.is_shutdown():
      try:
        for line in iter(p.stdout.readline, ""):
	  chunks = line.split(" ")
          addr = None
          rssi = None
          # addr
          candidates = [[chunk, len(chunk.split(":"))] for chunk in chunks if ":" in chunk]
          candidates = [candidate[0] for candidate in candidates if candidate[1] == 7]
          candidates = [candidate for candidate in candidates if candidate[0:3] == "SA:"]
          if len(candidates) == 1:
            addr = candidates[0][3:]
          # rssi
          if "signal" in chunks:
            rssi = int(chunks[chunks.index("signal")-1][:-2])
          # store
          if addr is not None and rssi is not None:
            with self.dataMutex:
              if addr in self.data.keys():
                self.data[addr].append(rssi)
              else:
                self.data[addr] = [rssi]
      except:
        pass
