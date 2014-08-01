#!/usr/bin/env python

import rospy
import time
import scapy.all as sca
import numpy as np
from msg import *
import thread
import subprocess

class WifiSensor():
  def __init__(self):
    # setup orspy and get parameters
    rospy.init_node("wifisensor")
    self.adapter = rospy.get_param("~adapter", "wlan0")
    self.channel = rospy.get_param("~channel", 9)
    self.rate = rospy.get_param("~rate", 5)
    # setup wifi adapter
    subprocess.call(["ifconfig", self.adapter, "down"])
    subprocess.call(["iwconfig", self.adapter, "mode", "monitor"])
    subprocess.call(["ifconfig", self.adapter, "up"])
    subprocess.call(["iwconfig", self.adapter, "channel", str(self.channel)])
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
        submsg.macaddr = addr
        submsg.rssi = data[addr]
        msg.data.append(submsg)
      self.pub.publish(msg)
      r.sleep()
  def mesRaw(self):
    while not rospy.is_shutdown():
      packets = sca.sniff(iface=self.adapter, count = 10)
      for pkt in packets:
        addr, rssi = self.parsePacket(pkt)
        if addr is not None:
          with self.dataMutex:
            if addr in self.data.keys():
              self.data[addr].append(rssi)
            else:
              self.data[addr] = [rssi]
  def parsePacket(self, pkt):
    if pkt.haslayer(sca.Dot11):
      if pkt.addr2 is not None:
        return pkt.addr2, ord(pkt.notdecoded[-4:-3])-256
    return None, None
