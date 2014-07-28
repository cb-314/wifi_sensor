#!/usr/bin/env python

import rospy
import time
import scapy.all as sca
import numpy as np
from msg import Rssi

class WifiSensor():
  def __init__(self):
    self.pub = rospy.Publisher("rssi_mean", Rssi, queue_size=10)
    while not rospy.is_shutdown():
      rssi = self.mesRaw()
      msg = Rssi()
      msg.rssi = rssi
      msg.variance = 0.0
      self.pub.publish(msg)
      time.sleep(0.1)
  def parsePacket(self, pkt):
    if pkt.haslayer(sca.Dot11) :
      if pkt.addr2 != None:
        return pkt.addr2, -(256-ord(pkt.notdecoded[-4:-3]))
    return None, None
  def mesRaw(self, n=10):
    data = []
    while(len(data) < n):
      packets = sca.sniff(iface="wlan0", count = n/5)
      for pkt in packets:
        addr, rssi = self.parsePacket(pkt)
        # eduroam
        if addr == "20:b3:99:1b:f7:58":
          data.append(rssi)
    return data
