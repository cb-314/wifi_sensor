#!/usr/bin/env python

import rospy
import time
import scapy.all as sca
import numpy as np
from msg import Rssi
import thread

class WifiSensor():
  def __init__(self):
    # setup shared data for threads and start
    self.data = {}
    self.dataMutex = thread.allocate_lock()
    thread.start_new_thread(self.mesRaw, ())
    # setup main loop
    rospy.init_node("wifisensor")
    self.pub = rospy.Publisher("rssi", Rssi, queue_size=10)
    r = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
      data = {}
      with self.dataMutex:
        data = dict(self.data)
        self.data = {}
      for addr in data.keys():
        msg = Rssi()
        msg.header.stamp = rospy.Time.now()
        msg.macaddr = addr
        msg.rssi = data[addr]
        self.pub.publish(msg)
      r.sleep()
  def mesRaw(self):
    while not rospy.is_shutdown():
      packets = sca.sniff(iface="wlan0", count = 10)
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
