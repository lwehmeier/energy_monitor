#!/usr/bin/python
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32, Int32, Int16
import struct

class EnergyMonitor:
    def __init__(self):
        self.power = 0.0
        self.averagePower = 0.0
        self.energy = 0.0
        self.voltage = 0.0
        self.current = 0.0
        self.lastUpdate = None
    def updateCurrent(self, current):
        if self.lastUpdate is None:
            self.lastUpdate = rospy.Time.now()
        else:
            self.energy += self.voltage * (current+self.current)/2 * ( -self.lastUpdate.secs + rospy.Time.now().secs + (rospy.Time.now().nsecs - self.lastUpdate.nsecs)/1000000000.0)
            self.lastUpdate = rospy.Time.now()
        self.current = current
        self.power = self.current * self.voltage
    def updateVoltage(self, voltage):
        if self.lastUpdate is None:
            self.lastUpdate = rospy.Time.now()
        else:
            self.energy += self.current * (voltage+self.voltage)/2 * ( -self.lastUpdate.secs + rospy.Time.now().secs + (rospy.Time.now().nsecs - self.lastUpdate.nsecs)/1000000000.0)
            self.lastUpdate = rospy.Time.now()
        self.voltage = voltage
        self.power = self.current * self.voltage

global monitor_drive
global monitor_electronics
global e_pub
global d_pub
monitor_drive = EnergyMonitor()
monitor_electronics=EnergyMonitor()
def callbackCD(data):
    monitor_drive.updateCurrent(data.data)
def callbackVD(data):
    monitor_drive.updateVoltage(data.data*3) #ina219 hw modifications. replaced shunt by a much smaller one to be able to measure out peak currents
def callbackVE(data):
    monitor_electronics.updateVoltage(data.data)
def callbackCE(data):
    monitor_electronics.updateCurrent(data.data)

def my_callback(event):
    e_pub.publish(Int32(monitor_electronics.energy/3600.0))#convert from milli-Joule to mWh
    d_pub.publish(Int32(monitor_drive.energy/3600.0))

rospy.init_node('energy_monitor')
rospy.Timer(rospy.Duration(0.5), my_callback)
d_pub = rospy.Publisher("/energy/drive", Int32, queue_size=3)
e_pub = rospy.Publisher("/energy/electronics", Int32, queue_size=3)
rospy.Subscriber("/batteries/drive/current", Float32, callbackCD)
rospy.Subscriber("/batteries/drive/voltage", Float32, callbackVD)
rospy.Subscriber("/batteries/electronics/current", Float32, callbackCE)
rospy.Subscriber("/batteries/electronics/voltage", Float32, callbackVE)
r = rospy.Rate(10) # 10hz
rospy.spin()
