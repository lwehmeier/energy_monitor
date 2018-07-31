#!/usr/bin/env python2
import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Vector3, PoseStamped, PoseArray, Twist

def cbDrive(int32):
    global drvEnergy
    drvEnergy = int32.data
def cbElec(int32):
    global elecEnergy
    elecEnergy = int32.data
def newStep(msg):
    global active
    if active:
        return
    active = True
    global last_drv
    last_drv = drvEnergy
    global last_elec
    last_elec = elecEnergy
def stepDone(msg):
    global active
    if not active:
        return
    active = False
    ePub.publish(Int32(elecEnergy - last_elec))
    dPub.publish(Int32(drvEnergy - last_drv))

active = False
drvEnergy =0
elecEnergy=0
last_drv = 0
last_elec = 0
rospy.init_node("energy_per_step")
ePub = rospy.Publisher("/energy/last_step/electronics", Int32, queue_size=5)
dPub = rospy.Publisher("/energy/last_step/drive", Int32, queue_size=5)
rospy.Subscriber("/energy/drive", Int32, cbDrive)
rospy.Subscriber("/energy/electronics", Int32, cbElec)
rospy.Subscriber("/direct_move/reached_target", Bool, stepDone)
#rospy.Subscriber("/direct_move/abort", Bool, stepDone) # TODO: handle this correctly, map_monitor might use it to stop before recomputing alternative path
rospy.Subscriber("/direct_move/rot_target", PoseStamped, newStep)
rospy.Subscriber("/direct_move/lin_target", PoseStamped, newStep)
rospy.Subscriber("/direct_move/lin_rot_target", PoseStamped, newStep)
rospy.Subscriber("/direct_move/lin_rot_trajectory", PoseArray, newStep)
rospy.Subscriber("/local_planner/target", PoseStamped, newStep)
rospy.Subscriber("/local_planner/trajectory", PoseArray, newStep)
rospy.Subscriber("/cmd_vel", Twist, newStep)
rospy.Subscriber("/platform/combined", Vector3, newStep)
rospy.spin()
