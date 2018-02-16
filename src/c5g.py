#!/usr/bin/env python

import socket
import time
import os
import rospy

PACKAGE='edo_core_pkg'
import roslib
roslib.load_manifest(PACKAGE)

#from dynamic_reconfigure.server import Server as DynamicReconfigureServer

import std_msgs.msg

from edo_core_msgs.msg import JointControl
from edo_core_msgs.msg import MovementCommand
from edo_core_msgs.msg import JointControlArray
from edo_core_msgs.msg import MovementFeedback

from edo_core_msgs.srv import ControlSwitch

from parse import parse

algo_jnt_ctrl = rospy.Publisher('algo_jnt_ctrl', JointControlArray, queue_size=1)
machine_move = rospy.Publisher('machine_move', MovementCommand, queue_size=1)

waitme = 0
def callback_algo_movement_ack(ack):
    global waitme

    #print("callback algo movement ack")
    if ack.type == 2:
        waitme = 1

startc5g = 0
def callback_bridge_c5g(val):
    global startc5g

    #print("callback bridge c5g")
    #print(val)
    if val.data == True:
        startc5g = 1
    else:
        startc5g = 0


rospy.init_node('c5g', anonymous=True)
rate = rospy.Rate(100) # 100hz

rospy.wait_for_service('algo_control_switch_srv')
control_switch = rospy.ServiceProxy('algo_control_switch_srv', ControlSwitch)

algo_movement_ack = rospy.Subscriber('algo_movement_ack', MovementFeedback, callback_algo_movement_ack)
bridge_c5g = rospy.Subscriber('bridge_c5g', std_msgs.msg.Bool, callback_bridge_c5g)

if __name__ == '__main__':

    while True:
        while startc5g == 0:
            time.sleep(1)

        print("enter slave mode")


        #msg = JointControlArray()
        #msg.size = 10

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("10.42.0.40", 8889))

        s.send("<CONNECTION_OPEN>")
        data = s.recv(1024)

        if data != "<CONNECTION_OPEN;OK>":
            exit()

        s.send("<GET_ARM_JNT;1>")
        data = s.recv(1024)
        parsed = parse("<GET_ARM_JNT;1;{};{};{};{};{};{};{};{};{};{}>", data).fixed

        joints = [
                float(parsed[0]),
                float(parsed[1]),
                float(parsed[2]),
                float(parsed[3]),
                float(parsed[4]),
                float(parsed[5])
                ]

        #machine_move_pub = rospy.Publisher('machine_move', MovementCommand)
        mmmsg = MovementCommand(0, len(joints), 25, joints, '0')
        #print mmmsg

        # publish move
        machine_move.publish(mmmsg)
        
        while waitme == 0:
            time.sleep(1)

        #print("after waitme")
        control_switch(1)
        os.system("kill $(ps aux | grep 'edo_algorithms' | awk '{print $2}')")
        #print("after control switch")

        while not rospy.is_shutdown() and startc5g == 1:
            s.send("<GET_ARM_JNT;1>")
            # print(" inviato: <GET_ARM_JNT;1>")
            data = s.recv(1024)
            # print("ricevuto: " + data)
            parsed = parse("<GET_ARM_JNT;1;{};{};{};{};{};{};{};{};{};{}>", data).fixed
            # print(parsed)
            # print(parse("<GET_ARM_JNT;{};{};{};{};{};{};{};{};{};{}>", data).fixed)
            joints = [
            float(parsed[0]),
            float(parsed[1]),
            float(parsed[2]),
            float(parsed[3]),
            float(parsed[4]),
            float(parsed[5])
            ]
            #joints = [1, 2, 3, 4, 5, 6]
            #print(joints)
            ctrlMsg = JointControlArray()
            ctrlMsg.size = len(joints)
            ctrlMsg.joints = [JointControl(i, 0, 0, 0, 0) for i in joints]
            algo_jnt_ctrl.publish(ctrlMsg)
            rate.sleep()

        s.send("<CONNECTION_CLOSE>")
        s.close()
        control_switch(2)

        waitme = 0
        print("exit slave mode")
