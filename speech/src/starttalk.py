#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
import os
import sys
import time
import wave
import datetime
import pyaudio
from std_msgs.msg import String
from control.msg import findmymate
from sound_play.libsoundplay import SoundClient
from speech.msg import Description
from control.msg import findmymate
def kill_node(node_name):
    os.system("gnome-terminal -x bash -c 'rosnode kill %s' "%(str(node_name)))
    return
class stop_wakeup(object):
    def __init__(self):
        self.speech_pub_to_control = rospy.Publisher('find_my_mate/control', findmymate, queue_size=1)

        self.end_sub = rospy.Subscriber('talkend', String, self.callback)


    def callback(self,msg):
	if msg.data=='ok':

	    task = findmymate()
            task.NowTask   = task.EnterGate
            task.NextTask = task.Description
            task.FinishState = True
            task.NeedHelp = False
            self.speech_pub_to_control.publish(task)
            kill_node("en_iat_publish")


if __name__ == '__main__':
    rospy.init_node('stop_wakeup', anonymous=True)
    speech_pub = rospy.Publisher('xfwakeup', String, queue_size=1)
    outdata="ok"
    for i in range(8):
	speech_pub.publish(outdata)
	print(outdata)
    kill_node("roi_follower")
    print("start_pre_kill")
    #os.system("gnome-terminal -x bash -c 'rostopic pub xfwakeup std_msgs/String 'ok'")#开启语音识别节点
    # os.system("rostopic pub xfwakeup std_msgs/String 'ok'")#开启语音识别节点
    ctrl = stop_wakeup()
    rospy.spin() 
