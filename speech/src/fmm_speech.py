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
from sound_play.libsoundplay import SoundClient
from speech.msg import Description
from control.msg import findmymate

class fmm_speech(object):
    def __init__(self):
        #名字列表
        self.name_list = ["amelia","angel","charlie","ava","hunter","jack","charlotte","max",
                          "noah","oliver","mia", "parker","sam","thomas","william", "olivia"]
        self.flag=1
        rospy.Subscriber("/xunfei_to_control", String, self.xfeiCallback)#订阅语音识别结果的话题
        rospy.Subscriber("/image/people_feature", Description, self.visDesCallback)
        rospy.Subscriber("/image/people_position", String, self.visPosCallback)
        rospy.Subscriber("/Find_My_Mate/control", findmymate, self.controlCallback)
        self.speech_pub_to_control = rospy.Publisher('/Find_My_Mate/control', findmymate, queue_size=1)

        self.message = findmymate()
        self.message.NeedHelp = False
        self.message.FinishState = False

        self.to_find_person_name = None #要找的人的名字
        self.person_information = None  #要找的人的信息

        self.age = "unknown" 
        self.gender = "unknown"
        self.skin_color = "unknown"
        self.hair_style = "unknown"
        self.glasses = "unknown"
        self.clothes_color = "unknown"
        self.clothes = "unknown"
        self.pose = "unknown"
        self.position = "unknown"        

        self.sh = SoundClient(blocking = True)
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")

    def xfeiCallback(self,msg):
        if msg.data.strip()=='':
            self.sh.say("I did not clearly hear you", self.voice)
            self.sh.say("please tell me again", self.voice)
            os.system("rostopic pub -1 /kws_data std_msgs/String 'jack'")

        else:
            string = msg.data
            symbols = ["!", "?", ".", ",", ";", ":"]
            output = []
            if string[-1] in symbols:
                string = string[:-1]
            for part in string.lstrip().split(","):
                for word in part.split():
                    for symbol in symbols:
                        if symbol in word:
                            word = word[:-1]
                    output.append(word)
            output = [item.lower() for item in output]
            print output

        for i in range(len(output)):
            if output[i] in self.name_list:
                self.to_find_person_name = output[i]
                break
        if self.to_find_person_name == None:
            self.sh.say("sorry, please tell me again", self.voice)
            os.system("rostopic pub -1 /kws_data std_msgs/String 'jack'")
        else:
            self.sh.say("ok i will find {}".format(self.to_find_person_name), self.voice)
            self.message.NowTask = self.message.Receiving
            self.message.NextTask = self.message.EnterHome
            self.message.FinishState = True
            self.message.NeedHelp = False
            self.speech_pub_to_control.publish(self.message)
            os.system("rosnode kill /voiceRecognition")#关闭语音识别节点

    def controlCallback(self,msg):
        if msg.NowTask == self.message.Receiving and msg.FinishState == False:
            self.sh.say("Dear operator, please talk to me after you hear ding ding dong", self.voice)
            self.sh.say("ok I'm ready, please give me the name", self.voice)
            self.message.NowTask = msg.NowTask
            self.message.NextTask = msg.NextTask
            self.message.FinishState = False
            self.message.NeedHelp = False

        if msg.NowTask == self.message.Inquiry and msg.FinishState == False:
            self.sh.say("Dear guest, i want to find {},please wave your hand".format(self.to_find_person_name), self.voice)
            self.message.NowTask = msg.NowTask
            self.message.NextTask = msg.NextTask
            self.message.FinishState = True
            self.message.NeedHelp = False
            self.speech_pub_to_control.publish(self.message)            
        if msg.NowTask == self.message.Description and msg.FinishState == False:
            self.sh.say("Dear operator,the information of {} as follows".format(self.to_find_person_name), self.voice)
            self.information()              
            self.sh.say(self.person_information, self.voice)
            self.sh.say("ok i will find next person, please give me the name", self.voice)
            self.to_find_person_name = None
            self.person_information = None
            os.system("gnome-terminal -x bash -c 'rosrun xfei_asr speech_recognition'")#开启语音识别节点
            rospy.sleep(1)
            os.system("rostopic pub -1 /kws_data std_msgs/String 'jack'")
            self.message.NowTask = msg.NowTask
            self.message.NextTask = msg.NextTask
            self.message.FinishState = True
            self.message.NeedHelp = False
            self.speech_pub_to_control.publish(self.message)

    def visDesCallback(self,msg):
	if self.flag==1:
            self.age = msg.age 
            self.gender = msg.gender
            self.skin_color = msg.skin_color
            self.hair_style = msg.hair_style
            self.glasses = msg.glasses
            self.clothes_color = msg.clothes_color
            self.clothes = msg.clothes
            self.pose = msg.pose
	    self.flag=0
        # 接受到消息则停止此任务

#my_change
        #task = findmymate()
        #task.NowTask = task.Recording
        #task.NextTask = task.EnterGate
        #task.FinishState = True
        #task.NeedHelp = False
        #self.speech_pub_to_control.publish(task)
#change_end

    def visPosCallback(self,msg):
        self.position = msg.data    

    def information(self):
        if self.gender == "male":
            self.person_information = "the guest is a " + self.gender + ", "
            if self.age != "unknown":
                self.person_information = self.person_information + "he is about {} years old, ".format(self.age)
            if self.skin_color != "unknown":
                self.person_information = self.person_information + "his skin color is " + self.skin_color + ", "
            if self.hair_style != "unknown":
                self.person_information = self.person_information + "his hair style is " + self. hair_style + ", "
            if self.glasses != "unknown":
                if self.glasses != "none":
                    self.glasses = "glasses"
                self.person_information = self.person_information + "he is wearing " + self.glasses + ", "
            if self.clothes_color != "unknown":
                if self.clothes != "unknown":
                    self.person_information = self.person_information + "he is wearing a " + self.clothes_color + " " + self.clothes + ", "
                else:
                    self.person_information = self.person_information + "he dresses up in " + self.clothes_color + ", "
            if self.pose == "sit":
                self.person_information = self.person_information + "he is sitting on the " + self.position
            if self.pose == "stand":
                self.person_information = self. person_information + "he is standing around the " + self.position
            
        if self.gender == "female":
            self.person_information = "the guest is a " + self.gender + ", "
            if self.age != "unknown":
                self.person_information = self.person_information + "she is about {} years old, ".format(self.age)
            if self.skin_color != "unknown":
                self.person_information = self.person_information + "her skin color is " + self.skin_color + ", "
            if self.hair_style != "unknown":
                self.person_information = self.person_information + "her hair style is " + self. hair_style + ", "
            if self.glasses != "unknown":
                self.person_information = self.person_information + "she is wearing " + self.glasses + ", "
            if self.clothes_color != "unknown":
                if self.clothes != "unknown":
                    self.person_information = self.person_information + "she is wearing a " + self.clothes_color + " " + self.clothes + ", "
                else:
                    self.person_information = self.person_information + "she dresses up in " + self.clothes_color + " "
            if self.pose == "sit":
                self.person_information = self.person_information + "she is sitting on the " + self.position
            if self.pose == "stand":
                self.person_information = self. person_information + "she is standing around the " + self.position

if __name__ == '__main__':
    rospy.init_node('fmm_speech', anonymous=True)
    ctrl = fmm_speech()
    rospy.spin() 
