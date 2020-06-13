#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
import os

def kill_node(node_name):
    os.system("gnome-terminal -x bash -c 'rosnode kill %s' "%(str(node_name)))
    return
class faceDetector:
    def __init__(self):
        self.take_photo_pub=False
        self.count=0
        self.faces_last_result=0
        self.face_pub=False
        self.control=False
        self.recog=False
        rospy.on_shutdown(self.cleanup)
	self.killed=False

        # 创建cv_bridge
        self.bridge = CvBridge()
        self.recog_sub = rospy.Subscriber("/start_recognize_faces",String,self.recog_callback,queue_size=1)
        self.control_sub = rospy.Subscriber("/start_control",String,self.control_callback,queue_size=1)
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.voice_pub = rospy.Publisher("/xfwords", String, queue_size=5)
        self.faces = rospy.Publisher("/xfwords", String, queue_size=5)


        cascade_1="/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_frontalface_alt.xml"
        cascade_2="/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_eye_tree_eyeglasses.xml"


        # 获取haar特征的级联表的XML文件，文件路径在launch文件中传入
        #cascade_1 = rospy.get_param("~cascade_1", "")
        #cascade_2 = rospy.get_param("~cascade_2", "")

        # 使用级联表初始化haar特征检测器
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)

        # 设置级联表的参数，优化人脸识别，可以在launch文件中重新配置
        self.haar_scaleFactor  = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        self.haar_minSize      = rospy.get_param("~haar_minSize", 40)
        self.haar_maxSize      = rospy.get_param("~haar_maxSize", 60)
        self.color = (50, 255, 50)

        # 初始化订阅rgb格式图像数据的订阅者，此处图像topic的话题名可以在launch文件中重映射
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
        if self.recog==True:
	    if self.killed==False:
	        kill_node('wave_detect')
		self.killed=True
		rospy.sleep(8)
            strings=""
            self.count+=1
            #key = getKey()
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                frame = np.array(cv_image, dtype=np.uint8)
            except CvBridgeError as e:
                print(e)

            # 创建灰度图像
            grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 创建平衡直方图，减少光线影响
            grey_image = cv2.equalizeHist(grey_image)

            # 尝试检测人脸
            faces_result = self.detect_face(grey_image)
            rospy.loginfo("faces_num %d"%(len(faces_result)))

            if len(faces_result)>(self.faces_last_result) and self.face_pub==False:
               # facesstring="I found %d face."%(len(faces_result))
 		facesstring="Now I find Jack."
                self.face_pub=True
                rospy.loginfo(facesstring)
                self.faces.publish(facesstring)
                self.faces_last_result=len(faces_result)
                #print(self.take_photo_pub,len(faces_result),self.control,self.count%10)
            if (self.take_photo_pub==False and len(faces_result)>0 and self.control==True and self.count%12==0):
                self.count=0
                rospy.loginfo("checking...")

            # 在opencv的窗口中框出所有人脸区域
                if len(faces_result)>0:
                    for face in faces_result:
                        x, y, w, h = face
                        cv2.rectangle(cv_image, (x, y), (x+w, y+h), self.color, 2)
                    mid_x=x+w/2
                    mid_y=y+h/2
                    rospy.loginfo(mid_x)
                    rospy.loginfo(mid_y)
            # 根据脸的位置发布速度消息
                    if  (mid_y<=140 and 220<mid_x and mid_x<420):
                        strings="Please step back a little"

            # 后退
                    elif (mid_y>=340 and 220<mid_x and mid_x<420):
                        strings="Please move forward a little"


            # 停止
                    elif (mid_y>=140 and mid_y<=340 and 220<=mid_x and mid_x<=420):
                        self.take_photo_pub=True
                        strings="take photo"


            # 左转
                    elif (mid_x>=420):
                        strings="Please move left a little"


            # 右转
                    elif (mid_x<=220):
                        strings="Please move right a little"

                    else:
                        strings="other"

                #if (key == '\x03'):
                    #break
                    self.voice_pub.publish(strings)
                    rospy.loginfo("strings:%s"%strings)


            # 将识别后的图像转换成ROS消息并发布
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        #rospy.loginfo("image_pubs")



    def control_callback(self, data):
	self.control=True

    def recog_callback(self, data):
	self.recog=True

    def detect_face(self, input_image):
        # 首先匹配正面人脸的模型
        if self.cascade_1:
            faces = self.cascade_1.detectMultiScale(input_image, 
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))
        #rospy.loginfo("face_num_zhengmian %d" %(len(faces)))  
               
        # 如果正面人脸匹配失败，那么就尝试匹配侧面人脸的模型
        #if len(faces) == 0 and self.cascade_2:
        #    faces = self.cascade_2.detectMultiScale(input_image, 
        #            self.haar_scaleFactor, 
        #            self.haar_minNeighbors, 
        #            cv2.CASCADE_SCALE_IMAGE, 
        #            (self.haar_minSize, self.haar_maxSize))
        
        #rospy.loginfo("face_num_zhengmian%d"%(len(faces)))   

        return faces

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

if __name__ == '__main__':

    try:
        # 初始化ros节点
        rospy.init_node("face_detector")
        faceDetector()
        rospy.loginfo("Face detector is started..")
        rospy.loginfo("Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
        cv2.destroyAllWindows()
