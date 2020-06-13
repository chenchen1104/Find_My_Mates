## API

#### bookshelf.py
1. 用于检测GAZEBO中的书架作为测试
2. 接受：
   1. 摄像头：              ~image_raw_topic_name[Image]
3. 发布
   1. 视野中目标的位置      ~object_view_topic_name[RegionOfInterest]
   2. 带有目标的图像        ~image_test_topic_name[Image]

#### wave_detect
1. 使用百度API进行关键点视觉处理。实际上可以使用openpose，但是其算力要求过高。
2. 方法：现在是检测肘，腕，肩三个位置判断。
3. 接受消息：
   1. RGB图像             ~image_raw_topic_name[Image]
4. 发布消息：
   1. 视野中目标的位置      ~object_view_topic_name[RegionOfInterest]
   2. 带有目标的图像        ~image_test_topic_name[Image]
5. 参数：
   1. check_gender                  bool    检查性别
   2. image_temp_save_path          str     临时变量

#### feature_detect
1. 使用百度API进行人体特征视觉处理。目前没有可以替代的离线程序。
2. 方法：利用百度API获得对象的特征。
3. 接受消息：
   1. RGB图像             ~image_raw_topic_name[Image]
   2. 感兴趣区域           ~object_view_topic_name[RegionOfInterest]
4. 发布消息：
   1. 人的特征            ~feature_topic_name[speech/Description]



## detection



## openpose

#### openpose_hand_point.py
1. 使用openpose进行人体手指指向识别判断
2. 方法：使用openpose的肘，腕的位置判断
3. 接受消息：
   1. RGB图像             /camera/rgb/image_raw[Image]
4. 发布消息：
   1. 手指指向             /openpose/point_direction[kamerider_image_msgs/PointDirection]
   2. 测试图像             /openpose/img[Image]

#### openpose_wave_detect.py
1. 使用openpose进行挥手识别
2. 方法：使用Openpose的肘，腕，肩位置判断，速度比百度云API快很多
3. 接受消息：
   1. RGB图像             /usb_cam/image_raw[Image]
4. 发布消息：
   1. RGB图像             /openpose/img[Image]
   2. 挥手方向             /openpose/wave[RegionOfInterest]


## pcl

#### pcd_get_depth
1. 使用点云和YOLO的感兴趣区域，计算距离目标的平均距离。
2. 方法：计算点云数据的深度。使用平均值或者区域附近的有效数值，距离以2m作为安全距离
3. 接受消息
   1. 感兴趣区域             ~object_view_topic_name[darknet_ros_msgs/BoundingBoxes]
   2. 点云                  ~pointcloud_topic_name[PointCloud2]
4. 发布消息
   1. 机器人坐标系的目标坐标   ~object_positon2d_topic_name[Pose]

#### pcd_get_depth_test
1. 使用点云和ROI的感兴趣区域，计算距离目标的平均距离。
2. 方法：计算点云数据的深度。使用平均值或者区域附近的有效数值，距离以2m作为安全距离
3. 接受消息
   1. 感兴趣区域             ~object_view_topic_name[RegionOfInterest]
   2. 点云                  ~pointcloud_topic_name[PointCloud2]
4. 发布消息
   1. 机器人坐标系的目标坐标   ~object_positon2d_topic_name[Pose]

#### darknet_to_roi.py
1. 将yolo的消息转成roi信息
2. 方法：直接接受和发布
3. 接受消息
   1. YOLO消息               /darknet_ros/bounding_boxes
4. 发布消息
   1. roi消息                roi
5. 参数
   1. 目标名字                target_name



## Darknet_ros

内部是yolo_ros的文件。

