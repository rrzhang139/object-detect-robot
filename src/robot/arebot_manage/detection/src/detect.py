#!/usr/bin/env python

from enum import Enum
import rospy
import numpy as np 
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Range, CameraInfo
from apriltag_ros.msg import AprilTagDetectionArray

class_id = None
bbox_x = 0
bbox_y = 0
IM_WIDTH = 1280
range_value = 1000
MIN_RANGE = 0.2
tag_pos =None 

def callback(msg):
   global class_id, bbox_x, bbox_y
   for det in msg.detections:
       for res in det.results:
           class_id = res.id
           if class_id == 47:
               bbox_x = det.bbox.center.x / IM_WIDTH
               bbox_y = det.bbox.center.y

def ultra_callback(msg):
    global range_value
    if range_value >= MIN_RANGE:
        range_value = msg.range
        print("close: ", range_value)

def tag_callback(data):
    global tag_pos
    if data.detections:
        for detection in data.detections:
            tag_id = detection.id[0]
            tag_pos = detection.pose.pose.pose.position
            rospy.loginfo("Detected AprilTag ID %d at position: x=%.2f, y=%.2f, z=%.2f" %
                          (tag_id, tag_pos.x, tag_pos.y, tag_pos.z))
            
def cam_callback(data):
    pass

def main():
    sub = rospy.Subscriber('/detectnet/detections', Detection2DArray, callback)
    tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)
    cam_sub = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, cam_callback)
    ultra_sub = rospy.Subscriber('/szar_ultrasonic/range', Range, ultra_callback)
    arm_pos_pub = rospy.Publisher('/szarbot_arm/position', Point, queue_size=10)
    arm_suc_pub = rospy.Publisher('/szarbot_arm/suction_cup', Bool, queue_size=1)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
    rospy.init_node('detection', anonymous=True)
    rate = rospy.Rate(0.5)
    msg = Twist()
    State = Enum('State', 'UnPositioned Positioned ArmPositioned Suction RePosition Finish')
    state = State.UnPositioned

    while not rospy.is_shutdown():
        if state == State.UnPositioned:
            if range_value < MIN_RANGE:
                print("stopping: ", range_value)
                msg.angular.z = 0
                msg.linear.x = 0
                pub.publish(msg)
                state = State.Positioned

            if class_id == 47:
                msg.angular.z = -(bbox_x - 0.5)
                msg.linear.x = 0.1
                pub.publish(msg)

        elif state == State.Positioned:
            # position the robot arm
            if tag_pos: # wait until it sees the april tag
                point = Point()
                tag_pose_cam = np.array([[tag_pos.x, tag_pos.y, tag_pos.z]])
                # TODO: get rot matrix from camera info
                transform_matrix = np.concat([rot_matrix, tag_pose_cam], axis=-1)
                
                point.x = tag_pos.x
                point.y = tag_pos.y
                point.z = tag_pos.z
                arm_pos_pub.publish(point)
                state = State.ArmPositioned
        elif state == State.ArmPositioned:
            # activate suction
            b = Bool()
            b.data=True
            arm_suc_pub.publish(b)
            state = State.Suction
        elif state == State.Suction:
            point = Point()
            point.x = 250
            point.y = 0
            point.z = 300 # TODO find exact height
            arm_pos_pub.publish(point)
            state = State.RePosition
        elif state == State.RePosition:
            # desuction
            b = Bool()
            b.data = False
            arm_suc_pub.publish(b)
            state = State.Finish

        rate.sleep()


    rospy.spin()

if __name__ == '__main__':
    main()
