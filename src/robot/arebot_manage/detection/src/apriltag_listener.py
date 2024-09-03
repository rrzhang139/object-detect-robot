#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray

def tag_callback(data):
    if data.detections:
        for detection in data.detections:
            tag_id = detection.id[0]
            tag_pos = detection.pose.pose.pose.position
            rospy.loginfo("Detected AprilTag ID %d at position: x=%.2f, y=%.2f, z=%.2f" % 
                          (tag_id, tag_pos.x, tag_pos.y, tag_pos.z))
    #else:
    #    rospy.loginfo("No AprilTags detected")

def main():
    rospy.init_node('apriltag_listener', anonymous=True)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)
    print("here")
    rospy.spin()

if __name__ == '__main__':
    main()



