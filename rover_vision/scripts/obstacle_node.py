# rover_vision/scripts/obstacle_node.py
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def depth_cb(msg):
  cv_depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
  # Do something with depth (e.g., generate a local obstacle map)

if __name__ == "__main__":
  rospy.init_node("obstacle_node")
  rospy.Subscriber("/camera/depth/image_raw", Image, depth_cb)
  rospy.spin()
