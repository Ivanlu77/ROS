#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    republished_msg = LaserScan()
    republished_msg = msg  # 复制原始数据
    republished_msg.header.stamp = rospy.Time.now()  # 重新打时间戳
    republished_msg.header.frame_id = "lidar_frame"  # 设置为你使用的 frame
    pub.publish(republished_msg)

if __name__ == '__main__':
    rospy.init_node('laser_republisher')
    
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rospy.Subscriber('/LiDAR/LD06', LaserScan, callback)

    rospy.spin()
