#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(scan_msg):
    new_msg = LaserScan()
    new_msg.header.stamp = rospy.Time.now()  # ✅ 用当前时间重设
    new_msg.header.frame_id = scan_msg.header.frame_id or "lidar_frame"
    new_msg.angle_min = scan_msg.angle_min
    new_msg.angle_max = scan_msg.angle_max
    new_msg.angle_increment = scan_msg.angle_increment
    new_msg.range_min = scan_msg.range_min
    new_msg.range_max = scan_msg.range_max
    new_msg.ranges = scan_msg.ranges
    new_msg.intensities = scan_msg.intensities

    # ✅ 重新设置时间
    new_msg.scan_time = 1.0 / 10.0  # 假设10Hz雷达
    if len(scan_msg.ranges) > 0:
        new_msg.time_increment = new_msg.scan_time / len(scan_msg.ranges)
    else:
        new_msg.time_increment = 0.0

    pub.publish(new_msg)

if __name__ == "__main__":
    rospy.init_node("relay_scan")
    pub = rospy.Publisher("/scan", LaserScan, queue_size=1)
    rospy.Subscriber("/LiDAR/LD06", LaserScan, callback)
    rospy.loginfo("✅ relay_scan.py started. Relaying /LiDAR/LD06 → /scan with fixes.")
    rospy.spin()

