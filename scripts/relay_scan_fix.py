#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def fix_scan(msg):
    fixed = LaserScan()
    fixed.header.stamp = rospy.Time.now()          # 新时间戳
    fixed.header.frame_id = "lidar_frame"

    # 角度信息原样拷贝
    fixed.angle_min        = msg.angle_min
    fixed.angle_max        = msg.angle_max
    fixed.angle_increment  = msg.angle_increment

    fixed.range_min = 0.02                         # LD06 规格
    fixed.range_max = 12.0
    fixed.ranges    = msg.ranges                   # intensities 对 gmapping 不重要
    fixed.intensities = []                         # 减肥一下

    fixed.scan_time      = 0.1                     # 10 Hz → 0.1 s/圈
    fixed.time_increment = fixed.scan_time / max(len(msg.ranges), 1)

    pub.publish(fixed)

rospy.init_node("relay_scan_fix")
pub = rospy.Publisher("/scan_fixed", LaserScan, queue_size=1)
rospy.Subscriber("/LiDAR/LD06", LaserScan, fix_scan)
rospy.loginfo("relay_scan_fix ➜ /scan_fixed")
rospy.spin()

