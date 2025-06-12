#!/usr/bin/env python3
import rospy
import socket
import json
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class NetworkOdomNode:
    def __init__(self, host='192.168.137.13', port=7777):
        rospy.init_node('network_odom_node')
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.br = tf.TransformBroadcaster()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        self.sock.settimeout(2.0)
        self.buffer = ""

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rospy.loginfo("✅ Connected to Raspberry Pi odometry server")

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                data = self.sock.recv(1024).decode('utf-8')
                self.buffer += data
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    self.process_line(line.strip())
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr(f"❌ Socket error: {e}")
                break
            rate.sleep()

    def process_line(self, line):
        try:
            msg = json.loads(line)
            angle_deg = float(msg['angle'])
            distance = float(msg['distance'])

            # 使用上一帧朝向更新位置，当前角度为朝向角
            dx = distance * math.cos(self.theta)
            dy = distance * math.sin(self.theta)
            self.x += dx
            self.y += dy
            self.theta = math.radians(angle_deg)

            self.publish_odom()

        except Exception as e:
            rospy.logwarn(f"⚠️ Failed to parse line: {line}")

    def publish_odom(self):
        now = rospy.Time.now()
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # 发布 odometry 消息
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0

        self.pub.publish(odom)

        # 发布 TF 变换（odom → base_link）
        self.br.sendTransform(
            (self.x, self.y, 0.0),
            quat,
            now,
            "base_link",
            "odom"
        )

if __name__ == '__main__':
    try:
        node = NetworkOdomNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

