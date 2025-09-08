#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
def cb(msg):
    rospy.loginfo(f"sub: {msg.data}")
def main():
    rospy.init_node("listener")
    rospy.Subscriber("/chatter", String, cb, queue_size=10)
    rospy.spin()
if __name__ == "__main__":
    main()