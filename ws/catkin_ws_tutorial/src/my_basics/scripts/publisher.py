#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("publisher_demo")
    pub = rospy.Publisher("/chatter", String, queue_size=10)
    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown():
        msg = f"hello #{i}"
        pub.publish(String(data=f"hello #{i}"))
        rospy.loginfo(f"pub: {msg}") 
        i += 1

        rate.sleep()

if __name__ == "__main__":
    main()
