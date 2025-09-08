#!/usr/bin/env python3
import rospy
from my_basics.msg import Person           # 导入自己包里的消息类型
def main():
    rospy.init_node("person_pub")
    pub = rospy.Publisher("/person", Person, queue_size=10)
    rate = rospy.Rate(2)
    i = 0
    while not rospy.is_shutdown():
        msg = Person(name=f"Alice{i}", age=i%100, height=1.70)
        pub.publish(msg)
        rate.sleep(); i+=1
        rospy.loginfo(f"pub: {msg}")
if __name__ == "__main__":
    main()