#!/usr/bin/env python3
import rospy
from bodyctrl_msgs.msg import MotorStatusMsg, CmdMotorCtrl, MotorCtrl

def cb(msg: MotorStatusMsg):
    kp = float(rospy.get_param("~kp", 5.0))   # stiffness
    kd = float(rospy.get_param("~kd", 0.2))   # damping

    out = CmdMotorCtrl()
    out.header = msg.header  # preserve timing/frame (not strictly required)

    # Mirror each joint's recorded position into a low-gain cmd
    for s in msg.status:
        m = MotorCtrl()
        m.name = s.name          # IDs like 1,2,3 from your status
        m.kp = kp
        m.kd = kd
        m.pos = s.pos            # follow recorded position
        m.spd = 0.0
        m.tor = 0.0
        out.cmds.append(m)

    pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("replay_head_from_status")
    pub = rospy.Publisher("/head/cmd_ctrl", CmdMotorCtrl, queue_size=10)
    rospy.Subscriber("/head/status", MotorStatusMsg, cb, queue_size=50)
    rospy.loginfo("replay_head_from_status: forwarding /head/status -> /head/cmd_ctrl")
    rospy.spin()
