#!/usr/bin/env python3
# - * - coding: utf-8 -*-
import rospy 

from bodyctrl_msgs.srv import set_angle_flexible,set_angle_flexibleRequest

def clamp01(x):
    return max(0.0,min(1.0, x ))

if __name__ == "__main__":
    rospy.init_node("hand_open_service_client")
    side = rospy.get_param("~hand_side","right_hand")
    ratio = float(rospy.get_param("~ratio",0.8))
    weights = rospy.get_param("~weights",[1,1,1,1,1,1])
    names = rospy.get_param("~names",['1','2','3','4','5','6'])
    ratio = clamp01(ratio)
    names = eval(names)
    if isinstance(weights, str):
        try:
            weights_param=eval(weights)
        except Exception:
            rospy.logwarn("weight param erro")
            weights_param = [1,1,1,1,1,1]
    weights = [float(w) for w in weights_param]
    if len(weights)!=6:
        weights = [1,1,1,1,1,1]
    mw = max (1.0,max(weights))
    angle_ratios = [clamp01(ratio*(weights[int(fid)-1]/mw))for fid in names]
    #names = [str(i) for i in range(6)]
    srv_name = "/inspire_hand/set_angle_flexible/{}".format(side)
    rospy.loginfo("conecting service:%s",srv_name)
    rospy.wait_for_service (srv_name)
    cli = rospy.ServiceProxy(srv_name,set_angle_flexible)
    req = set_angle_flexibleRequest()
    req.name = [str(n) for n in names]
    req.angleRatio = angle_ratios
    try:
        resp = cli(req)
        rospy.loginfo("OK:side=%s ratio=%.2f angleRatio = %s", side,ratio,angle_ratios )
    except rospy.ServiceException as e:
        rospy.logerr("service faile: %s", e)
