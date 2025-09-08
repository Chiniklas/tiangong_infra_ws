#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import subprocess
import rospy
from sensor_msgs.msg import JointState

def _csv_to_list(s, cast=float):
    s = (s or "").strip()
    if not s:
        return None
    parts = [p.strip() for p in s.split(",") if p.strip() != ""]
    return [cast(p) for p in parts]

def _csv_to_strlist(s):
    s = (s or "").strip()
    if not s:
        return None
    return [p.strip() for p in s.split(",") if p.strip() != ""]

def _import_service_class(pkg_dot_srv, cls_candidates):
    try:
        mod = __import__(pkg_dot_srv, fromlist=['*'])
    except Exception:
        return (None, None)
    for cls in cls_candidates:
        try:
            Srv = getattr(mod, cls)
            Req = getattr(mod, cls + "Request")
            return (Srv, Req)
        except Exception:
            continue
    return (None, None)

def _call_service_dynamic(srv_name, names, values, srv_pkg='bodyctrl_msgs.srv',
                          cls_candidates=('set_angle_flexible', 'SetAngleFlexible'),
                          field_names=('name', 'position')):
    try:
        from rosservice import get_service_class_by_name as _get_srv_class
        SrvCls = _get_srv_class(srv_name)
        if SrvCls is not None:
            req = SrvCls._request_class()
            setattr(req, field_names[0], names)
            setattr(req, field_names[1], values)
            rospy.wait_for_service(srv_name, timeout=5.0)
            proxy = rospy.ServiceProxy(srv_name, SrvCls)
            return True, proxy(req)
    except Exception:
        pass

    Srv, Req = _import_service_class(srv_pkg, cls_candidates)
    if Srv is None:
        return False, "Could not import service class from {} candidates {}.".format(srv_pkg, cls_candidates)
    try:
        rospy.wait_for_service(srv_name, timeout=5.0)
        proxy = rospy.ServiceProxy(srv_name, Srv)
        req = Req()
        setattr(req, field_names[0], names)
        setattr(req, field_names[1], values)
        return True, proxy(req)
    except Exception as e:
        return False, "Service call failed: {}".format(e)

def _rosservice_shell_call(srv_name, payload_yaml):
    try:
        out = subprocess.check_output(["rosservice", "call", srv_name, payload_yaml],
                                      stderr=subprocess.STDOUT)
        return True, out.decode("utf-8", errors="ignore")
    except subprocess.CalledProcessError as e:
        return False, e.output.decode("utf-8", errors="ignore")
    except Exception as e:
        return False, str(e)

def _publish_topic_fallback(hand, names, positions):
    pub = rospy.Publisher("/inspire_hand/ctrl/{}".format(hand), JointState, queue_size=1, latch=True)
    rospy.sleep(0.3)
    msg = JointState()
    msg.name = names
    msg.position = positions
    pub.publish(msg)
    rospy.logwarn("Fell back to topic command: /inspire_hand/ctrl/%s", hand)

def main():
    rospy.init_node("hand_angle_test")

    hand         = rospy.get_param("~hand", "left_hand")
    names_csv    = rospy.get_param("~names", "fore,thumb_bend")
    pos_csv      = rospy.get_param("~positions", "80,40")
    speed_csv    = rospy.get_param("~speed", "")
    force_csv    = rospy.get_param("~force", "")

    names        = _csv_to_strlist(names_csv)
    positions    = _csv_to_list(pos_csv, float)
    speeds       = _csv_to_list(speed_csv, float)
    forces       = _csv_to_list(force_csv, float)

    if not names or not positions or len(names) != len(positions):
        rospy.logerr("Invalid names/positions. names=%s positions=%s", names, positions)
        sys.exit(1)

    angle_srv = "/inspire_hand/set_angle_flexible/{}".format(hand)
    speed_srv = "/inspire_hand/set_speed/{}".format(hand)
    force_srv = "/inspire_hand/set_force/{}".format(hand)

    # Optional: speed
    if speeds:
        if len(speeds) != len(names):
            rospy.logwarn("Speed length != names; broadcasting one value to all.")
            if len(speeds) == 1:
                speeds = [speeds[0]] * len(names)
            else:
                rospy.logwarn("Skipping speed setup due to mismatch.")
                speeds = None
        if speeds:
            ok, resp = _call_service_dynamic(speed_srv, names, speeds,
                                             cls_candidates=("set_speed", "SetSpeed"),
                                             field_names=("name", "speed"))
            if not ok:
                rospy.logwarn("Speed service failed (%s). Trying rosservice shell...", resp)
                ok2, resp2 = _rosservice_shell_call(speed_srv, "{name: %s, speed: %s}" % (names, speeds))
                if not ok2:
                    rospy.logwarn("Speed via shell failed (%s). Continuing.", resp2)

    # Optional: force
    if forces:
        if len(forces) != len(names):
            rospy.logwarn("Force length != names; broadcasting one value to all.")
            if len(forces) == 1:
                forces = [forces[0]] * len(names)
            else:
                rospy.logwarn("Skipping force setup due to mismatch.")
                forces = None
        if forces:
            ok, resp = _call_service_dynamic(force_srv, names, forces,
                                             cls_candidates=("set_force", "SetForce"),
                                             field_names=("name", "force"))
            if not ok:
                rospy.logwarn("Force service failed (%s). Trying rosservice shell...", resp)
                ok2, resp2 = _rosservice_shell_call(force_srv, "{name: %s, force: %s}" % (names, forces))
                if not ok2:
                    rospy.logwarn("Force via shell failed (%s). Continuing.", resp2)

    # Angle command
    ok, resp = _call_service_dynamic(angle_srv, names, positions,
                                     cls_candidates=("set_angle_flexible", "SetAngleFlexible"),
                                     field_names=("name", "position"))
    if not ok:
        rospy.logwarn("Angle service failed (%s). Trying rosservice shell...", resp)
        ok2, resp2 = _rosservice_shell_call(angle_srv, "{name: %s, position: %s}" % (names, positions))
        if not ok2:
            rospy.logwarn("rosservice shell also failed (%s). Falling back to topic publisher.", resp2)
            _publish_topic_fallback(hand, names, positions)
        else:
            rospy.loginfo("Angle set via rosservice shell: %s", resp2)
    else:
        rospy.loginfo("Angle service responded: %s", str(resp))

    rospy.loginfo("hand_angle_test complete.")

if __name__ == "__main__":
    main()

