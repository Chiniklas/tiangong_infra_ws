#include <ros/ros.h>
#include <hand_control.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <inspire_hand/status.h>

int hand_id_;
std::string port_name_;
int baudrate_;
int test_flags;
serial::Serial *com_port_;
uint8_t hand_state_;

void callback(const inspire_hand::statusConstPtr &st)
{
    std::cout << "angles set: " << st->angle1_set << ' ' << st->angle2_set << ' ' << st->angle3_set << ' ' << st->angle4_set << ' ' << st->angle5_set << ' ' << st->angle6_set << '\n';
    std::cout << "forces set: " << st->force1_set << ' ' << st->force2_set << ' ' << st->force3_set << ' ' << st->force4_set << ' ' << st->force5_set << ' ' << st->force6_set << '\n';
    std::cout << "speeds set: " << st->speed1_set << ' ' << st->speed2_set << ' ' << st->speed3_set << ' ' << st->speed4_set << ' ' << st->speed5_set << ' ' << st->speed6_set << '\n';
    std::cout << "angles act: " << st->angle1_act << ' ' << st->angle2_act << ' ' << st->angle3_act << ' ' << st->angle4_act << ' ' << st->angle5_act << ' ' << st->angle6_act << '\n';
    std::cout << "forces act: " << st->force1_act << ' ' << st->force2_act << ' ' << st->force3_act << ' ' << st->force4_act << ' ' << st->force5_act << ' ' << st->force6_act << '\n';
    std::cout << "currents: " << st->current1 << ' ' << st->current2 << ' ' << st->current3 << ' ' << st->current4 << ' ' << st->current5 << ' ' << st->current6 << '\n';
    std::cout << "errors: " << (int)st->error1 << ' ' << (int)st->error2 << ' ' << (int)st->error3 << ' ' << (int)st->error4 << ' ' << (int)st->error5 << ' ' << (int)st->error6 << '\n';
    std::cout << "statuses: " << (int)st->status1 << ' ' << (int)st->status2 << ' ' << (int)st->status3 << ' ' << (int)st->status4 << ' ' << (int)st->status5 << ' ' << (int)st->status6 << '\n';
    std::cout << "temps: " << (int)st->temp1 << ' ' << (int)st->temp2 << ' ' << (int)st->temp3 << ' ' << (int)st->temp4 << ' ' << (int)st->temp5 << ' ' << (int)st->temp6 << '\n' << '\n';
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hand_control_comprehensive_subscriber");
    ros::NodeHandle nh;

    //Read launch file params
    nh.getParam("inspire_hand/hand_id", hand_id_);
    nh.getParam("inspire_hand/portname", port_name_);
    nh.getParam("inspire_hand/baudrate", baudrate_);
    nh.getParam("inspire_hand/test_flags", test_flags);

    //topic
    ros::Subscriber sub = nh.subscribe("/status_topic", 1000, callback);
    ros::spin();

    return (EXIT_SUCCESS);
}
