#include <ros/ros.h>
#include <hand_control.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <inspire_hand/angle_set.h>

int hand_id_;
std::string port_name_;
int baudrate_;
int test_flags;
int Serial_flags_;
serial::Serial *com_port_;
uint8_t hand_state_;

void callback(const inspire_hand::angle_setConstPtr &as)
{
    int add = 1486;
    std::vector<uint8_t> output;
    output.push_back(0xEB);
    output.push_back(0x90);
    output.push_back(hand_id_);
    output.push_back(0x0F);
    output.push_back(0x12);
    output.push_back(add & 0xFF);
    output.push_back((add >> 8) & 0xFF);

    output.push_back(as->angle1_set & 0xff);
    output.push_back((as->angle1_set >> 8) & 0xff);
    output.push_back(as->angle2_set & 0xff);
    output.push_back((as->angle2_set >> 8) & 0xff);
    output.push_back(as->angle3_set & 0xff);
    output.push_back((as->angle3_set >> 8) & 0xff);
    output.push_back(as->angle4_set & 0xff);
    output.push_back((as->angle4_set >> 8) & 0xff);
    output.push_back(as->angle5_set & 0xff);
    output.push_back((as->angle5_set >> 8) & 0xff);
    output.push_back(as->angle6_set & 0xff);
    output.push_back((as->angle6_set >> 8) & 0xff);

    //Checksum calculation
    unsigned int check_num = 0;
    int len = output[3] + 5;

    for (int i = 2; i < len - 1; i++)
        check_num = check_num + output[i];
    //Add checksum to the output buffer
    output.push_back(check_num & 0xff);

    //Send message to the module
    com_port_->write(output);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hand_control_setangle_subscriber");
    ros::NodeHandle nh;

    //Read launch file params
    nh.getParam("inspire_hand/hand_id", hand_id_);
    nh.getParam("inspire_hand/portname", port_name_);
    nh.getParam("inspire_hand/baudrate", baudrate_);
    nh.getParam("inspire_hand/test_flags", test_flags);

    com_port_ = new serial::Serial(port_name_, (uint32_t)baudrate_, serial::Timeout::simpleTimeout(100));

    nh.getParam("inspire_hand/Serial_flags", Serial_flags_);
    if(Serial_flags_ == 0)
    {
        ROS_INFO_STREAM("Serial is closed!");
    }
    else
    {
        //topic
        ros::Subscriber sub = nh.subscribe("/setangle_topic", 1000, callback);
        ros::spin();
    }

    return (EXIT_SUCCESS);
}
