#include <ros/ros.h>
#include <hand_control.h>
#include <inspire_hand/angle_set.h>

int hand_id_;
std::string port_name_;
int baudrate_;
int test_flags_;
int Serial_flags_;
int hand_state_;
serial::Serial *com_port_;
inspire_hand::angle_set as;

int main(int argc, char** argv){

    ros::init(argc, argv, "hand_control_setangle_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<inspire_hand::angle_set>("/setangle_topic", 1);

    ros::Rate loop_rate(10);

    //Read launch file params
    nh.getParam("inspire_hand/hand_id", hand_id_);
    nh.getParam("inspire_hand/portname", port_name_);
    nh.getParam("inspire_hand/baudrate", baudrate_);
    nh.getParam("inspire_hand/test_flags", test_flags_);
    
    //Initialize and open serial port
    com_port_ = new serial::Serial(port_name_, (uint32_t)baudrate_, serial::Timeout::simpleTimeout(100));
    
    as.angle1_set = 999;
    as.angle2_set = 999;
    as.angle3_set = 999;
    as.angle4_set = 999;
    as.angle5_set = -1;
    as.angle6_set = -1;
    int direction = -1;

    while(ros::ok()){

        nh.getParam("inspire_hand/Serial_flags", Serial_flags_);
        if(Serial_flags_ == 0)
        {
            ROS_INFO_STREAM("Serial is closed!");
        }
        else if (Serial_flags_ == 1)
        {
            as.angle1_set += direction*30;
            as.angle2_set += direction*30;
            as.angle3_set += direction*30;
            as.angle4_set += direction*30;
            if(as.angle1_set <= 1)
            {
                direction = 1;
            }
            else if(as.angle1_set >= 999)
            {
                direction = -1;
            }

            pub.publish(as);
            loop_rate.sleep();
        }
    }
    return 0;
}