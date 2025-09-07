#include <ros/ros.h>
#include <hand_control.h>
#include <inspire_hand/status.h>

int hand_id_;
std::string port_name_;
int baudrate_;
int test_flags_;
int Serial_flags_;
int hand_state_;
serial::Serial *com_port_;
inspire_hand::status st;

void get3SET(serial::Serial *port)
{
    int add = 1486;
    std::vector<uint8_t> output;
    output.push_back(0xEB);
    output.push_back(0x90);
    output.push_back(hand_id_);
    output.push_back(0x04);
    output.push_back(0x11);
    output.push_back(add & 0xFF);
    output.push_back((add >> 8) & 0xFF);
    output.push_back(36);
    unsigned int check_num = 0;
    int len = output[3] + 5;
    for (int i = 2; i < output.size(); i++)
        check_num = check_num + output[i];
    //Add checksum to the output buffer
    output.push_back(check_num & 0xff);
    //Send message to the module
    port->write(output);

    ros::Duration(0.015).sleep();

    std::string s1;
    for (int i = 0; i < output.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", output[i]);
        s1 = s1 + str + " ";
    }
    if (test_flags_ == 1)
        ROS_INFO_STREAM("Write: " << s1);

    //Read response
    std::vector<uint8_t> input;
    while (input.empty())
    {
        port->read(input, (size_t)64);
    }

    std::string s2;
    for (int i = 0; i < input.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", input[i]);
        s2 = s2 + str + " ";
    }
    if (test_flags_ == 1)
        ROS_INFO_STREAM("Read: " << s2);

    int temp[18] = {0};
    for (int j = 0; j < 18; j++)
    {
        temp[j] = (input[7 + j*2]&0xFF) + ((input[8 + j*2]&0xFF) << 8);
    }
    st.angle1_set = temp[0];
    st.angle2_set = temp[1];
    st.angle3_set = temp[2];
    st.angle4_set = temp[3];
    st.angle5_set = temp[4];
    st.angle6_set = temp[5];
    st.force1_set = temp[6];
    st.force2_set = temp[7];
    st.force3_set = temp[8];
    st.force4_set = temp[9];
    st.force5_set = temp[10];
    st.force6_set = temp[11];
    st.speed1_set = temp[12];
    st.speed2_set = temp[13];
    st.speed3_set = temp[14];
    st.speed4_set = temp[15];
    st.speed5_set = temp[16];
    st.speed6_set = temp[17];
}

void get3ACT(serial::Serial *port)
{
    int add = 1546;
    std::vector<uint8_t> output;
    output.push_back(0xEB);
    output.push_back(0x90);
    output.push_back(hand_id_);
    output.push_back(0x04);
    output.push_back(0x11);
    output.push_back(add & 0xFF);
    output.push_back((add >> 8) & 0xFF);
    output.push_back(36);
    unsigned int check_num = 0;
    int len = output[3] + 5;
    for (int i = 2; i < output.size(); i++)
        check_num = check_num + output[i];
    //Add checksum to the output buffer
    output.push_back(check_num & 0xff);
    //Send message to the module
    port->write(output);

    ros::Duration(0.015).sleep();

    std::string s1;
    for (int i = 0; i < output.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", output[i]);
        s1 = s1 + str + " ";
    }
    if (test_flags_ == 1)
        ROS_INFO_STREAM("Write: " << s1);

    //Read response
    std::vector<uint8_t> input;
    while (input.empty())
    {
        port->read(input, (size_t)64);
    }

    std::string s2;
    for (int i = 0; i < input.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", input[i]);
        s2 = s2 + str + " ";
    }
    if (test_flags_ == 1)
        ROS_INFO_STREAM("Read: " << s2);

    int temp[18] = {0};
    for (int j = 0; j < 18; j++)
    {
        temp[j] = (input[7 + j*2]&0xFF) + ((input[8 + j*2]&0xFF) << 8);
    }
    st.angle1_act = temp[0];
    st.angle2_act = temp[1];
    st.angle3_act = temp[2];
    st.angle4_act = temp[3];
    st.angle5_act = temp[4];
    st.angle6_act = temp[5];
    st.force1_act = temp[6];
    st.force2_act = temp[7];
    st.force3_act = temp[8];
    st.force4_act = temp[9];
    st.force5_act = temp[10];
    st.force6_act = temp[11];
    st.current1 = temp[12];
    st.current2 = temp[13];
    st.current3 = temp[14];
    st.current4 = temp[15];
    st.current5 = temp[16];
    st.current6 = temp[17];
}

void get38BIT(serial::Serial *port)
{
    int add = 1606;
    std::vector<uint8_t> output;
    output.push_back(0xEB);
    output.push_back(0x90);
    output.push_back(hand_id_);
    output.push_back(0x04);
    output.push_back(0x11);
    output.push_back(add & 0xFF);
    output.push_back((add >> 8) & 0xFF);
    output.push_back(18);
    unsigned int check_num = 0;
    int len = output[3] + 5;
    for (int i = 2; i < output.size(); i++)
        check_num = check_num + output[i];
    //Add checksum to the output buffer
    output.push_back(check_num & 0xff);
    //Send message to the module
    port->write(output);

    ros::Duration(0.015).sleep();

    std::string s1;
    for (int i = 0; i < output.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", output[i]);
        s1 = s1 + str + " ";
    }
    if (test_flags_ == 1)
        ROS_INFO_STREAM("Write: " << s1);

    //Read response
    std::vector<uint8_t> input;
    while (input.empty())
    {
        port->read(input, (size_t)64);
    }

    std::string s2;
    for (int i = 0; i < input.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", input[i]);
        s2 = s2 + str + " ";
    }
    if (test_flags_ == 1)
        ROS_INFO_STREAM("Read: " << s2);

    int temp[18] = {0};
    for (int j = 0; j < 18; j++)
    {
        temp[j] = input[7 + j];
    }
    st.error1 = temp[0];
    st.error2 = temp[1];
    st.error3 = temp[2];
    st.error4 = temp[3];
    st.error5 = temp[4];
    st.error6 = temp[5];
    st.status1 = temp[6];
    st.status2 = temp[7];
    st.status3 = temp[8];
    st.status4 = temp[9];
    st.status5 = temp[10];
    st.status6 = temp[11];
    st.temp1 = temp[12];
    st.temp2 = temp[13];
    st.temp3 = temp[14];
    st.temp4 = temp[15];
    st.temp5 = temp[16];
    st.temp6 = temp[17];
}


int main(int argc, char** argv){

    ros::init(argc, argv, "hand_control_getstatus_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<inspire_hand::status>("/status_topic", 1);

    ros::Rate loop_rate(10);

    //Read launch file params
    nh.getParam("inspire_hand/hand_id", hand_id_);
    nh.getParam("inspire_hand/portname", port_name_);
    nh.getParam("inspire_hand/baudrate", baudrate_);
    nh.getParam("inspire_hand/test_flags", test_flags_);
    
    //Initialize and open serial port
    com_port_ = new serial::Serial(port_name_, (uint32_t)baudrate_, serial::Timeout::simpleTimeout(100));
    
    while(ros::ok()){

        nh.getParam("inspire_hand/Serial_flags", Serial_flags_);
        if(Serial_flags_ == 0)
        {
            ROS_INFO_STREAM("Serial is closed!");
        }
        else if (Serial_flags_ == 1)
        {
            get3SET(com_port_);
            get3ACT(com_port_);
            get38BIT(com_port_);

            pub.publish(st);
            loop_rate.sleep();
        }
    }
    return 0;
}