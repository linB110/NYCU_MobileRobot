#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_topic_publisher");
    ros::NodeHandle node_obj;
    ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("user_input", 10);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        int inputValue = 0;  
        std::cin >> inputValue;

        std_msgs::Int32 msg;
        msg.data = inputValue;
        ROS_INFO("user's input is %d", msg.data);
        number_publisher.publish(msg);
    }
    return 0;
}

