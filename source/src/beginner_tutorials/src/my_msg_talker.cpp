#include "beginner_tutorials/my_msg.h"
#include "ros/ros.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<beginner_tutorials::my_msg>("chatter", 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
        beginner_tutorials::my_msg msg;
        msg.id = count;
        msg.title = "hello";
        msg.content = "hello from c++";
        ROS_INFO("%d", count); // stdout
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}