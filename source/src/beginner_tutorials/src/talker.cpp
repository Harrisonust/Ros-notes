#include "ros/ros.h"
#include "std_msgs/String.h" //define of msg structs are included here
#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker"); //node name
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    /**
    * @param  type: std_msg(the ros standard msg struct) 
    * @param  topic_name: chatter
    */
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {

        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world diudiudiu" << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str()); // stdout
        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}