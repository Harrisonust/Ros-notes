		
# Ros "topic"
## What is "Topic" in Ros
is like a subreddit, it's a place where nodes can publish new data, and subscribe to recieve data published by others.

- This includes everything from positioning estimates to lidar scans or camera images.
- A topic has a name so that nodes can refer to it. A node will usually have a default name for any data it publishes/subscribes, and either use launch arguments to customize the name, or ROS supports redirection by adding certain command line arguments.
- A ROS topic will usually have a specific datatype, but only by convention, technically nothing enforces this.
## Prepare
The current workspace structure:
```
~/catkin_ws/			-- WORKSPACE
└── src				-- SOURCE SPACE
    ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake 
	 			-- 'Toplevel' CMake file, provided by catkin
    ├── beginner_tutorials
    │   ├── CMakeLists.txt	-- CMakeLists.txt file for package_1
    │   ├── include
    │   ├── package.xml		-- Package manifest for package_1
    │   └── src
	....
    └── package_n
        ├── CMakeLists.txt	-- CMakeLists.txt file for package_n
        ├── include
        ├── package.xml		-- Package manifest for package_n
        └── src

```

Now we do:
```bash
$ cd ~/catkin_ws/src/package_1/src 
```
and from now on we write all the source codes in that folder.  
## Publisher and Subscriber (C++)
talker.cpp:
``` cpp
#include "ros/ros.h"
#include "std_msgs/String.h" //define of msg structs are included here
#include <sstream>
   
int main(int argc, char **argv){

    ros::init(argc, argv, "talker"); //node name
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    /**
    * @param  type: std_msg(the ros standard msg struct) 
    * @param  topic_name: chatter
    */
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()){
        
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str()); // stdout
        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
// ros::ok() will return false if any situation happening;
//  1. Ctrl+C
//  2. another node with same node name is invoked
//  3. ros::shutdown() is called somewhere else in the program
//  4. all ros::NodeHandles are destroyed
```

listener.cpp:
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
  
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback); 
    // invoke callback function whenever receiver msg
    // from the subscribing topics

    ros::spin();
    return 0;
}
```
package/CMakeLists.txt:
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
## add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
## add_dependencies(listener beginner_tutorials_generate_messages_cpp)

## catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py
##   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
## )
```
Remember to run ```catkin_make``` after finishing the CMakeLists.txt
```bash 
$ cd ~/catkin_ws
$ catkin_make
```

## Publisher and Subscriber (Python)

talker.py:
```python
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

listerner.py:
```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```
remember also run catkin_make even we are using python
```bash 
$ cd ~/catkin_ws
$ catkin_make
```

## Examining Publishers and Subscribers

```bash
$ roscore //make sure roscore(master node ) is running
$ source ~/catkin_ws/devel/setup.bash
```
to run talkers, use the following command:
```bash
$ rosrun beginner_tutorials talker      # (C++)
$ rosrun beginner_tutorials talker.py   # (Python)
```
and you will see the output:
```bash
  [INFO] [WallTime: 1314931831.774057] hello world 1314931831.77
  [INFO] [WallTime: 1314931832.775497] hello world 1314931832.77
  [INFO] [WallTime: 1314931833.778937] hello world 1314931833.78
  [INFO] [WallTime: 1314931834.782059] hello world 1314931834.78
  [INFO] [WallTime: 1314931835.784853] hello world 1314931835.78
  [INFO] [WallTime: 1314931836.788106] hello world 1314931836.79
```

to receive msg from talkers, we run listerners by following command:
```bash
$ rosrun beginner_tutorials listener     # (C++)
$ rosrun beginner_tutorials listener.py  # (Python) 
```

and you will see it is receiving msg from the talkers correctly:
```bash 
  [INFO] [WallTime: 1314931969.258941] /listener_17657_1314931968795I heard hello world 1314931969.26
  [INFO] [WallTime: 1314931970.262246] /listener_17657_1314931968795I heard hello world 1314931970.26
  [INFO] [WallTime: 1314931971.266348] /listener_17657_1314931968795I heard hello world 1314931971.26
  [INFO] [WallTime: 1314931972.270429] /listener_17657_1314931968795I heard hello world 1314931972.27
  [INFO] [WallTime: 1314931973.274382] /listener_17657_1314931968795I heard hello world 1314931973.27
  [INFO] [WallTime: 1314931974.277694] /listener_17657_1314931968795I heard hello world 1314931974.28
  [INFO] [WallTime: 1314931975.283708] /listener_17657_1314931968795I heard hello world 1314931975.28
```