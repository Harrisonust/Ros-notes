# Ros custom message
## ROS message
is some data with a certain datatype that is sent and received between nodes through topics.

- ROS has a fuckton of builtin datatypes for certain things like position (PoseEstimate), lidar scans, etc.
## Primitive message types
|  Primitive Type   | C++ | Python |
|  ---------------  | --- | ------ |
| bool  | uint8_t | bool |
| uint32  | uint32_t | int |
| int32  | int32_t | int |
| float32  | float | float |
| string  | std::string | str |
| time  |  ||
| duration  |  ||


## .msg file
First we create a folder for message.
```
~/catkin_ws/			
└── src				
    ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake 
    ├── beginner_tutorials
    │   ├── CMakeLists.txt
    │   ├── include
    │   ├── package.xml		
    │   ├── msg		
    │   │   ├── my_msg.msg		
    │   ├── src		
    │   └── srv
```

```console
// my_msg.msg
int64 id
string title
string content
```
In package.xml:     
```xml 
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

In package/CMakeLists.txt:
```cmake
...
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)
...
catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES beginner_tutorials
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
    # DEPENDS system_lib
)
...
add_message_files(
    FILES
    my_msg.msg
)
...
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
```bash
$ catkin_make  # remember to run catkin_make in /catkin_ws to build the message
```

usage:
- cpp
    ```cpp
    #include "ros/ros.h"
    #include "beginner_tutorials/my_msg.h"
    int main(int argc, char **argv){
        ros::init(argc, argv, "talker");
        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<beginner_tutorials::my_msg>("chatter", 1000);
        ros::Rate loop_rate(10);
        int count = 0;
        while (ros::ok()){
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
    ```
- python
    ```python
    from beginner_tutorials.msg import my_msg
    msg = my_msg()
    ```

## .srv file
similarly we create a folder for srv structure

```console
$ roscd beginner_tutorials
$ mkdir srv
$ cd srv
$ vim my_srv.srv
```

my_srv.srv:

```txt
int64 id
---
string name
string gender
int64 age
```

in CMakeLists.txt:
```cmake
add_service_files(
    FILES
    my_srv.srv
)
```