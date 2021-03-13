# Ros custom message
## Custom message
TODO

## .msg file
first we create a folder for msg structure
```console 
$ roscd beginner_tutorials
$ mkdir msg
$ cd msg
$ vim my_msg.msg
```

```console
// ros_primitive_type
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
```console
$ catkin_make
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