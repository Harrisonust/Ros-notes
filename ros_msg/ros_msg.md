## Ros self-defined message
### Common cmd for msg and srv
```console
$rosmsg show <package_name>/<msg_name>
$rosmsg info <package_name>/<msg_name>
$rossrv show <package_name>/<srv_name>
$rossrv info <package_name>/<srv_name>
```
### .msg file
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

In CMakeLists.txt:
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
generate_messages(
    DEPENDENCIES
    std_msgs
)
```
in package/CMakeLists:
```cmake
add_message_files(
    FILES
    my_msg.msg
)
```
```console
$ catkin_make
```

usage:
- cpp
    ```cpp
    #include <beginner_tutorials/my_msg.h>
    beginner_tutorials::my_msg msg;
    ```
- python
    ```python
    from beginner_tutorials.msg import my_msg
    msg = my_msg()
    ```

### .srv file
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