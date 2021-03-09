## Ros Service

### Ros Service common cmd
- **List all services**
    ```console
    $ rosservice list
    ```
    ```console
      /clear
      /kill
      /reset
      /rosout/get_loggers
      /rosout/set_logger_level
      /spawn
      /teleop_turtle/get_loggers
      /teleop_turtle/set_logger_level
      /turtle1/set_pen
      /turtle1/teleport_absolute
      /turtle1/teleport_relative
      /turtlesim/get_loggers
      /turtlesim/set_logger_level
    ```
- **Get detail of sepcific service**
    ```console
    $ rosservice type [service]
    $ rosservice type /spawn | rossrv show
    ```
    ```console 
      float32 x
      float32 y
      float32 theta
      string name
      ---
      string name
    ```

- **Call services**
    ```console
    $ rosservice call [service] [args]
    $ rosservice call /spawn 2 2 0.2 ""
    ```

### Rosparam
- **List all services with parameters**
    ```console
    $ rosservice list
    ```
    ```console
      /rosdistro
      /roslaunch/uris/host_nxt__43407
      /rosversion
      /run_id
      /turtlesim/background_b
      /turtlesim/background_g
      /turtlesim/background_r
    ```
- **Set services' parameter**
    ```console
    $ rosparam set [param_name]
    $ rosparam set /turtlesim/background_r 150
    ```
    You will need to call **clear service** to make refresh services.
    ```console
    $ rosservice call /clear  
    ```
- **Get services' parameter**
    ```console
    rosparam get [param_name]
    $ rosparam get /turtlesim/background_g 
    ```
    ```console
      86
    ```

    also you cloud use
    ```console 
    $ rosparam get /
    ```
    ```console
      rosdistro: 'noetic

      '
      roslaunch:
      uris:
      host_nxt__43407: http://nxt:43407/
      rosversion: '1.15.5

      '
      run_id: 7ef687d8-9ab7-11ea-b692-fcaa1494dbf9
      turtlesim:
      background_b: 255
      background_g: 86
      background_r: 69
    ```
- **Save services' parameters to files**
    ```console
    $ rosparam dump [file_name] [namespace]
    $ rosparam dump params.yaml
    ```
- **Load services' parameters to service from files**
    ```console
    $ rosparam load [file_name] [namespace]
    $ rosparam load params.yaml copy_turtle
    ```


### Make simple ros service(C++)
First of all you need to open a package of ~/catkin_ws/src/beginner_tutorials/src/add_two_ints_server.cpp

add_two_ints_server.cpp:
```c++
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res){
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
```

### Make simple ros client(C++)

add_two_ints_client.cpp:
```cpp
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv){
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3){
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv)){
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else{
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
```
CMakeLists.txt:
```cmake
......
# add all these at the end of CMakeLists.txt
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client beginner_tutorials_gencpp)
```

```console
$ cd ~/catkin_ws
$ catkin_make
```
### Make simple ros service(python)

add_two_ints_server.py:
```python
#!/usr/bin/env python
from __future__ import print_function

from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```
### Make simple ros client(python)
add_two_ints_client.py:
```python
#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
```

CMakeLists.txt:
```cmake
# add all these at the end of CMakeLists.txt
catkin_install_python(PROGRAMS scripts/add_two_ints_server.py scripts/add_two_ints_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### Examining the Simple Service and Client

First we run:
```console 
$ rosrun beginner_tutorials add_two_ints_server     # (C++)
$ rosrun beginner_tutorials add_two_ints_server.py  # (Python)
```

and run:
```console
$ rosrun beginner_tutorials add_two_ints_client 1 3     # (C++)
$ rosrun beginner_tutorials add_two_ints_client.py 1 3  # (Python)
```
you will see 
```console 
  Requesting 1+3
  1 + 3 = 4
```
