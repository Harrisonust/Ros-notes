# ROS BASIC

## What is ROS
TODO
## Ros Workspace Structure

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

## Ros command 

### Master
- ```roscore``` : Launch node manager. This command also launch other core pakcages.

### Node

- ```rosnode info <node_name>``` : Print out detial infomation of a node.

- ```rosnode kill <node_name>``` : Kill a running node.

- ```rosnode list``` : List all active nodes.

- ```rosnode machine <hostname>``` : List all active the nodes on local host.

- ```rosnode ping <node_name>``` : Test ping of a node.

- ```rosnode cleanup``` : Clear register status of unreachable nodes.

### Topic

- ``` rostopic bw </topic_name>``` : Show bandwidth of a topic.

- ```rostopic echo </topic_name>``` : Echo datas from a topic.

- ```rostopic find <message_type>``` : 用于按照消息类型查找主题。

- ```rostopic hz </topic_name>``` : Show publish freqency of a topic.

- ```rostopic info </topic_name>``` : Show Subscribers/Publishers/Services of a topic.

- ```rostopic list``` : Show active topics.

- ```rostopic pub </topic_name> <message_type> <args>``` : Publish data through command to topics.

- ```rostopic type </topic_name>``` : 用于输出主题中发布的消息类型。

### Message

- ```rosmsg show <message_type>``` : 用于显示一条消息的字段。 

- ```rosmsg list``` : List all the messages.用于列出所有消息。

- ```rosmsg package <package_name>``` : 用于列出功能包的所有消息。

- ```rosmsg packages``` : 用于列出所有具有该消息的功能包。

- ```rosmsg users <message_type>``` : 用于搜索使用该消息类型的代码文件。

- ```rosmsg md5 <message_type>``` : 用于显示一条消息的MD5求和结果。


### Service

- ```rosservice call </service_name> <args>``` : 用于通过命令行参数调用服务。

- ```rosservice find <service_type>``` : 用于根据服务类型查询服务。

- ```rosservice info </service_name>``` : 用于输出服务信息。

- ```rosservice list``` : 用于列出活动服务清单。

- ```rosservice type </service_name>``` : 用于输出服务类型。

- ```rosservice uri </service_name>``` : 用于输出服务的ROSRPC URI。


### Server

- ```rosparam list``` : 用于列出参数服务器中的所有参数。

- ```rosparam get <parameter_name>``` : 用于获取参数服务器中的参数值。

- ```rosparam set <parameter_name> <value>``` : 用于设置参数服务器中参数的值。

- ```rosparam delete <parameter_name>``` : 用于将参数从参数服务器中删除。

- ```rosparam dump <file>``` : 用于将参数服务器的参数保存到一个文件。

- ```rosparam load <file>``` : 用于从文件将参数加载到参数服务器。
### Other
- ```catkin_create_pkg <package_name> [depend1] [depend2] [depend3]```
- ``` cd ~/catkin_ws && catkin_make ```: Configure workspace
- ```rospack find [package name]```: Return the path of the package. 
	   
-  ```roscd [locationname[/subdir]] ```
   
- ```echo $ROS_PACKAGE_PATH``` 
   
-  ``` rosls [locationname[/subdir]] ```: List all the folders/files under the location
 
- ```rospack depends1 beginner_tutorials```: List ros package  depends


		
- ```rosrun [package_name] [node_name]```: Run ros nodes from cmd
	
- ```rosrun rqt_graph rqt_graph```: Print out the node graph
	

## CMakeLists
TODO
## PackageXml

### Description tag
```xml
<description>The beginner_tutorials package</description>
```

### Maintainer tag 
```xml
<maintainer email="harrison@todo.todo">harrison</maintainer>
```

### license tag
```xml
<license>BSD</license>
```

### dependency tag
```xml
<buildtool_depend>catkin</buildtool_depend>
<build_depend>roscpp</build_depend>
<build_depend>rospy</build_depend>
<build_depend>std_msgs</build_depend>
<exec_depend>roscpp</exec_depend>
<exec_depend>rospy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

### whole view
```xml
<?xml version="1.0"?>
<package format="2">
	<name>beginner_tutorials</name>
	<version>0.1.0</version>
	<description>The beginner_tutorials package</description>
	<maintainer email="you@yourdomain.tld">Your Name</maintainer>
	<license>BSD</license>
	<url type="website">http://wiki.ros.org/beginner_tutorials</url>
	<author email="you@yourdomain.tld">Jane Doe</author>
	<buildtool_depend>catkin</buildtool_depend>
	<build_depend>roscpp</build_depend>
	<build_depend>rospy</build_depend>
	<build_depend>std_msgs</build_depend>
	<exec_depend>roscpp</exec_depend>
	<exec_depend>rospy</exec_depend>
	<exec_depend>std_msgs</exec_depend>
</package>
```

