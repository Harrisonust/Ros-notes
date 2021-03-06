
### Ros Structure
```console
  workspace_folder/        -- WORKSPACE
      src/                   -- SOURCE SPACE
        CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
        package_1/
          CMakeLists.txt     -- CMakeLists.txt file for package_1
          package.xml        -- Package manifest for package_1
        ...
        package_n/
          CMakeLists.txt     -- CMakeLists.txt file for package_n
          package.xml        -- Package manifest for package_n
```
### Ros Common Command
- **Create ros package** 
	cmd: catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
	```console
	 $ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp 
	 ```
- **Configure workspace**
    ```console
	$ cd ~/catkin_ws
	$ catkin_make
	```
- **Find package**
	```console 
	$ rospack find [package name]
	```
	return the path of the package. ex: opt/ros/noetic/share/roscpp
   
- **Change dir**
    ```console
	$ roscd [locationname[/subdir]] 
	```
   
- **List all ros package path**  
    ```console
    $ echo $ROS_PACKAGE_PATH
    ```
   
- **Ros ls**
    ```console 
	$ rosls [locationname[/subdir]] 
	```
    list all the folders/files under the location
 
- **List ros package  depends**
	```console
	$ rospack depends1 beginner_tutorials
	```
	```console
	   std_msgs
	   rospy
	   roscpp
	```
- **List ros nodes**
	```console
	$ rosnode list
	```
		
- **Run ros nodes from cmd**
	```console
	rosrun [package_name] [node_name]
	```
- **Print out the node graph**
	```console
	rosrun rqt_graph rqt_graph
	```

### Package.xml
```console
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
