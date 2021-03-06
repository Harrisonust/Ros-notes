		
### Ros topic
- **Get the msg from specific topic**
		
	```console
	rostopic echo [topic]
	```
	
	
- **Get info of specific topic**
  use the command:
  ```console 
  rostopic type [topic]
  rostopic type /turtle1/cmd_vel
  ```
  and it will return the msg type like:
  ```console
  geometry_msgs/Twist
  ```
  Further more if run command:
  ```console
  rosmsg show geometry_msgs/Twist
  ```
  ```console
	geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  ```
 - **Directly public messages to specific topics**
    ```console
      rostopic pub [topic] [msg_type] [args]
	  rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
	  ```