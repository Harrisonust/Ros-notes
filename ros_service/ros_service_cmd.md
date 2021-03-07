## Ros Service notes

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