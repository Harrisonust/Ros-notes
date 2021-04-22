#Ros Parma
```xml
<launch>
    <param name="/print_frq" type="double" value="2.0" /> <!-- 定義一個全域變數給潛在的node使用 -->

    <node name="talker" pkg="beginner_tutorials" type="talker.py" /> <!-- 執行node -->
    <include file="rplidar.launch"> <!-- recursive執行node? -->  
        <arg name="lidar_dev" value="0"/> <!-- set lidar_dev = 0; -->
    </include>
</launch>
```


