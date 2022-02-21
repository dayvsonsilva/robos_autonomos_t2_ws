### tf2 static broadcaster  

#### Bradcast fixo apenas com implementação de um arquivo launch com o pacote tf2_ros.

``` html
    <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="1 0 0 0 0 0 1 link1_parent link1" />
```

``` sh
cd robos_autonomos_t2_ws/
source devel/setup.bash
roslaunch learning_tf2 show_static_tf2_ros.launch
```

#### Broadcast fixo usando node python

``` sh
cd robos_autonomos_t2_ws/
source devel/setup.bash
rosrun learning_tf2 static_turtle_tf2_broadcaster.py  mystaticturtle 0 0 1 0 0 0
```  



### riting a tf2 broadcaster (Python)

Terminal 1
``` sh
cd robos_autonomos_t2_ws/
source devel/setup.bash
roslaunch learning_tf2 start_demo.launch
```  

Terminal 2
``` sh
cd robos_autonomos_t2_ws/
source devel/setup.bash
rosrun tf tf_echo /world /turtle1
```  








### Referências
[ROS Tutorials TF2](http://wiki.ros.org/tf2/Tutorials)
