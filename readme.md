### Passos de instalação  

```shell
mkdir robos_autonomos_t2_ws/src
cd robos_autonomos_t2_ws
source /opt/ros/noetic/setup.bash
catkin_make
```

```shell
cd robos_autonomos_t2_ws
source devel/setup.bash
```

### Diretorios base para o Jackal

- apos clonar os repositorios certifiquese de mudar para a branch correta, nos cassos onde não existir a branch "noetic" use a branche "melodic" essa orientação é valida para os repositorios da clearpath

```
cd robos_autonomos_t2_ws/src
git clone https://github.com/jackal/jackal.git
```

```
cd robos_autonomos_t2_ws/src
git clone https://github.com/clearpathrobotics/LMS1xx.git
```

```
cd robos_autonomos_t2_ws/src
git clone https://github.com/jackal/jackal_simulator.git
```

```
https://github.com/ros-visualization/interactive_marker_twist_server.git
```
Instalar dependências
```
$rosdep install --from-paths src --ignore-src -r -y
```
Resumo dos comandos

```
mkdir rat2_ws
cd rat2_ws
source /opt/ros/noetic/setup.bash
mkdir src
cd
cd rat2_ws
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/jackal/jackal.git
git clone https://github.com/clearpathrobotics/LMS1xx.git
git clone https://github.com/jackal/jackal_simulator.git
git clone https://github.com/ros-visualization/interactive_marker_twist_server.git
git clone https://github.com/jackal/jackal_desktop.git
cd
cd rat2_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

### Comandos para rodar ambiente

Terminal 1
```
cd robos_autonomos_t2_ws/
source devel/setup.bash
roslaunch jackal_gazebo jackal_world.launch config:=front_laser world:=world1

```

Terminal 2
```
cd robos_autonomos_t2_ws/
source devel/setup.bash
roslaunch jackal_viz view_robot.launch
```

Terminal 3
```
cd robos_autonomos_t2_ws/
source devel/setup.bash
rqt
```

Terminal 4
```
cd robos_autonomos_t2_ws/
source devel/setup.bash
rosrun explore_and_measure main.py
```


terminal unico
```
cd robos_autonomos_t2_ws/
source devel/setup.bash
roslaunch explore_and_measure area_measurement.launch world:=world2
```


### Referencias

- [Clearpath](https://www.clearpathrobotics.com/assets/guides/noetic/jackal/simulation.html0)
- [dinvincible98](https://github.com/dinvincible98/Jackal_ROS_Noetic_Bringup)
- [LaserScan msg](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) 
- [ConvexHull](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.ConvexHull.html)


### Rascunho

   <node pkg="explore_and_measure" type="main.py"
          name="explore" />

### Referencias estudos

https://github.com/ros/geometry_tutorials

