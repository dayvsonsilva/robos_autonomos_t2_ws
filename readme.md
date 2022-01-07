### Passos de intalação

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

$rosdep install --from-paths src --ignore-src -r -y

### testar comandos

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

### Referencias

- [Clearpath](https://www.clearpathrobotics.com/assets/guides/noetic/jackal/simulation.html0)
- [dinvincible98](https://github.com/dinvincible98/Jackal_ROS_Noetic_Bringup)
