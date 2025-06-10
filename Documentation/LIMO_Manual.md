# **Configuración de ROS2 LIMO**

## Objetivos de esta sección:

* Configurar el proyecto del robot en un entorno virtual para simulación.
* Configurar el proyecto del robot para el control en entorno real.
* Instalar las interfaces necesarias.

### Recurso utilizado:

* Robot comercial **LIMO**

![Robot LIMO](./Images/limo_robot.png)

### Webgrafía:

* [TheConstruct: Build Your First ROS2 Based Robot](https://www.robotigniteacademy.com/courses/309)
* [Repositorio LIMO (GitHub)](https://github.com/agilexrobotics/limo_ros2/tree/humble)
* [Manual de usuario LIMO (Foxy)](https://github.com/agilexrobotics/limo_pro_doc/blob/master/Limo%20Pro%20Ros2%20Foxy%20user%20manual%28EN%29.md)
* [Bitbucket LIMO (TheConstruct)](https://bitbucket.org/theconstructcore/limo_robot/src/main/)
* [Proyectos ROB (TheConstruct Bitbucket)](https://bitbucket.org/theconstructcore/workspace/projects/ROB)
* [Imagen Docker TheConstruct Humble-v3](https://hub.docker.com/r/theconstructai/limo/tags)
* [Repositorio Odometry Control - ROS2](https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control/tree/main)
* [Repositorio Odometry Control - ROS1](https://github.com/AntoBrandi/Self-Driving-and-ROS-Learn-by-Doing-Odometry-Control)
* [Repositorio Arduino-Bot (humble)](https://github.com/AntoBrandi/Arduino-Bot/tree/humble)

---

## **1. Configuración del proyecto del robot en entorno virtual para simulación**

### a) Simulación en TheConstruct

Utilizando la interfaz de TheConstruct, clona el siguiente repositorio:

```bash
git clone https://github.com/Mattyete/ROS2_LIMO_ws.git
cd ROS2_LIMO_ws
colcon build
source install/local_setup.bash
```

#### Añadir en `.bashrc` las siguientes líneas:

```bash
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=/home/user/ROS2_LIMO_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /home/user/ROS2_rUBot_tutorial_ws/install/setup.bash
source /home/user/ROS2_LIMO_ws/install/setup.bash
cd /home/user/ROS2_LIMO_ws
```

#### Si aparecen advertencias sobre `setuptools`:

```bash
sudo apt install python3-pip
pip3 list | grep setuptools
pip3 install setuptools==58.2.0
```

#### Si aparecen advertencias sobre `PREFIX_PATH`:

```bash
unset COLCON_PREFIX_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
cd ~/ROS2_LIMO_ws
rm -rf build/ install/ log/
colcon build
```

Abre una nueva terminal para que se vuelva a leer el `.bashrc`.

---

## **2. Configuración del proyecto del robot para control real**

Aquí se revisa el ordenador embarcado usado en el robot y el proceso de configuración diseñado.

La instalación se basa en un Docker personalizado para integrarse correctamente con ROS2.

### **2.1. Configuración del robot LIMO**

* Ordenador Jetson Nano embarcado.
* Uso de `Dockerfile` y `docker-compose` personalizados.

Cuando el robot LIMO se conecta, se ejecuta el servicio definido en `docker-compose-v3.yaml` y el robot queda listo para ser controlado desde TheConstruct.

Si falla el docker-compose-v3.yaml, parar el contenedor y seguir los pasos que están en https://hub.docker.com/r/theconstructai/limo. (Aplicar lo mismo pero con el archivo docker-compose-v3.yaml)

#### Desde un PC en la misma red:

* Conéctate por SSH con VSCode.
* Verifica los contenedores activos.
* Abre una nueva ventana de VSCode adjunta al contenedor LIMO.
* Para ver los tópicos de ROS:

```bash
source /limo_entrypoint-v3.sh
ros2 topic list
```

#### Desde el servicio RRL de TheConstruct:

* Instala en tu cuenta el robot LIMO.
* Conéctate al robot y en una nueva terminal escribe:

```bash
ros2 topic list
```
