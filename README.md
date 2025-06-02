# Введение
Все действия, описанные здесь необходимо реализовывать на самом роботе.
# Зависимости
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
- pyrealsense2
- flask (python3)
# Установка зависимостей
## unitree_ros2
```bash 
git clone https://github.com/unitreerobotics/unitree_ros2
sudo apt install ros-foxy-rmw-cyclonedds-cpp
sudo apt install ros-foxy-rosidl-generator-dds-idl
```
Далее необходимо закомментировать в файле `~/.bashrc` строку `source /opt/ros/foxy/setup.bash` и перезапустим терминал.
Затем выполним следующие команды
```bash
cd ~/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
colcon build --packages-select cyclonedds 
```
После этого, необходимо расскомментировать в файле `~/.bashrc` строку `source /opt/ros/foxy/setup.bash` и перезапустить терминал вновь.
Далее выполним
```bash
cd ~/unitree_ros2/cyclonedds_ws
colcon build
```
После успешной компиляции добавим в файл `~/.bashrc` следующую сточку, которая позволит автоматизировать `source` пакетов с сообщениями
```bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
```
## unitree_sdk2_python
```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```
## pyrealsense2 и flask
```bash
pip install flask pyrealsense2
```
# Установка
```bash
cd ~
mkdir -p GQW_walking_robots_ws/src
cd GQW_walking_robots_ws/src
git clone https://github.com/cyberbanana777/GQW_walking_robots.git .
cd ..
colcon build
```
# Использование
```bash
source ~/GQW_walking_robots_ws/install/setup.bash
ros2 launch completed_scripts Fedor_teleop_with_hands.launch.py
```