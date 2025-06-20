#!/usr/bin/env python3

'''
АННОТАЦИЯ
Этот код представляет собой ROS2-ноду для преобразования и публикации данных 
о суставах робота. Нода подписывается на топик с данными углов суставов 
копирующего устройства (Fedor), преобразует их в соответствующие суставы 
робота Unitree H1 и публикует выбранный сустав в топик для визуализации 
в PlotJuggler. Основная логика включает таймер для периодической публикации 
и обработку входящих JSON-сообщений. Код поддерживает настройку целевого 
сустава через параметр ROS2.
'''

'''
ANNOTATION
This code is a ROS2 node for converting and publishing robot joint data. 
The node subscribes to a topic containing joint angle data from the Fedor 
master device, converts them to corresponding joints of the Unitree H1 robot, 
and publishes the selected joint to a topic for visualization in PlotJuggler. 
The core logic includes a timer for periodic publishing and processing of 
incoming JSON messages. The code supports configuration of the target joint 
via a ROS2 parameter.
'''

from unitree_go.msg import LowState
from unitree_go.msg import MotorStates
from std_msgs.msg import Float32
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
import json


TOPIC_PUBLISH_GROUP = "plotjuggler/joint_"
TOPIC_SUBSCRIBE = "Fedor_bare_data"
FREQUENCY = 333.3  # Частота мониторинга в Герцах

TRANSLATER_FOR_JOINTS_UNITREE_H1_TO_FEDOR = {
    16: 0,   # left_shoulder_roll_joint -> L.ShoulderF
    17: 1,   # left_shoulder_pitch_joint -> L.ShoulderS
    18: 2,   # left_shoulder_yaw_joint -> L.ElbowR
    19: 3,   # left_elbow_joint -> L.Elbow
    29: 7,   # L.Finger.Index
    26: 8,   # L.Finger.Little
    28: 9,   # L.Finger.Middle
    27: 10,  # L.Finger.Ring
    31: 11,  # L.Finger.Thumb
    30: 12,  # L.Finger.Thumbs
    12: 13,  # right_shoulder_roll_joint -> R.ShoulderF
    13: 14,  # right_shoulder_pitch_joint -> R.ShoulderS
    14: 15,  # right_shoulder_yaw_joint -> R.ElbowR
    15: 16,  # right_elbow_joint -> R.Elbow
    23: 20,  # R.Finger.Index
    20: 21,  # R.Finger.Little
    22: 22,  # R.Finger.Middle
    21: 23,  # R.Finger.Ring
    25: 24,  # R.Finger.Thumbs
    24: 25   # R.Finger.Thumb
}


class ExtractorNode(Node):
    """
    ROS2 нода для выбора joint unitree H1 и публикации соответсвенного 
    значения, идущего с копирующего устройства
    """

    def __init__(self):
        super().__init__("extractor_node")

        self.impact = 1.0
        self.time_for_return_control = 8.0
        self.control_dt = 1 / FREQUENCY
        self.joint_Fedor_angle_value = 0.0
        self.H1_joint_angle_value = 0.0

        # Объявление параметра со значением по умолчанию
        self.declare_parameter('H1_joint_num', 16)

        # Получение значения параметра
        self.H1_joint_num_value = self.get_parameter('H1_joint_num').value
        self.get_logger().info(
            f'Parameter "H1_joint_num" value: {self.H1_joint_num_value}')

        self.create_timer(self.control_dt, self.timer_callback)

        # Публикация угла выбранного джоинта Федор
        self.publisher_fedor = self.create_publisher(
            Float32,
            TOPIC_PUBLISH_GROUP + str(self.H1_joint_num_value) + '/fedor',
            10
        )

        # Публикация угла выбранного джоинта H1
        self.publisher_H1 = self.create_publisher(
            Float32,
            TOPIC_PUBLISH_GROUP + str(self.H1_joint_num_value) + '/h1',
            10
        )

        # Создание объекта сообщения для публикации
        self.msg_float = Float32()

        # Подписка на топик, в который публикуется информацию об углах поворота звеньев копирующего устройства в радианах
        self.subscription_LowCmd = self.create_subscription(
            String,
            TOPIC_SUBSCRIBE,
            self.listener_Fedor_data_rad,
            10
        )

        # Подписка на топик, в который публикуется информацию об степени сжаия пальцев H1
        self.subscription_state = self.create_subscription(
            MotorStates,
            'inspire/state',
            self.listener_callback_states,
            10
        )

        # подписка на топик, в который публикуется информация о углах поворота звеньев H1
        self.subscription_LowCmd = self.create_subscription(
            LowState,
            'lowstate',
            self.listener_callback_LowCmd,
            10
        )

    def timer_callback(self):
        """Публикация угла выбранного джоинта в радианах"""
        msg_float = Float32()
        msg_float.data = float(self.joint_Fedor_angle_value)
        self.publisher_fedor.publish(msg_float)

        msg_float = Float32()
        msg_float.data = float(self.H1_joint_angle_value)
        self.publisher_H1.publish(msg_float)

        # self.get_logger().info(f'Tracking joint: {self.H1_joint_num_value}')

    def listener_Fedor_data_rad(self, msg):
        """Получение угла выбранного джоинта в радианах"""
        data = json.loads(msg.data)
        joint_fedor = TRANSLATER_FOR_JOINTS_UNITREE_H1_TO_FEDOR[self.H1_joint_num_value]

        data_to_send = data['slaves'][joint_fedor]['target']
        self.get_logger().info(f'{data_to_send}')

        # if str(joint_fedor) not in data:
        #     self.get_logger().warn(
        #         f"Joint {joint_fedor} not found in incoming data")
        #     return

        # self.joint_Fedor_angle_value = data[str(joint_fedor)]
        # self.get_logger().debug(f'{self.joint_Fedor_angle_value}')

    def listener_callback_LowCmd(self, msg):
        """Получение угла выбранного джоинта H1"""
        if self.H1_joint_num_value < 20:
            try:
                self.H1_joint_angle_value = msg.motor_state[self.H1_joint_num_value].q
                self.get_logger().info(f'{self.H1_joint_angle_value}')
            except Exception as e:
                self.get_logger().warn(
                    f"Joint {self.H1_joint_num_value} not found in incoming data")
        else:
            return

    def listener_callback_states(self, msg):
        """Получение угла выбранного джоинта кисти H1"""
        if self.H1_joint_num_value >= 20:
            try:
                self.H1_joint_angle_value = msg.states[self.H1_joint_num_value - 20].q
                self.get_logger().info(f'{self.H1_joint_angle_value}')
            except Exception as e:
                self.get_logger().warn(
                    f"Joint {self.H1_joint_num_value} not found in incoming data")
        else:
            return


def main(args=None):
    """Основная функция для запуска ноды."""
    rclpy.init(args=args)
    node = ExtractorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Stop node.')

    except Exception as e:
        node.get_logger().error(str(e))

    finally:
        node.get_logger().info('Node stoped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(args=None)
