#!/usr/bin/env python3

'''
АННОТАЦИЯ
Данная ROS2 нода слушает сырые данные суставов робота FEDOR из топика 'bare_data',
конвертирует их в радианы и публикует обработанные данные в топик 'Fedor_data_rad'. 
Преобразование использует предопределенные соответствия суставов и 
масштабирование диапазонов.
Нода работает на частоте 333.3 Гц для работы в реальном времени.
'''

'''
ANNOTATION
This ROS2 node listens to raw joint data from the FEDOR robot 
on the 'bare_data' topic, converts it into radians 
and publishes the processed data to the 'Fedor_data_rad' topic. 
The conversion uses predefined joint mappings and range scaling to ensure proper actuation. 
The node runs at 333.3 Hz for real-time control applications.
'''

import json
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math


TOPIC_PUBLISH = "Fedor_data_rad"
TOPIC_SUBSCRIBE = "Fedor_bare_data"
FREQUENCY = 333.3  # Частота мониторинга в Герцах


TRANSLATER_FOR_JOINTS_FROM_FEDOR_TO_UNITREE_H1 = {
    0: 16,  # L.ShoulderF -> left_shoulder_roll_joint
    1: 17,  # L.ShoulderS -> left_shoulder_pitch_joint
    2: 18,  # L.ElbowR -> left_shoulder_yaw_joint
    3: 19,  # L.Elbow -> left_elbow_joint
    4: None,  # L.WristR
    5: None,  # L.WristS
    6: None,  # L.WristF
    7: 29,  # L.Finger.Index
    8: 26,  # L.Finger.Little
    9: 28,  # L.Finger.Middle
    10: 27,  # L.Finger.Ring
    11: 31,  # L.Finger.Thumb
    12: 30,  # L.Finger.Thumbs
    13: 12,  # R.ShoulderF -> right_shoulder_roll_joint
    14: 13,  # R.ShoulderS -> right_shoulder_pitch_joint
    15: 14,  # R.ElbowR -> right_shoulder_yaw_joint
    16: 15,  # R.Elbow -> right_elbow_joint
    17: None,  # R.WristR
    18: None,  # R.WristS
    19: None,  # R.WristF
    20: 23,  # R.Finger.Index
    21: 20,  # R.Finger.Little
    22: 22,  # R.Finger.Middle
    23: 21,  # R.Finger.Ring
    24: 25,  # R.Finger.Thumbs
    25: 24   # R.Finger.Thumb
}

# LIMITS_OF_JOINTS_FEDOR_RAD_ANGLES = {
#     0: [-0.43, 0.43],  # right_hip_roll_joint M
#     1: [-3.14, 2.53],  # right_hip_pitch_joint M
#     2: [-0.26, 2.05],  # right_knee_joint L
#     3: [-0.43, 0.43],  # left_hip_roll_joint M
#     4: [-3.14, 2.53],  # left_hip_pitch_joint M
#     5: [0.26, 2.05],  # left_knee_joint L
#     6: [-2.35, 2.35],  # torso_joint M
#     7: [-0.43, 0.43],  # left_hip_yaw_joint M
#     8: [-0.43, 0.43],  # right_hip_yaw_joint M
#     9: [None, None],  # NOT USED
#     10: [-0.87, 0.52],  # left_ankle_joint S
#     11: [-0.87, 0.52],  # right_ankle_joint S
#     12: [-1.9, 0.5],  # right_shoulder_pitch_joint M
#     13: [-2.2, 0.0],  # right_shoulder_roll_joint M
#     14: [-1.5, 1.3],  # right_shoulder_yaw_joint M
#     15: [-0.5, 1.65],  # right_elbow_joint M
#     16: [-1.9, 0.5],  # left_shoulder_pitch_joint M
#     17: [0.0, 2.2],  # left_shoulder_roll_joint M
#     18: [-1.3, 1.5],  # left_shoulder_yaw_joint M
#     19: [-0.5, 1.65],  # left_elbow_joint M
#     20: [0.0, 1.0],  # right_pinky
#     21: [0.0, 1.0],  # right_ring
#     22: [0.0, 1.0],  # right_middle
#     23: [0.0, 1.0],  # right_index
#     24: [0.0, 1.0],  # right_thumb-bend
#     25: [0.0, 1.0],  # right_thumb-rotation
#     26: [0.0, 1.0],  # left_pinky
#     27: [0.0, 1.0],  # left_ring
#     28: [0.0, 1.0],  # left_middle
#     29: [0.0, 1.0],  # left_index
#     30: [0.0, 1.0],  # left_thumb-bend
#     31: [0.0, 1.0]  # left_thumb-rotation
# }

KOEFFICIENT_OF_JOINTS_FEDOR = {
    # Руки (не пальцы) → pi/18
    0: math.pi / 18,  # L_ShoulderF
    1: math.pi / 18,  # L_ShoulderS
    2: math.pi / 18,  # L_ElbowR
    3: math.pi / 18,  # L_Elbow
    4: math.pi / 18,  # L_WristR
    5: math.pi / 18,  # L_WristS
    6: math.pi / 18,  # L_WristF
    13: math.pi / 18,  # R_ShoulderF
    14: math.pi / 18,  # R_ShoulderS
    15: math.pi / 18,  # R_ElbowR
    16: math.pi / 18,  # R_Elbow
    17: math.pi / 18,  # R_WristR
    18: math.pi / 18,  # R_WristS
    19: math.pi / 18,  # R_WristF

    # Пальцы → 11 или -11 (в зависимости от знака интервала)
    7: 1/-11,  # L_Finger_Index (интервал [-11, 0] → отрицательный)
    8: 1/-11,  # L_Finger_Little
    9: 1/-11,  # L_Finger_Middle
    10: 1/-11,  # L_Finger_Ring
    # L_Finger_ThumbS (интервал [-3, 9] → есть положительные значения)
    11: 1/12,
    12: 1/11,  # L_Finger_Thumb
    20: 1/11,  # R_Finger_Index (интервал [0, 11] → положительный)
    21: 1/11,  # R_Finger_Little
    22: 1/11,  # R_Finger_Middle
    23: 1/11,  # R_Finger_Ring
    # R_Finger_ThumbS (интервал [-9, 3] → есть отрицательные значения)
    24: 1/12,
    25: -1/11,  # R_Finger_Thumb (интервал [0, 11] → положительный)
}


def convert_to_unitree_h1(data: list) -> dict:
    """Конвертирует данные из условных единиц Федора в радианы, 
       учитывая коэффициенты преобразования."""
    output_targets = {}

    for i in range(len(data)):
        input_target = data[i]['target']
        index_in_fedor = i
        index_in_unitree_h1 = TRANSLATER_FOR_JOINTS_FROM_FEDOR_TO_UNITREE_H1[i]

        if index_in_unitree_h1 is not None:
            # Получаем коэффициент преобразования для текущего сустава
            coefficient = KOEFFICIENT_OF_JOINTS_FEDOR[index_in_fedor]

            # Применяем коэффициент к входному значению
            output_target = input_target * coefficient

            if index_in_fedor in (11, 24):
                output_target += 3/12

            if index_in_fedor == 20:
                output_target = 1 - output_target

            # Округляем до 2 знаков после запятой
            output_targets[index_in_fedor] = round(output_target, 2)

    return output_targets


class ConverterNode(Node):
    """ROS2 нода конвертации данных."""

    def __init__(self):
        super().__init__("converter_angles_fedor_into_rad_node")

        self.control_dt = 1 / FREQUENCY
        self.bare_data = ''
        self.last_data = None

        self.create_timer(self.control_dt, self.timer_callback)
        self.publisher = self.create_publisher(
            String,
            TOPIC_PUBLISH,
            10
        )

        self.subscription = self.create_subscription(
            String,
            TOPIC_SUBSCRIBE,
            self.bare_data_callback,
            10
        )

    def bare_data_callback(self, msg):
        """Получение данных необработанных данных с копирующего устройства"""
        self.bare_data = msg.data

    def timer_callback(self):
        """Публикация json конвертированного в радианы"""

        self.msg = String()
        if not self.bare_data:
            return
        try:
            response = json.loads(self.bare_data)
            formated_type = response['slaves']
        except Exception as e:
            self.get_logger().error(f"Error processing data: {e}")
            return

        # Convert the data to the format of unitree_h1
        convert_data = convert_to_unitree_h1(formated_type)
        # convert_data[28] = convert_data[27]

        self.msg.data = json.dumps(convert_data)
        self.get_logger().debug(f'data = {(self.msg.data)}')
        self.publisher.publish(self.msg)


def main(args=None):
    """Основная функция для запуска ноды."""
    rclpy.init(args=args)
    node = ConverterNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Stop node.')

    except Exception as e:
        node.get_logger().error(str(e))

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(args=None)
