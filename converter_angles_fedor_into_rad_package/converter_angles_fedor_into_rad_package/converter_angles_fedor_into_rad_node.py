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


TOPIC_PUBLISH = "Fedor_data_rad"
TOPIC_SUBSCRIBE = "bare_data"
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

LIMITS_OF_JOINTS_FEDOR_RAD_ANGLES = {
    0: [-0.43, 0.43],  # right_hip_roll_joint M
    1: [-3.14, 2.53],  # right_hip_pitch_joint M
    2: [-0.26, 2.05],  # right_knee_joint L
    3: [-0.43, 0.43],  # left_hip_roll_joint M
    4: [-3.14, 2.53],  # left_hip_pitch_joint M
    5: [0.26, 2.05],  # left_knee_joint L
    6: [-2.35, 2.35],  # torso_joint M
    7: [-0.43, 0.43],  # left_hip_yaw_joint M
    8: [-0.43, 0.43],  # right_hip_yaw_joint M
    9: [None, None],  # NOT USED
    10: [-0.87, 0.52],  # left_ankle_joint S
    11: [-0.87, 0.52],  # right_ankle_joint S
    12: [-1.9, 0.5],  # right_shoulder_pitch_joint M
    13: [-2.2, 0.0],  # right_shoulder_roll_joint M
    14: [-1.5, 1.3],  # right_shoulder_yaw_joint M
    15: [-0.5, 1.65],  # right_elbow_joint M
    16: [-1.9, 0.5],  # left_shoulder_pitch_joint M
    17: [0.0, 2.2],  # left_shoulder_roll_joint M
    18: [-1.3, 1.5],  # left_shoulder_yaw_joint M
    19: [-0.5, 1.65],  # left_elbow_joint M
    20: [0.0, 1.0],  # right_pinky
    21: [0.0, 1.0],  # right_ring
    22: [0.0, 1.0],  # right_middle
    23: [0.0, 1.0],  # right_index
    24: [0.0, 1.0],  # right_thumb-bend
    25: [0.0, 1.0],  # right_thumb-rotation
    26: [0.0, 1.0],  # left_pinky
    27: [0.0, 1.0],  # left_ring
    28: [0.0, 1.0],  # left_middle
    29: [0.0, 1.0],  # left_index
    30: [0.0, 1.0],  # left_thumb-bend
    31: [0.0, 1.0]  # left_thumb-rotation
}

LIMITS_OF_JOINTS_FEDOR = {
    0: [-12.0, 4.0],  # L_ShoulderF        [4.0, -12] + назад совпадают
    1: [0.0, 12.0],  # L_ShoulderS    [0, 12] + вверх от тела совпадают
    2: [-9.0, 9.0],  # L_ElbowR        [-9.0, 9.0] + против часовой совпадают
    3: [-12.0, 0, 0],  # L_Elbow                 [0, -12] + вниз совпадают
    4: [-3.820199, 6.866401],  # L_WristR
    6: [-2.501599, 3.0],  # L_WristF
    7: [-11.0, -1.5],  # L_Finger_Index       [0, -11] -согнут +разогнут
    9: [-11.0, -1.5],  # L_Finger_Middle      [0, -11]
    8: [-11.0, -1.5],  # L_Finger_Little      [0, -11]
    10: [-11.0, -1.5],   # L_Finger_Ring       [0, -11]
    11: [-3.0, 9.0],  # L_Finger_ThumbS     [-3, 9] -сжать, +разжать поворот
    12: [11.0, 1.5],  # L_inger_Thumb       [0, 11] сгибание
    13: [-12.0, 4.0],  # R_ShoulderF      [4.0, -12] + назад совпадают
    14: [-12.0, 0.0],  # R_ShoulderS        [0, -12] + вниз к телу совпадают
    15: [-9.0, 9.0],  # R_ElbowR            [-9.0, 9.0] + против часовой совпадают
    16: [-12.0, 0.0],  # R_Elbow            [0, -12]    + вниз совпадают
    17: [-11.0, 11.0],  # R_WristR
    18: [-2.0, 7.0],  # R_WristS
    19: [-1.734846, 3.000598],  # R_WristF
    20: [11.0, 1.5],   # R_Finger_Index      [0, 11] +согнут 0 разогнут
    21: [11.0, 1.5],   # R_Finger_Little     [0, 11] [0.595, 11]
    22: [11.0, 1.5],   # R_Finger_Middle     [0, 11] [0.0749, 11]
    23: [11.0, 1.5],   # R_Finger_Ring       [0, 11]
    24: [3.0, -9.0],  # R_Finger_ThumbS     [-9, 3] [-9, 1.173] +сжать -разжать
    25: [-11.0, -1.5],  # R_Finger_Thumb       [0, 11] [-0.034, 11]
}


def map_range(value: float,
              in_min: float,
              in_max: float,
              out_min: float,
              out_max: float
              ) -> float:
    """Преобразует значение из одного диапазона в другой."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def convert_to_unitree_h1(data: list) -> dict:
    """Конвертирует данные из условных единиц Федора в радианы."""
    output_targets = {}

    for i in range(0, len(data)):
        input_target = data[i]['target']
        index_in_fedor = i
        index_in_unitree_h1 = TRANSLATER_FOR_JOINTS_FROM_FEDOR_TO_UNITREE_H1[i]

        if index_in_unitree_h1 is not None:
            limits_of_this_joint_from_fedor = [
                LIMITS_OF_JOINTS_FEDOR[index_in_fedor][0],
                LIMITS_OF_JOINTS_FEDOR[index_in_fedor][1]]
            limits_of_this_joint_from_unitree_h1 = [
                index_in_fedor[index_in_unitree_h1][0],
                index_in_fedor[index_in_unitree_h1][1]]
            if (
                (limits_of_this_joint_from_fedor[0] is not None)
                and
                    (limits_of_this_joint_from_fedor[1] is not None)):
                a_fedor = limits_of_this_joint_from_fedor[0]
                b_febor = limits_of_this_joint_from_fedor[1]
                output_target = map_range(
                    np.clip(input_target, min(a_fedor, b_febor),
                            max(a_fedor, b_febor)),
                    limits_of_this_joint_from_fedor[0],
                    limits_of_this_joint_from_fedor[1],
                    limits_of_this_joint_from_unitree_h1[0],
                    limits_of_this_joint_from_unitree_h1[1])
            else:
                output_target = input_target

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
        convert_data[28] = convert_data[27]

        self.msg.data = json.dumps(convert_data)
        self.get_logger().debug(f'data = {(self.last_data)}')
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
