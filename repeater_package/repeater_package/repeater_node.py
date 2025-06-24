#!/usr/bin/env python3

'''
АННОТАЦИЯ
Этот код представляет собой ROS2-ноду (UDPRepeaterNode), которая прослушивает
UDP-сокет на указанном IP и порту, получает данные в формате JSON и публикует 
их в ROS2-топик "Fedor_bare_data" с частотой 333.3 Гц. Нода корректно 
обрабатывает ошибки, логирует полученные данные и обеспечивает безопасное 
завершение работы, включая закрытие сокета и освобождение ресурсов ROS2.
'''

'''
ANNOTATION
This code is a ROS2 node (UDPRepeaterNode) that listens on a UDP socket on the
specified IP and port, receives data in JSON format, and publishes it to the 
ROS2 topic "Fedor_bare_data" at 333.3 Hz. The node handles errors gracefully,
logs the received data, and ensures safe shutdown, including closing the socket
and freeing up ROS2 resources.
'''

from rclpy.node import Node
from std_msgs.msg import String
import rclpy
import socket
import json

HOST = '192.168.123.162'
PORT = 34567
DATA_PAYLOAD = 2000
TOPIC = "Fedor_bare_data"
FREQUENCY = 333.3  # Частота мониторинга в Герцах


class UDPRepeaterNode(Node):
    """ROS2 нода для прослушивания UDP-сокета пересылки полученных данных в топик."""

    def __init__(self):
        super().__init__("udp_repeater_node")

        self.control_dt = 1 / FREQUENCY

        # Create a UDP socket
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind((HOST, PORT))

        # Status information
        self.get_logger().info(f"Repeater node is listening {HOST}:{PORT}")
        self.get_logger().info(
            f"Repeater node publishing in '/{TOPIC}' ROS2-topic")

        self.create_timer(self.control_dt, self.timer_callback)
        self.publisher = self.create_publisher(String, TOPIC, 10)

        self.msg = String()
        self.last_data = None

    def timer_callback(self):
        """Обратный вызов таймера для обработки данных."""
        try:
            data, address = self.s.recvfrom(DATA_PAYLOAD)

        except Exception as e:
            self.get_logger().error(f"Error processing data: {e}")
            return

        response = json.loads(data)
        self.last_data = json.dumps(response)
        self.msg.data = self.last_data
        self.get_logger().debug(f'data = {(self.last_data)}')
        self.publisher.publish(self.msg)


def main(args=None):
    """Основная функция для запуска ноды."""
    rclpy.init(args=args)
    node = UDPRepeaterNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Stop node.')

    except Exception as e:
        node.get_logger().error(e)

    finally:
        node.s.close()
        node.get_logger().info('Socket closed.')
        node.get_logger().info('Node stoped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(args=None)
