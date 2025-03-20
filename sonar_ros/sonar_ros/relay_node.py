#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('relay_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.get_logger().info('Ready to receive commands (RON to switch ON, ROFF to switch OFF)')

        self.current_command = None
        
        # 명령어 요청 스레드 시작
        self.input_thread = threading.Thread(target=self.ask_for_input, daemon=True)
        self.input_thread.start()

        # 주기적으로 명령어 전송 스레드 시작
        self.send_thread = threading.Thread(target=self.send_command, daemon=True)
        self.send_thread.start()

    def ask_for_input(self):
        while rclpy.ok():
            self.current_command = input("Enter command (RON to switch ON, ROFF to switch OFF): ").strip()

    def send_command(self):
        while rclpy.ok():
            if self.current_command is not None:
                if self.current_command == 'ON':
                    self.serial_port.write(b'DO\n')
                elif self.current_command == 'OFF':
                    self.serial_port.write(b'DF\n')
                else:
                    self.get_logger().warning('Invalid command. Please enter "ON" or "OFF".')
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()

    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        serial_node.get_logger().info('Exiting...')
    finally:
        serial_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()