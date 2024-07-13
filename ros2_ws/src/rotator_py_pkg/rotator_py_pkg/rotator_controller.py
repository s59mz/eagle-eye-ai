#!/usr/bin/env python3

import rclpy
import serial

from rclpy.node import Node

from rotator_interfaces.msg import MotorCmd, SwitchCmd


class RotatorControllerNode(Node):
    def __init__(self):
        super().__init__("rotator_controller")

        # Declare parameters for serial communication
        self.declare_parameter("port", "/dev/ttyUL0")
        self.declare_parameter("baudrate", "9600")
        self.declare_parameter("address", 1)

        # Retrieve parameter values
        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value
        self.address_ = self.get_parameter("address").value

        # Open serial port for communication with the RS485 dongle
        self.serial_ = self.open_serial_port(self.port_, self.baudrate_)

        # Initialize Pelco-D message buffer
        self.pelcod_message_ = self.create_pelco_d_message()

        # Create subscribers for motor control and switch control topics
        self.sub_motor_control = self.create_subscription(
            MotorCmd, "motor_control", self.callback_motor_control, 10
        )
        self.sub_switch_control = self.create_subscription(
            SwitchCmd, "switch_control", self.callback_switch_control, 10
        )

        self.get_logger().info("Node Created")

    def callback_switch_control(self, switch_cmd: SwitchCmd):
        # Update Pelco-D message based on switch control command
        self.create_pelco_d_message()
        self.update_switch(switch_cmd.switch_on)
        self.update_checksum()
        self.send_message()

    def callback_motor_control(self, motor_cmd: MotorCmd):
        # Update Pelco-D message based on motor control command
        self.create_pelco_d_message()
        self.update_pan_speed(motor_cmd.pan_speed)
        self.update_tilt_speed(motor_cmd.tilt_speed)
        self.update_checksum()
        self.send_message()

    def update_switch(self, switch_on: bool):
        # Update Pelco-D message to control switch
        if switch_on:
            cmd2 = 0x09  # Command to turn switch on
        else:
            cmd2 = 0x0b  # Command to turn switch off

        self.pelcod_message_[3] = 0xff & cmd2
        self.pelcod_message_[5] = 0x01  # Auxiliary command for switch control

    def update_pan_speed(self, speed: int):
        # Update Pelco-D message with pan speed command
        cmd2 = self.pelcod_message_[3]
        cmd2 = cmd2 & ~0x06

        if speed > 0:   # Pan left
            cmd2 = cmd2 | 0x04
        elif speed < 0: # Pan right
            cmd2 = cmd2 | 0x02
            speed *= -1  # Convert negative speed to positive for command

        self.pelcod_message_[3] = 0xff & cmd2
        if speed > 0x3F:
            speed = 0xff  # Limit speed if necessary

        self.pelcod_message_[4] = 0xff & speed

    def update_tilt_speed(self, speed: int):
        # Update Pelco-D message with tilt speed command
        cmd2 = self.pelcod_message_[3]
        cmd2 = cmd2 & ~0x18

        if speed > 0:   # Tilt up
            cmd2 = cmd2 | 0x08
        elif speed < 0: # Tilt down
            cmd2 = cmd2 | 0x10
            speed *= -1  # Convert negative speed to positive for command

        self.pelcod_message_[3] = 0xff & cmd2
        if speed > 0x3F:
            speed = 0x3F  # Limit speed if necessary

        self.pelcod_message_[5] = 0xff & speed

    def update_checksum(self):
        # Calculate and update checksum of Pelco-D message
        checksum = 0

        for i in range(1, 6):
            checksum += self.pelcod_message_[i]

        self.pelcod_message_[6] = 0xFF & checksum

    def create_pelco_d_message(self):
        # Initialize Pelco-D message structure
        self.pelcod_message_ = bytearray([0xff, self.address_, 0x00, 0x00, 0x00, 0x00, 0x00])

    def send_message(self):
        # Send Pelco-D message via serial port
        self.serial_.write(self.pelcod_message_)

    def open_serial_port(self, port, baudrate=9600, timeout=1):
        # Open serial port for communication
        try:
            ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            self.get_logger().info(f"Serial port {port} opened successfully.")
            return ser

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            return None

    def close_serial_port(self):
        # Close serial port
        self.serial_.close()
        self.serial_ = None
        self.pelcod_message_ = None

        self.get_logger().info(f"Serial port {self.port_} is now closed.")


def main(args=None):
    rclpy.init(args=args)

    node = RotatorControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.close_serial_port()
        return

    node.close_serial_port()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
