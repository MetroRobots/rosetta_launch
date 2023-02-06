import rclpy
from rclpy.node import Node

running = True


def callback():
    global running
    running = False


def main():
    rclpy.init()
    node = Node('five_seconds')
    node.create_timer(5.0, callback)
    while running:
        rclpy.spin_once(node)

    rclpy.shutdown()
