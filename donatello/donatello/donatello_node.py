import rclpy
from rclpy.node import Node


def main():
    try:
        rclpy.init()
        node = Node('donatello',
                    allow_undeclared_parameters=True,
                    automatically_declare_parameters_from_overrides=True)
        logger = node.get_logger()
        for param in node._parameters:
            v = node.get_parameter(param).value
            logger.info(f'Parameter {param}={v}')
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
