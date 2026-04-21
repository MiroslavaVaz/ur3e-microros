#!/usr/bin/env python3
"""
button_input_node.py
--------------------
Subscribes to the /joint_button topic published by the ESP32 (micro-ROS).

Topic   : /joint_button
Msg type: std_msgs/Int32

Button mapping:
  PB1 -> 1  (Joint 1 / shoulder_pan_joint)
  PB2 -> 2  (Joint 2 / shoulder_lift_joint)
  PB3 -> 3  (Joint 3 / elbow_joint)
  PB4 -> 4  (Joint 4 / wrist_1_joint)
  PB5 -> 5  (Joint 5 / wrist_2_joint)
  PB6 -> 6  (Joint 6 / wrist_3_joint)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


# Human-readable names for each button/joint index (1-based)
JOINT_NAMES = {
    1: 'shoulder_pan_joint',
    2: 'shoulder_lift_joint',
    3: 'elbow_joint',
    4: 'wrist_1_joint',
    5: 'wrist_2_joint',
    6: 'wrist_3_joint',
}

VALID_BUTTONS = set(JOINT_NAMES.keys())  # {1, 2, 3, 4, 5, 6}


class ButtonInputNode(Node):
    """
    Listens for ESP32 pushbutton events on /joint_button and logs them.
    """

    def __init__(self):
        super().__init__('button_input_node')

        self._subscription = self.create_subscription(
            Int32,
            '/joint_button',
            self._button_callback,
            qos_profile=10,
        )

        self.get_logger().info(
            'ButtonInputNode started — listening on /joint_button'
        )

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------

    def _button_callback(self, msg: Int32) -> None:
        """Called every time a message arrives on /joint_button."""
        button_id = msg.data

        if button_id not in VALID_BUTTONS:
            self.get_logger().warn(
                f'Received unknown button value: {button_id} '
                f'(expected 1–6). Ignoring.'
            )
            return

        joint_name = JOINT_NAMES[button_id]
        self.get_logger().info(
            f'Button pressed: PB{button_id}  →  joint "{joint_name}"  '
            f'(raw value: {button_id})'
        )


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    node = ButtonInputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()