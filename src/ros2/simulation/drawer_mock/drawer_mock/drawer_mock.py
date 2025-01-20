import rclpy
from rclpy.node import Node
from communication_interfaces.msg import DrawerAddress, StateFeedback

MANUAL_DRAWER_OPEN_TIME_IN_S = 3


class DrawerMock(Node):
    def __init__(self) -> None:
        super().__init__("drawer_mock")

        self.__state_feedback_publisher = self.create_publisher(
            StateFeedback, "/module_state_update", 10
        )

        self.create_subscription(
            DrawerAddress,
            "trigger_drawer_tree",
            self.__drawer_tree_callback,
            10,
        )
        self.create_subscription(
            DrawerAddress,
            "trigger_electric_drawer_tree",
            self.__electric_drawer_tree_callback,
            10,
        )
        self.create_subscription(
            DrawerAddress,
            "trigger_partial_drawer_tree",
            self.__electric_drawer_tree_callback,
            10,
        )
        self.create_subscription(
            DrawerAddress,
            "close_drawer",
            self.__close_drawer_callback,
            10,
        )

        self.__is_any_drawer_open = False
        self.__timer = None

    def __drawer_tree_callback(self, msg: DrawerAddress) -> None:
        if not self.__is_any_drawer_open:
            self.__is_any_drawer_open = True
            module_id, drawer_id = self.__read_drawer_address(msg)
            self.get_logger().info(f"Opening manual drawer: {module_id}, {drawer_id}")
            msg = self.__create_drawer_status_msg(
                module_id, drawer_id, drawer_is_open=True
            )
            self.__state_feedback_publisher.publish(msg)
            self.__timer = self.create_timer(
                MANUAL_DRAWER_OPEN_TIME_IN_S,
                lambda: self.__manual_close_mock_callback(module_id, drawer_id),
            )

    def __manual_close_mock_callback(self, module_id: int, drawer_id: int) -> None:
        self.__timer.cancel()
        self.get_logger().info(f"Closing manual drawer: {module_id}, {drawer_id}")
        msg = self.__create_drawer_status_msg(
            module_id, drawer_id, drawer_is_open=False
        )
        self.__state_feedback_publisher.publish(msg)
        self.__is_any_drawer_open = False

    def __electric_drawer_tree_callback(self, msg: DrawerAddress) -> None:
        if not self.__is_any_drawer_open:
            self.__is_any_drawer_open = True
            module_id, drawer_id = self.__read_drawer_address(msg)
            self.get_logger().info(f"Opening electric drawer: {module_id}, {drawer_id}")
            msg = self.__create_drawer_status_msg(
                module_id, drawer_id, drawer_is_open=True
            )
            self.__state_feedback_publisher.publish(msg)

    def __close_drawer_callback(self, msg: DrawerAddress) -> None:
        if self.__is_any_drawer_open:
            module_id, drawer_id = self.__read_drawer_address(msg)
            self.get_logger().info(f"Closing electric drawer: {module_id}, {drawer_id}")
            msg = self.__create_drawer_status_msg(
                module_id, drawer_id, drawer_is_open=False
            )
            self.__state_feedback_publisher.publish(msg)
            self.__is_any_drawer_open = False

    def __read_drawer_address(self, msg: DrawerAddress) -> tuple[int, int]:
        return msg.module_id, msg.drawer_id

    def __create_drawer_status_msg(
        self, module_id: int, drawer_id: int, drawer_is_open: bool
    ) -> StateFeedback:
        msg = StateFeedback()
        msg.drawer_address.module_id = module_id
        msg.drawer_address.drawer_id = drawer_id
        msg.state = "open" if drawer_is_open else "closed"
        return msg


def main(args=None):
    rclpy.init(args=args)

    drawer_mock = DrawerMock()

    rclpy.spin(drawer_mock)

    drawer_mock.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
