import rclpy

from heiligenhafen_commander.heiligenhafen_commander import HeiligenhafenCommander


def main():
    rclpy.init()

    heiligenhafen_commander = HeiligenhafenCommander()

    rclpy.spin(heiligenhafen_commander)

    heiligenhafen_commander.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
