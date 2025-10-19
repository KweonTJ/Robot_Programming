import rclpy
from rclpy.node import Node

# Custom interface
from cafe_interfaces.msg import RobotStatus
from rclpy.callback_groups import ReentrantCallbackGroup

class CafeManager(Node):
    def __init__(self) -> None:
        super().__init__('cafe_manager')
        self.grp = ReentrantCallbackGroup()
        self.last: RobotStatus | None = None
        self.update_cnt = 0

        # 구독자
        # self.create_subscription(
        #     RobotStatus,
        #     '/robot/status',
        #     self.on_status,
        #     10
        # )
        self.sub_status = self.create_subscription(
            RobotStatus, 
            '/robot/status', 
            self.on_status, 10,
            callback_group=self.grp)

        # 5 초 타이머
        # self.create_timer(5.0, self.report)
        self.timer_report = self.create_timer(5.0, self.report, callback_group=self.grp)

    # ───────── 콜백 ─────────
    def on_status(self, msg: RobotStatus) -> None:
        self.last = msg
        self.update_cnt += 1

        if msg.is_busy:
            self.get_logger().info(
                f'[BUSY] {msg.robot_id} "{msg.current_task}" @ {msg.current_location}')

        if msg.battery_level < 20.0:
            self.get_logger().warn(
                f'[LOW BATTERY] {msg.robot_id} {msg.battery_level:.1f}%')

    def report(self) -> None:
        if not self.last:
            self.get_logger().info('No status yet.')
            return

        state = 'BUSY' if self.last.is_busy else 'IDLE'
        self.get_logger().info(
            f'[REPORT] robot={self.last.robot_id}  state={state}  '
            f'task="{self.last.current_task}"  loc="{self.last.current_location}"  '
            f'batt={self.last.battery_level:.1f}%  updates={self.update_cnt}'
        )


# def main() -> None:
#     rclpy.init()
#     rclpy.spin(CafeManager())
#     rclpy.shutdown()

def main() -> None:
    rclpy.init()
    node = CafeManager()

    exec = rclpy.executors.MultiThreadedExecutor()
    exec.add_node(node)

    try:
        exec.spin()                # 블로킹
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()