import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

# Custom interface
from cafe_interfaces.msg import MenuItem, RobotStatus
from cafe_interfaces.srv import GetMenu, CheckPrice, CancelOrder
from cafe_interfaces.action import MakeDrink, DeliverOrder

class CafeRobot(Node):
    def __init__(self):
        super().__init__('cafe_robot')

        # ── parameters ─────────────────────────────
        self.declare_parameter('robot_id', 'robot_001')
        self.declare_parameter('battery_level', 100.0)       # initial only
        self.declare_parameter('drink_making_time', 5.0)
        self.declare_parameter('delivery_time', 3.0)

        self.robot_id = self.get_parameter('robot_id').value
        self.battery  = self.get_parameter('battery_level').value
        self.t_make   = self.get_parameter('drink_making_time').value
        self.t_deliv  = self.get_parameter('delivery_time').value

        # ── state ──────────────────────────────────
        self.is_busy          = False
        self.current_task     = ''
        self.current_location = 'kitchen'
        self.active_orders: dict[int, str] = {}

        # static menu
        self.menu = {
            'Americano':  MenuItem(drink_name='Americano',  description='Coffee', price=3.0, available=True),
            'Latte':      MenuItem(drink_name='Latte',      description='Coffee', price=3.5, available=True),
            'Cappuccino': MenuItem(drink_name='Cappuccino', description='Coffee', price=3.2, available=True),
            'Espresso':   MenuItem(drink_name='Espresso',   description='Coffee', price=2.5, available=False),
        }

        # ── comms (Reentrant) ──────────────────────
        # group = ReentrantCallbackGroup()
        self.grp = ReentrantCallbackGroup()

        # topic publisher (1 Hz)
        self.pub_status = self.create_publisher(RobotStatus, '/robot/status', 10)
        # self.create_timer(1.0, self.publish_status)
        self.timer_status = self.create_timer(1.0, self.publish_status, callback_group=self.grp)


        # services
        self.create_service(GetMenu,     'get_menu',     self.srv_get_menu,     callback_group=self.grp)
        self.create_service(CheckPrice,  'check_price',  self.srv_check_price,  callback_group=self.grp)
        self.create_service(CancelOrder, 'cancel_order', self.srv_cancel_order, callback_group=self.grp)

        # actions
        # ActionServer(self, MakeDrink,    'make_drink',    self.act_make_drink, callback_group=group)
        # ActionServer(self, DeliverOrder, 'deliver_order', self.act_deliver,    callback_group=group)
        self.action_make = ActionServer(self, MakeDrink, 'make_drink', self.act_make_drink, callback_group=self.grp)
        self.action_deliver = ActionServer(self, DeliverOrder, 'deliver_order', self.act_deliver, callback_group=self.grp)


    # ───────────────── service handlers ───────────
    def srv_get_menu(self, req, resp):
        resp.menu_items = [m for m in self.menu.values()
                           if (not req.available_only) or m.available]
        resp.success, resp.message = True, 'menu ready'
        return resp

    def srv_check_price(self, req, resp):
        item = self.menu.get(req.drink_name)
        if item:
            resp.price, resp.available, resp.message = item.price, item.available, 'found'
        else:
            resp.price, resp.available, resp.message = 0.0, False, 'not found'
        resp.success = True
        return resp

    def srv_cancel_order(self, req, resp):
        removed = self.active_orders.pop(req.order_id, None)
        resp.success = bool(removed)
        resp.message = 'cancelled' if removed else 'no such order'
        if removed:
            self.get_logger().info(f'Order {req.order_id} cancelled')
        return resp

    # ───────────────── action handlers ────────────
    def act_make_drink(self, goal_handle):
        g = goal_handle.request
        item = self.menu.get(g.drink_name)
        if (item is None) or (not item.available):
            goal_handle.abort()
            return MakeDrink.Result(success=False, message='unavailable', completion_time=0)

        # accept
        self.is_busy = True
        self.current_task = f'make {g.drink_name}'
        self.active_orders[g.order_id] = g.drink_name
        self.get_logger().info(f'Start make_drink order={g.order_id} {g.drink_name}')

        steps = ['Grinding beans', 'Brewing', 'Adding ingredients', 'Finishing']
        step_time = self.t_make / len(steps)

        for i, step in enumerate(steps, 1):
            if goal_handle.is_cancel_requested:
                self.reset_state()
                goal_handle.canceled()
                self.get_logger().info(f'Order {g.order_id} cancelled during make')
                return MakeDrink.Result(success=False, message='cancelled', completion_time=0)

            fb = MakeDrink.Feedback(progress_percent=int(i/len(steps)*100), current_step=step)
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f'[MAKE] {step} ({i}/{len(steps)})')
            time.sleep(step_time)

        self.battery = max(0.0, self.battery - 5.0)
        self.reset_state()
        goal_handle.succeed()
        self.get_logger().info(f'Finish make_drink order={g.order_id}')
        return MakeDrink.Result(success=True, message='done', completion_time=int(time.time()))

    def act_deliver(self, goal_handle):
        g = goal_handle.request
        if g.order_id not in self.active_orders:
            goal_handle.abort()
            return DeliverOrder.Result(success=False, message='no order', delivery_time=0.0)

        self.is_busy = True
        self.current_task = f'deliver {g.order_id}'
        self.get_logger().info(f'Start deliver order={g.order_id}')
        path = ['kitchen', 'hallway', 'dining area', f'table {g.table_number}']
        hop_time = self.t_deliv / len(path)

        for i, loc in enumerate(path, 1):
            fb = DeliverOrder.Feedback(
                current_location=loc,
                distance_remaining=float(len(path) - i),
                progress_percent=int(i/len(path)*100)
            )
            goal_handle.publish_feedback(fb)
            self.current_location = loc
            self.get_logger().info(f'[DELIVER] at {loc} ({i}/{len(path)})')
            time.sleep(hop_time)

        self.battery = max(0.0, self.battery - 3.0)
        self.active_orders.pop(g.order_id, None)
        self.reset_state()
        goal_handle.succeed()
        self.get_logger().info(f'Finish deliver order={g.order_id}')

        # if not self.active_orders:            # 모든 주문 완료
        #     self.get_logger().info('idle → shutdown in 1 s')
        #     self.timer_exit = self.create_timer(
        #         1.0, lambda: rclpy.shutdown(), callback_group=self.grp)

        return DeliverOrder.Result(success=True, message='delivered', delivery_time=self.t_deliv)

    # ───────────────── helper ─────────────────────
    def publish_status(self):
        msg = RobotStatus(
            robot_id=self.robot_id,
            battery_level=self.battery,
            current_location=self.current_location,
            current_task=self.current_task,
            is_busy=self.is_busy,
            timestamp=int(self.get_clock().now().to_msg().sec)
        )
        self.pub_status.publish(msg)

        self.get_logger().info(
            f'[STATUS] {self.robot_id} batt={self.battery:.1f}% '
            f'task="{self.current_task or "idle"}" loc="{self.current_location}"'
        )

    def reset_state(self):
        self.is_busy, self.current_task, self.current_location = False, '', 'kitchen'


def main():
    rclpy.init()
    node = CafeRobot()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == '__main__':
    main()