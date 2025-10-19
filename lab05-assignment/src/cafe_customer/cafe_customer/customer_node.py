import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import random
import time

#Custrom interfaces
import threading
from cafe_interfaces.srv import GetMenu, CheckPrice, CancelOrder
from cafe_interfaces.action import MakeDrink, DeliverOrder
from rclpy.callback_groups import ReentrantCallbackGroup

class CafeCustomer(Node):
    def __init__(self):
        super().__init__('cafe_customer')
        self.grp = ReentrantCallbackGroup()

        # service clients
        self.menu_cli   = self.create_client(GetMenu,   'get_menu')
        self.price_cli  = self.create_client(CheckPrice, 'check_price')
        self.cancel_cli = self.create_client(CancelOrder, 'cancel_order')

        # action clients
        self.make_act = ActionClient(self, MakeDrink,    'make_drink', callback_group=self.grp)
        self.deliv_act = ActionClient(self, DeliverOrder, 'deliver_order', callback_group=self.grp)

        # 시나리오 한 번 실행
        # self.create_timer(0.5, self.start_once)
        self._started = False
        self._running = False
        self.timer_start  = self.create_timer(0.5, self.start_once, callback_group=self.grp)
        # self._started = False

    # ───────── helpers ─────────
    def wait(self, cli_or_act):
        if hasattr(cli_or_act, 'service_is_ready'):
            cli_or_act.wait_for_service()
        else:
            cli_or_act.wait_for_server()

    def call_sync(self, client, req):
        self.wait(client)
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def run_customer_scenario_async(self):
        threading.Thread(target=self.run_customer_scenario, daemon=True).start()


    # ───────── scenario entry ─────────
    def start_once(self):
        if self._started:
            return

        ready_flags = {
            "menu":   self.menu_cli.service_is_ready(),
            "price":  self.price_cli.service_is_ready(),
            "cancel": self.cancel_cli.service_is_ready(),
            "make":   self.make_act.server_is_ready(),
            "deliver":self.deliv_act.server_is_ready(),
        }
        self.get_logger().info(f'[READY] {ready_flags}')   # 준비상태 가시화

        if not all(ready_flags.values()):
            return

        self._started = True
        self.timer_start.cancel()
        self.get_logger().info('Start continuous random-order mode')
        self.run_customer_scenario_async()

        # self.create_timer(5.0, self.run_customer_scenario, callback_group=self.grp)
        self.timer_loop = self.create_timer(5.0, self.run_customer_scenario_async, callback_group=self.grp)

    # ───────── base scenario ─────────
    def run_customer_scenario(self):
        if self._running:                    # 이미 실행 중이면 새 주문 건너뜀
            return
        self._running = True  
        try:
            menu_res = self.call_sync(self.menu_cli, GetMenu.Request(available_only=True))
            choices = [m.drink_name for m in menu_res.menu_items]
            if not choices:
                self.get_logger().warning('no available menu'); return
            drink = random.choice(choices)

            price_res = self.call_sync(self.price_cli, CheckPrice.Request(drink_name=drink))
            self.get_logger().info(f'{drink} 가격 {price_res.price}')

            order_id = random.randint(1000, 9999)
            table = random.randint(1, 5)
            self.make_drink(order_id, drink, table)
            self.deliver(order_id, drink, table)
        except Exception as e:
            self.get_logger().error(f'run_customer_scenario error: {e}')
        finally:
            self._running = False  

    # ───────── advanced scenario ─────────
    def run_advanced_scenario(self):
        """무작위 메뉴·ID·테이블로 주문·제조·배달 1회 수행"""

        # 1) 랜덤 메뉴 선택
        menu_res = self.call_sync(self.menu_cli, GetMenu.Request(available_only=True))
        drink = choice([m.name for m in menu_res.menu_items])

        # 2) 주문 정보
        order_id = randint(1000, 9999)
        table    = randint(1, 5)

        self.get_logger().info(f'새 주문: {drink} (order={order_id}, table={table})')

        # 3) 제조 → 배달
        self.make_drink(order_id, drink, table)
        self.deliver(order_id, drink, table)

        # 4) 다음 주문 예약 (e.g. 5 초 후)
        self.create_timer(5.0, self.run_random_order, callback_group=self.grp)

    # ───────── action helpers ─────────
    def make_drink(self, order_id, drink, table, async_only=False):
        self.wait(self.make_act)
        goal = MakeDrink.Goal(drink_name=drink, order_id=order_id, table_number=table)
        fut = self.make_act.send_goal_async(goal, feedback_callback=self.fb_make)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('make_drink rejected')
            return None
        if async_only:
            return gh
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result().result
        self.get_logger().info(f'make_drink result: {res.message}')
        return gh

    def deliver(self, order_id, drink, table):
        self.wait(self.deliv_act)
        goal = DeliverOrder.Goal(order_id=order_id, table_number=table, drink_name=drink)
        fut = self.deliv_act.send_goal_async(goal, feedback_callback=self.fb_deliv)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('deliver rejected')
            return
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result().result
        self.get_logger().info(f'delivery result: {res.message}')

    # ───────── feedback ─────────
    def fb_make(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'[제조] {fb.progress_percent}% - {fb.current_step}')

    def fb_deliv(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'[배달] {fb.progress_percent}% - {fb.current_location}')


def main():
    rclpy.init()
    node = CafeCustomer()
    exec = rclpy.executors.MultiThreadedExecutor()
    exec.add_node(node)
    try:
        exec.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # rclpy.spin(node)  # 종료는 node 내부에서 rclpy.shutdown 호출


if __name__ == '__main__':
    main()