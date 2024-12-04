import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time
from new_interfaces.srv import MoveCar
from pyrobosim_msgs.action import ExecuteTaskAction, ExecuteTaskPlan
from pyrobosim_msgs.msg import TaskAction, TaskPlan
from pyrobosim_msgs.srv import RequestWorldState


class TaxiBotServer(Node):
    def __init__(self):
        super().__init__("TaixBotServer")
        self.move_car_srv = self.create_service(MoveCar, 'move_car', self.move_car_callback)
        # Action client for a single action
        self.action_client = ActionClient(self, ExecuteTaskAction, "execute_action")
        # Action client for a task plan
        self.plan_client = ActionClient(self, ExecuteTaskPlan, "execute_task_plan")
        # Call world state service to ensure node is running
        self.world_state_client = self.create_client(
            RequestWorldState, "request_world_state"
        )
        while rclpy.ok() and not self.world_state_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("Waiting for world state server...")
        future = self.world_state_client.call_async(RequestWorldState.Request())
        rclpy.spin_until_future_complete(self, future)

    def move_car_callback(self, request, responce):
        car_id = request.car_id
        target_location = request.target_loc
        self.get_logger().info(f"Recovied request to move car {car_id} to {target_location}")
        self.get_logger().info("Executing task action...")
        goal = ExecuteTaskAction.Goal()
        goal.action = TaskAction(
            robot=f"TaxiBot{car_id}",
            type="navigate",
            target_location=target_location,
        )
        self.send_action_goal(goal, cancel=False)
        
        return responce
    def send_action_goal(self, goal, cancel=False):
        self.action_client.wait_for_server()
        goal_future = self.action_client.send_goal_async(goal)
        if cancel:
            goal_future.add_done_callback(self.goal_response_callback)

    def send_plan_goal(self, goal, cancel=False):
        self.plan_client.wait_for_server()
        goal_future = self.plan_client.send_goal_async(goal)
        if cancel:
            goal_future.add_done_callback(
                lambda goal_future: self.goal_response_callback(
                    goal_future, cancel_delay=12.5
                )
            )

    def goal_response_callback(self, goal_future, cancel_delay=2.0):
        """Starts a timer to cancel the goal handle, upon receiving an accepted goal."""
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal was rejected.")
            return

        self.cancel_timer = self.create_timer(
            cancel_delay, lambda: self.cancel_goal(goal_handle)
        )

    def cancel_goal(self, goal):
        """Timer callback function that cancels a goal."""
        self.get_logger().info("Canceling goal")
        goal.cancel_goal_async()
        self.cancel_timer.cancel()


def main():
    rclpy.init()
    Node = TaxiBotServer()
    rclpy.spin(Node)
    Node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()