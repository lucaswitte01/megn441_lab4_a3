import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from action_msgs.msg import GoalStatus

from demo_msgs.action import Waypoint

# Corners of the box in turtlesim coordinates (the window is roughly 0-11).
CORNERS = [(3.0, 3.0), (8.0, 3.0), (8.0, 8.0), (3.0, 8.0)]
CLOSE_BOX = True  # add a final leg back to the first corner

# Big picture:
# 1. Run start(), then for each corner (send_next_goal),
# 2. send_goal_async, wait for accepted/rejected (send_goal_callback)
# 3. get_results_async, wait for response (result_callback)
# 4. result_callback calls send_next_goal again

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.client = ActionClient(self, Waypoint, 'waypoint')

        self.waypoints = list(CORNERS)
        if CLOSE_BOX:
            self.waypoints.append(CORNERS[0])
        self.index = 0

        # Completes with True (box finished) or False (a goal failed)
        self.done_future = Future()

    def start(self):
        # Wait for the server, then send the first waypoint.
        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server() # Blocks code until action server is ready
        self.send_next_goal()

    def send_next_goal(self):
        # Send next waypoint

        # If you're at the end of the list, return done!
        if self.index >= len(self.waypoints):
            self.get_logger().info('Box complete!')
            self.done_future.set_result(True)
            return

        # Set the goal to the next waypoint from the list
        x, y = self.waypoints[self.index]
        goal_msg = Waypoint.Goal()
        goal_msg.target.x = x
        goal_msg.target.y = y

        self.get_logger().info(
            f'Sending waypoint {self.index + 1}/{len(self.waypoints)}: ({x}, {y})')

        # Send the goal asyncronously
        # Feedback callback does something with the feedback (here it just prints)
        send_future = self.client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        # What do we do once the goal is accepted or rejected? goal_response_callback        
        send_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # When we get feedback, print it out
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'  distance: {fb.distance:.2f}  angle: {fb.angle:.2f}',
            throttle_duration_sec=0.5)

    def goal_response_callback(self, future):
        goal_handle = future.result() # Accepted or rejected?
        # If rejected, the whole thing fails
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server, stopping')
            self.done_future.set_result(False)
            return

        # Set up future to wait for the results
        result_future = goal_handle.get_result_async()
        # What do we do when we get the results? result_callback 
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        # Callback to run once action server sends results

        result = future.result() # Succeed or fail?
        # If fail, print out reason and return failure overall
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Waypoint {self.index + 1} did not succeed '
                f'(status {result.status}), stopping')
            self.done_future.set_result(False)
            return

        # If we've succeeded, print to logger, move to next index and send_next goal
        pos = result.result.position
        self.get_logger().info(f'Reached ({pos.x:.2f}, {pos.y:.2f})')
        self.index += 1
        self.send_next_goal()


def main():
    rclpy.init()
    node = ClientNode()
    try:
        # Run start function, then spin node until overall success/failure
        node.start()
        rclpy.spin_until_future_complete(node, node.done_future)
        if node.done_future.done() and not node.done_future.result():
            node.get_logger().error('Box was not completed')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()