import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float64, String
from geometry_msgs.msg import Twist
from enum import Enum, auto
from delivery_robot.nav2_client import Nav2Client


class DeliveryState(Enum):
    IDLE       = auto()
    NAVIGATING = auto()
    ALIGNING   = auto()
    ARRIVED    = auto()
    DELIVERING = auto()
    RETURNING  = auto()
    ERROR      = auto()


TX_ALIGNMENT_THRESHOLD_DEG = 2.0
TA_ARRIVAL_THRESHOLD = 25.0
FORWARD_SPEED  =  0.2
TURN_SPEED_MAX =  0.4
TURN_GAIN      =  0.03

# Map of room tag IDs to (x, y, yaw) positions on the facility map
# TODO: Replace with real coordinates after SLAM mapping run
ROOM_POSITIONS = {
    1:  (2.0,  1.0, 0.0),
    2:  (2.0,  4.0, 0.0),
    3:  (2.0,  7.0, 0.0),
    4:  (2.0, 10.0, 0.0),
    5:  (2.0, 13.0, 0.0),
}

# Home base position (where robot returns after delivery)
HOME_POSITION = (0.0, 0.0, 0.0)


class DeliveryManagerNode(Node):

    def __init__(self):
        super().__init__('delivery_manager')

        self._state = DeliveryState.IDLE
        self._target_tag_id = -1
        self._tag_visible = False
        self._tx = 0.0
        self._ta = 0.0
        self._current_tag_id = -1
        self._delivery_elapsed = 0.0

        # Nav2 client for sending navigation goals
        self._nav2 = Nav2Client(self)

        self.create_subscription(Bool,    '/limelight/tag_visible', self._cb_tag_visible, 10)
        self.create_subscription(Int32,   '/limelight/tag_id',      self._cb_tag_id,      10)
        self.create_subscription(Float64, '/limelight/tx',          self._cb_tx,          10)
        self.create_subscription(Float64, '/limelight/ta',          self._cb_ta,          10)
        self.create_subscription(Int32,   '/delivery/request',      self._cb_request,     10)

        self.pub_cmd_vel = self.create_publisher(Twist,  '/cmd_vel',         10)
        self.pub_status  = self.create_publisher(String, '/delivery/status', 10)

        self.create_timer(0.1, self._control_loop)

        self.get_logger().info('Delivery Manager ready. Waiting for delivery requests...')
        self.get_logger().info('Send a request: ros2 topic pub /delivery/request std_msgs/msg/Int32 "data: 3"')

    def _cb_tag_visible(self, msg): self._tag_visible = msg.data
    def _cb_tag_id(self, msg):      self._current_tag_id = msg.data
    def _cb_tx(self, msg):          self._tx = msg.data
    def _cb_ta(self, msg):          self._ta = msg.data

    def _cb_request(self, msg):
        if self._state != DeliveryState.IDLE:
            self.get_logger().warn(
                f'Delivery request for room {msg.data} ignored — '
                f'robot is currently {self._state.name}'
            )
            return
        self._target_tag_id = msg.data
        self._transition_to(DeliveryState.NAVIGATING)
        self.get_logger().info(f'Delivery requested to room tag #{msg.data}')

    def _control_loop(self):
        if   self._state == DeliveryState.IDLE:        self._handle_idle()
        elif self._state == DeliveryState.NAVIGATING:  self._handle_navigating()
        elif self._state == DeliveryState.ALIGNING:    self._handle_aligning()
        elif self._state == DeliveryState.ARRIVED:     self._handle_arrived()
        elif self._state == DeliveryState.DELIVERING:  self._handle_delivering()
        elif self._state == DeliveryState.RETURNING:   self._handle_returning()
        elif self._state == DeliveryState.ERROR:       self._handle_error()

        status_msg = String()
        status_msg.data = (
            f'State: {self._state.name} | '
            f'Target room: {self._target_tag_id} | '
            f'Tag visible: {self._tag_visible} | '
            f'tx: {self._tx:.1f}° | '
            f'Nav2 active: {self._nav2.is_navigating()}'
        )
        self.pub_status.publish(status_msg)

    def _handle_idle(self):
        self._stop()

    def _handle_navigating(self):
        # If Limelight spots the target tag, switch to fine alignment
        if self._tag_visible and self._current_tag_id == self._target_tag_id:
            self.get_logger().info(
                f'Target tag {self._target_tag_id} spotted! Cancelling Nav2, switching to ALIGNING.'
            )
            self._nav2.cancel_goal()
            self._transition_to(DeliveryState.ALIGNING)
            return

        # Send Nav2 goal if not already navigating
        if not self._nav2.is_navigating():
            if self._target_tag_id not in ROOM_POSITIONS:
                self.get_logger().error(f'No position known for tag ID {self._target_tag_id}!')
                self._transition_to(DeliveryState.ERROR)
                return

            x, y, yaw = ROOM_POSITIONS[self._target_tag_id]
            success = self._nav2.send_goal(
                x, y, yaw,
                on_complete=self._on_nav2_complete
            )
            if not success:
                self.get_logger().warn('Nav2 not available yet, retrying...')

    def _on_nav2_complete(self, success: bool):
        """Called by Nav2Client when navigation finishes."""
        if success:
            self.get_logger().info('Nav2 reached goal area. Waiting for Limelight tag...')
        else:
            self.get_logger().warn('Nav2 failed to reach goal. Switching to ERROR.')
            self._transition_to(DeliveryState.ERROR)

    def _handle_aligning(self):
        if not self._tag_visible or self._current_tag_id != self._target_tag_id:
            self.get_logger().warn('Lost sight of target tag. Returning to NAVIGATING.')
            self._transition_to(DeliveryState.NAVIGATING)
            return

        turn = -self._tx * TURN_GAIN
        turn = max(-TURN_SPEED_MAX, min(TURN_SPEED_MAX, turn))

        centered = abs(self._tx) < TX_ALIGNMENT_THRESHOLD_DEG
        close_enough = self._ta > TA_ARRIVAL_THRESHOLD

        if centered and close_enough:
            self._stop()
            self._transition_to(DeliveryState.ARRIVED)
            return

        forward = FORWARD_SPEED if not close_enough else 0.0
        self._drive(forward=forward, turn=turn)

    def _handle_arrived(self):
        self._stop()
        self.get_logger().info(f'Arrived at room {self._target_tag_id}! Starting delivery...')
        self._delivery_elapsed = 0.0
        self._transition_to(DeliveryState.DELIVERING)

    def _handle_delivering(self):
        self._stop()
        self._delivery_elapsed += 0.1
        if self._delivery_elapsed >= 3.0:
            self._delivery_elapsed = 0.0
            self.get_logger().info('Delivery complete! Returning to base.')
            self._transition_to(DeliveryState.RETURNING)

    def _handle_returning(self):
        if not self._nav2.is_navigating():
            x, y, yaw = HOME_POSITION
            success = self._nav2.send_goal(
                x, y, yaw,
                on_complete=self._on_return_complete
            )
            if not success:
                self.get_logger().warn('Nav2 not available, going straight to IDLE.')
                self._transition_to(DeliveryState.IDLE)

    def _on_return_complete(self, success: bool):
        """Called when robot finishes returning to base."""
        if success:
            self.get_logger().info('Returned to base successfully!')
        else:
            self.get_logger().warn('Failed to return to base cleanly.')
        self._transition_to(DeliveryState.IDLE)

    def _handle_error(self):
        self._stop()
        self._nav2.cancel_goal()
        self.get_logger().error('Robot is in ERROR state. Manual reset required.')

    def _transition_to(self, new_state: DeliveryState):
        self.get_logger().info(f'State: {self._state.name} → {new_state.name}')
        self._state = new_state

    def _drive(self, forward: float, turn: float):
        msg = Twist()
        msg.linear.x  = forward
        msg.angular.z = turn
        self.pub_cmd_vel.publish(msg)

    def _stop(self):
        self._drive(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
