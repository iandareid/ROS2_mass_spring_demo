import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from mass_spring_msgs.srv import Setpoint
from mass_spring_msgs.msg import State

# This controller subscribes to the state of the system, and
# then calculates a control effort and publishes it.

class Controller(Node):
    def __init__(self) -> None:
        super().__init__("controller")

        # Controller gains
        self.declare_parameter('kp', -10.0)
        self.declare_parameter('kd', -4.0)
        self.declare_parameter('ki', -0.03)
        
        # Frequency of the control in Hz.
        self.control_frequency = 100
        self.Ts = 1/self.control_frequency

        self.control_publisher = self.create_publisher(Float32, "control", 10)
        self.state_sub = self.create_subscription(State, "state", self.state_callback, 10)

        self.control_timer = self.create_timer(self.Ts, self.control)

        self.setpoint_service = self.create_service(Setpoint, 'set_setpoint', self.setpoint_callback)

        self.error = 0.0
        self.prev_error = 0.0
        self.integrator = 0.0

        self.commanded_position = 2.2

        self.position = 0.0
        self.velocity = 0.0

    def state_callback(self, msg):
        self.position = msg.position
        self.velocity = msg.velocity

    def control(self):

        kp = self.get_parameter('kp').get_parameter_value().double_value
        kd = self.get_parameter('kd').get_parameter_value().double_value
        ki = self.get_parameter('ki').get_parameter_value().double_value
        
        # Compute control effort using PID
        self.error = self.position - self.commanded_position
        self.integrator += self.Ts/2.0 * self.error + self.prev_error

        u = kp*self.error + kd*self.velocity + ki*self.integrator

        msg = Float32()

        msg.data = u

        self.control_publisher.publish(msg)

    def setpoint_callback(self, request, response):
        self.get_logger().info('Changing setpoint.')

        self.commanded_position = request.setpoint
        response.success = True
        response.message = f"The setpoint was set to {self.commanded_position}"
        return response

def main():
    rclpy.init()

    controller = Controller()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()    

