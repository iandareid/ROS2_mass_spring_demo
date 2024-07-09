import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


# This controller subscribes to the state of the system, and
# then calculates a control effort and publishes it.

class Controller(Node):
    def __init__(self) -> None:
        super().__init__("controller")

        # Controller gains
        self.kp = 1.0 
        self.kd = 1.0
        self.ki = 1.0
        
        # Frequency of the control in Hz.
        self.control_frequency = 50
        self.Ts = 1/self.control_frequency

        self.control_publisher = self.create_publisher(Float32, "control", 10)

        self.control_timer = self.create_timer(self.Ts, self.control)

        self.error = 0.0
        self.prev_error = 0.0
        self.integrator = 0.0

        self.commanded_position = 2.2

        self.position = 0.0

    def control(self):
        
        # Compute control effort using PID
        self.error = self.position - self.commanded_position
        error_dot = (self.error - self.prev_error)/(self.Ts)
        self.integrator += self.Ts/2.0 * self.error + self.prev_error

        u = self.kp*self.error + self.kd*error_dot + self.ki*self.integrator

        msg = Float32()

        msg.data = u

        self.control_publisher.publish(msg)

def main():
    rclpy.init()

    controller = Controller()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()    

