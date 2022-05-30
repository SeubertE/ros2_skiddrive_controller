import queue
import rclpy
from simple_pid import PID

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from freenove_interfaces.msg import Motor as MotorMsg
import message_filters

class drivebase_pidctrl(Node):
    def __init__(self):
        super().__init__('drivebase_pidctrl')

        # Subscribers (need to be time synced)
        self.cmd_vel_sub = message_filters.Subscriber(Twist, '/cmd_vel')
        self.odom_sub = message_filters(Odometry, 'odom/filtered')

        self.time_synced_sub = message_filters.ApproximateTimeSynchronizer([self.cmd_vel_sub, self.odom_sub], 10, 0.1, False)
        self.time_synced_sub.registerCallback(self.input_callback)

        # Publisher
        self.motor_pub = self.create_publisher(MotorMsg, '/motor_topic', 10)

        # Paramters
        self.Kp_l_param = self.declare_parameter('Kp_l', 0.0)
        self.Ki_l_param = self.declare_parameter('Ki_l', 0.0)
        self.Kd_l_param = self.declare_parameter('Kd_l', 0.0)

        self.Kp_a_param = self.declare_parameter('Kp_a', 0.0)
        self.Ki_a_param = self.declare_parameter('Ki_a', 0.0)
        self.Kd_a_param = self.declare_parameter('Kd_a', 0.0)

        self.motor_scale_param = self.declare_parameter('motor_scale', 4.92)
        self.wheel_radius_param = self.declare_parameter('wheel_radius', 0.0508)
        self.wheel_base_param = self.declare_parameter('wheel_base', 0.0125)

        # Initalize PID controller
        self.linear_pid = PID(0.0, 0.0, 0.0, 0.0, None)
        self.angular_pid = PID(0.0, 0.0, 0.0, 0.0, None)

    def input_callback(self, cmd_vel_msg = Twist, odom_msg = Odometry):
        #===============================
        # Controller
        #===============================
        # Allow for PID constants to be updated at runtime
        Kp_l = self.get_parameter('Kp_l').float_value
        Ki_l = self.get_parameter('Ki_l').float_value
        Kd_l = self.get_parameter('Kd_l').float_value

        Kp_a = self.get_parameter('Kp_a').float_value
        Ki_a = self.get_parameter('Ki_a').float_value
        Kd_a = self.get_parameter('Kd_a').float_value

        self.linear_pid.tunings = (Kp_l, Ki_l, Kd_l)
        self.angular_pid.tunings = (Kp_a, Ki_a, Kd_a)

        # Get /cmd_vel and odom/filtered to make error signal
        #V_x_error = cmd_vel_msg.linear.x - odom_msg.twist.twist.linear.x
        #W_z_error = cmd_vel_msg.angular.z - odom_msg.twist.twist.angular.z

        self.linear_pid.setpoint = cmd_vel_msg.linear.x
        self.angular_pid.setpoint = cmd_vel_msg.angular.z

        # Get outputs from PID controller
        ctrl_v_x = self.linear_pid(odom_msg.twist.twist.linear.x)
        ctrl_w_z = self.angular_pid(odom_msg.twist.twist.angular.z)

        #===============================
        # Plant
        #===============================
        # Calculate angular velocities
        motor_scale = self.get_parameter('motor_scale').float_value
        wheel_radius = self.get_parameter('wheel_radius').float_value
        wheel_base = self.get_parameter('wheel_base').float_value

        arclength = ctrl_w_z * (wheel_base/2)
        w_l = (ctrl_v_x + arclength) / wheel_radius
        w_r = (ctrl_v_x - arclength) / wheel_radius

        # Scale and send to plant (motors)
        motor_msg = MotorMsg()
        m_w_l = w_l * motor_scale
        m_w_r = w_r * motor_msg

        motor_msg.left_lower_wheel = m_w_l
        motor_msg.left_upper_wheel = m_w_r
        self.motor_pub.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)
    print("Starting PID Controller for Motor")

    drivectrl = drivebase_pidctrl()

    rclpy.spin(drivectrl)

    rclpy.shutdown()