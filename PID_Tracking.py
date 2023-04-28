import rospy
from geometry_msgs.msg import Twist
from yolov5_pytorch_ros.msg import BoundingBox

# this python file is running on the yahboom ros enviroment
# this code structure is referenced from Yahboom x3 car example
# this code still need to improve, bc the tracking part is still not functioning properly need to debug in the futuer devel

class SimplePIDTracker:
    def __init__(self):
        # Initialize the bounding box subscriber
        self.bb_subscriber = rospy.Subscriber("/bounding_box", BoundingBox, self.bb_callback)
        # Initialize the PID parameters
        self.Kp = 0.5
        self.Ki = 0.0
        self.Kd = 2.0
        # Initialize the target position
        self.target_position = None
        # Initialize the previous error
        self.previous_error = 0.0
        # Initialize the integral error
        self.integral_error = 0.0
        # Initialize the maximum and minimum velocities
        self.max_linear_velocity = 0.5
        self.min_linear_velocity = 0.0
        self.max_angular_velocity = 1.0
        self.min_angular_velocity = -1.0
        # Initialize the publisher to control the robot's movement
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def bb_callback(self, msg):
        # Compute the error between the center of the image and the center of the bounding box
        error = msg.xmin + (msg.xmax - msg.xmin) / 2 - 320
        # Update the integral error
        self.integral_error += error
        # Compute the derivative error
        derivative_error = error - self.previous_error
        # Update the previous error
        self.previous_error = error
        # Compute the control signal using the PID formula
        control_signal = (self.Kp * error) + (self.Ki * self.integral_error) + (self.Kd * derivative_error)
        # Clamp the control signal to the maximum and minimum velocities
        control_signal = max(min(control_signal, self.max_angular_velocity), self.min_angular_velocity)
        # Create the Twist message to control the robot's movement
        twist_msg = Twist()
        twist_msg.linear.x = max(min(0.1 * msg.get_area(), self.max_linear_velocity), self.min_linear_velocity)
        twist_msg.angular.z = control_signal
        # Publish the Twist message
        self.velocity_publisher.publish(twist_msg)

    def run(self):
        # Run the tracker loop
        while not rospy.is_shutdown():
            # Sleep to control the loop rate
            rospy.sleep(0.1)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('simple_pid_tracker', anonymous=True)
    # Create the PID tracker object
    tracker = SimplePIDTracker()
    # Run the tracker loop
    tracker.run()