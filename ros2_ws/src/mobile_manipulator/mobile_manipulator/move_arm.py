import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        # This topic matches the plugin in your URDF
        self.publisher_ = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.move_arm_sequence)
        self.step = 0

    def move_arm_sequence(self):
        if self.step > 0:
            return # Only run this command once

        msg = JointTrajectory()
        # "base_link" is the frame of reference
        msg.header.frame_id = 'base_link'
        # These names MUST match the names in your URDF file exactly
        msg.joint_names = ['joint1', 'joint2', 'joint3']

        point = JointTrajectoryPoint()
        
        # --- COMMAND THE JOINTS ---
        # joint1 (0.0)  = Face forward
        # joint2 (1.57) = 90 degrees (Horizontal/Insert)
        # joint3 (0.0)  = Straight wrist
        point.positions = [0.0, 1.57, 0.0]
        
        # Take 2 seconds to complete the move (so it doesn't snap instantly)
        point.time_from_start = Duration(sec=2, nanosec=0)

        msg.points.append(point)
        self.publisher_.publish(msg)
        
        self.get_logger().info('Moving Arm to Insertion Pose...')
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()