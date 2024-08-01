import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time  # Importing the time module for sleep functionality

class YellowBallDetector(Node):
    def __init__(self):
        super().__init__('yellow_ball_detector')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.yellow_threshold = 35.0
        self.saved_pose = None
        self.position_logged = False
        self.goal_sent = False
        self.origin_pose = None  # To store the origin position

        # Initialize the origin position
        self.get_initial_pose()

        self.get_logger().info("YellowBallDetector node has been started.")

    def get_initial_pose(self):
        try:
            self.get_logger().info("Waiting for the 'map' frame to become available...")
            while not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                self.get_logger().warn("Cannot transform 'map' to 'base_link' yet. Waiting...")
                rclpy.spin_once(self, timeout_sec=1.0)
            
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_link', now)
            self.origin_pose = PoseStamped()
            self.origin_pose.header.stamp = transform.header.stamp
            self.origin_pose.header.frame_id = 'map'
            self.origin_pose.pose.position.x = transform.transform.translation.x
            self.origin_pose.pose.position.y = transform.transform.translation.y
            self.origin_pose.pose.position.z = transform.transform.translation.z
            self.origin_pose.pose.orientation = transform.transform.rotation
            self.get_logger().info(f"Origin position saved: {self.origin_pose.pose.position}")
        except Exception as e:
            self.get_logger().error(f"Failed to get initial transform: {e}")

    def image_callback(self, msg):
        if self.position_logged:
            return  # Stop processing further images

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([15, 80, 80])
            upper_yellow = np.array([35, 255, 255])
            mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
            yellow_pixels = np.sum(mask > 0)
            total_pixels = mask.size
            yellow_percentage = (yellow_pixels / total_pixels) * 100
            
            self.get_logger().info(f"Yellow percentage: {yellow_percentage:.2f}%")
            
            if yellow_percentage > self.yellow_threshold and not self.position_logged:
                self.get_logger().info("Yellow threshold reached, saving robot position...")
                try:
                    now = rclpy.time.Time()
                    transform = self.tf_buffer.lookup_transform('map', 'base_link', now)
                    
                    self.saved_pose = PoseStamped()
                    self.saved_pose.header.stamp = transform.header.stamp
                    self.saved_pose.header.frame_id = 'map'
                    self.saved_pose.pose.position.x = transform.transform.translation.x
                    self.saved_pose.pose.position.y = transform.transform.translation.y
                    self.saved_pose.pose.position.z = transform.transform.translation.z
                    self.saved_pose.pose.orientation = transform.transform.rotation
                    
                    self.get_logger().info(f"Saved robot position: {self.saved_pose.pose.position}")
                    self.position_logged = True

                    # Close the OpenCV window
                    cv2.destroyAllWindows()

                    # Prompt user to send coordinates to Nav2
                    self.prompt_user_to_send_goal()
                except Exception as e:
                    self.get_logger().error(f"Failed to get transform: {e}")
            
            # Display the image window only if threshold has not been met
            if not self.position_logged:
                cv2.imshow("Yellow Mask", mask)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def prompt_user_to_send_goal(self):
        print("Ready to send the coordinates to Nav2? (y/n)")
        user_input = input().strip().lower()
        if user_input == 'y':
            self.send_goal(self.saved_pose, self.goal_reached_callback)

    def send_goal(self, pose, callback):
        if pose and not self.goal_sent:
            self.get_logger().info("Sending goal to Nav2...")
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            self.action_client.wait_for_server()
            future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            future.add_done_callback(callback)
            self.goal_sent = True

    def goal_reached_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by Nav2.')
            return

        self.get_logger().info('Goal accepted by Nav2. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.return_to_origin)

    def return_to_origin(self, future):
        result = future.result()
        if result.status == 3:  # SUCCEEDED
            self.get_logger().info('Goal reached, waiting before returning to origin...')
            time.sleep(5)  # Wait for 5 seconds before returning
            self.get_logger().info('Returning to origin...')
            self.send_goal(self.origin_pose, self.origin_reached_callback)

    def origin_reached_callback(self, future):
        self.get_logger().info('Robot has returned to the origin.')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: current position x={feedback.current_pose.pose.position.x} y={feedback.current_pose.pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    node = YellowBallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

