import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from geometry_msgs.msg import Twist
from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy, videoOutput

class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.get_logger().info('Initializing PersonFollower node...')
        
        # Initialize the subscriber to get the images from the TurtleBot
        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.get_logger().info("Subscriber to /image_raw/compressed topic created.")
        
        # Initialize the publisher to send the commands to the TurtleBot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Publisher to cmd_vel topic created.")
        
        # Initialize the AI from Jetson Inference
        self.net = detectNet("ssd-mobilenet-v2", threshold=0.6)
        self.net.SetTrackingEnabled(True)
        self.tracker = None
        self.display = videoOutput("display://0")      # If you need a video file, juste replace with my_video.mp4
        self.get_logger().info("AI initialized.")
                    
        # Set up the speed parameters
        self.angular_param = 0.005 		# Needs to be between 0 and 0.00875
        self.linear_param = 0.004		# Needs to be between 0 and 0.004

    def image_callback(self, msg):
        self.get_logger().info("Received a new image message.")
        
        twist = Twist()
        
        try:
            # Convert ROS CompressedImage message to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img_center_x = img.shape[1] // 2

            # Verify if the image is correctly decoded
            if img is None:
                self.get_logger().error("Failed to decode image.")
                return

            # Convert OpenCV image (numpy array) to CUDA image
            cuda_img = cudaFromNumpy(img)

            # Use DetectNet on the CUDA image
            detections = self.net.Detect(cuda_img)
            
            # Manage tracker
            if self.tracker is not None:
                # Check if the tracked object is still detected
                tracked = False
                for detection in detections:
                    if detection.TrackID == self.tracker:
                        # Get center coordinates of the tracked object
                        center = detection.Center
                        height = detection.Height
                        self.get_logger().info(f"Tracking object with ID {self.tracker}") 
                               
                        # Adjust the values to send to the robot
                        twist.angular.z = (img_center_x - center[0]) * self.angular_param  
                        if 440 - height < 0:
                        	twist.linear.x = (320 - height) * self.linear_param
                        else : 
                        	twist.linear.x = (320 - height) * self.linear_param * 1.7
                        
                        twist.linear.x = (440 - height) * linear_param
      
                        tracked = True
                        break
                if not tracked:
                    # If the tracked object is not detected, reset the tracker
                    self.tracker = None
                    self.get_logger().info("Tracked object disappeared, resetting tracker.")
            else:
                # Look for the object to track
                for detection in detections:
                    if self.net.GetClassDesc(detection.ClassID) == "person":
                        self.tracker = detection.TrackID
                        self.get_logger().info(f"Started tracking object with ID {self.tracker}")
                        break

	    # Publish the command
            self.cmd_vel_pub.publish(twist)

            # Render the image
            self.display.Render(cuda_img)
            self.display.SetStatus("Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

