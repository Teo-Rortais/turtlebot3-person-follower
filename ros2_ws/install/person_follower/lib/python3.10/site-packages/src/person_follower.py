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
        self.display = videoOutput("my_video.mp4")      #display://0
        self.get_logger().info("AI initialized.")

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
                        self.get_logger().info(f"Tracking object with ID {self.tracker}: Center ({center[0]}) / {img.shape[1]}")
                        # twist.angular.z = (center[0] - img_center_x)*0.2
                        twist.angular.z = -(center[0] - img_center_x)*0.008
                        self.get_logger().info(f"MOUVEMENT : {img_center_x} --- {twist.angular.z}")
                        self.get_logger().info("MVT : {center[0]} - {img.shape[1]}")
                        #if center[0] < img.shape[1]*2/5:
                        #    self.get_logger().info("LEFT")
                        #    twist.angular.z = 0.5
                        #elif center[0] > img.shape[1]*3/5:
                        #    self.get_logger().info("RIGHT")
                        #    twist.angular.z = -0.5
                        #else:
                        #    self.get_logger().info("CENTER")
                        #    twist.angular.z = 0.0
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

