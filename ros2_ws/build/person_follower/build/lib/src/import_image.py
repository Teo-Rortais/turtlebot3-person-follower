import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import os

class ImportImage(Node):

    def __init__(self):
        super().__init__('import_image')
        
        # Initilalization of the subscriber to get the images from the TurtleBot
        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Subscription to /image_raw/compressed topic created.")

        # Create a directory to save images
        self.save_dir = 'src/person_follower/saved_images'
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"Images will be saved to: {self.save_dir}")


    def image_callback(self, msg):
        self.get_logger().info("Received a new image message.")
        try:
            # Convert ROS CompressedImage message to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Verify if the image is correctly decoded
            if image is None:
                self.get_logger().error("Failed to decode image.")
                return

            # Save the image to file
            image_filename = f'image_debug.jpg'
            image_path = os.path.join(self.save_dir, image_filename)
            cv2.imwrite(image_path, image)
            self.get_logger().info(f"Image saved: {image_path}")

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    import_image = ImportImage()
    rclpy.spin(import_image)
    import_image.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
