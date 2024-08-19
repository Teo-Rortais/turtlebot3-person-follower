#
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

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
