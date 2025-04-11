import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from flask import Flask, Response
from threading import Thread, Lock

# Flask app for web server
app = Flask(__name__)
current_frame = None  # To store the latest frame
frame_lock = Lock()  # Lock to safely access current_frame


def generate_frame():
    """Yield the latest frame for the web page."""
    global current_frame
    while True:
        with frame_lock:
            if current_frame is not None:
                _, buffer = cv2.imencode('.jpg', current_frame)
                frame_data = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
            else:
                yield b''


@app.route('/')
def index():
    """Home page displaying the video feed."""
    return Response(generate_frame(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


class ImageWebServerNode(Node):
    def __init__(self):
        super().__init__('image_web_server_node')
        self.subscription = self.create_subscription(
            Image,
            'rob498_drone_9/recovery/detected_image',  # Replace with your image topic name
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Image Web Server Node started")

    def image_callback(self, msg):
        """Callback to process the image."""
        global current_frame
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            with frame_lock:
                current_frame = cv_image  # Update the current frame
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    # Start the ROS2 node
    rclpy.init(args=args)
    node = ImageWebServerNode()

    # Start the Flask server in a separate thread
    flask_thread = Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000, 'debug': False})
    flask_thread.daemon = True  # Ensures Flask thread exits with the main program
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
