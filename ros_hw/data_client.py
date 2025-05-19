import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32MultiArray
from ros_hw_interfaces.srv import ThreeDOF


class DataClient(Node):
    def __init__(self):
        super().__init__('data_client')

        # Declare the 'publish_frequency' parameter with a default value of 10.0 Hz
        self.declare_parameter('publish_frequency', 10.0)

        # Publishers
        self.raw_pub = self.create_publisher(Float32MultiArray, 'raw_data', 10)
        self.cleaned_pub = self.create_publisher(Float32MultiArray, 'cleaned_data', 10)

        # Clients
        self.raw_client = self.create_client(ThreeDOF, 'get_raw_3dof_data')
        self.cleaned_client = self.create_client(ThreeDOF, 'get_cleaned_3dof_data')

        # Wait for both services to become available
        self.get_logger().info("Waiting for services...")
        while not self.raw_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for raw service...')
        while not self.cleaned_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for cleaned service...')
        self.get_logger().info("Services found. Starting timer.")

        # Initialize the timer with the current frequency
        self.timer = None
        self.update_timer()

        # Add a callback to handle parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def update_timer(self):
        # Cancel the existing timer if it exists
        if self.timer is not None:
            self.timer.cancel()

        # Get the current frequency from the parameter
        frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # Validate the frequency
        if frequency <= 0.0:
            self.get_logger().warn("publish_frequency must be greater than 0.0 Hz. Timer not created.")
            self.timer = None
            return

        # Create a new timer with the updated frequency
        period = 1.0 / frequency
        self.timer = self.create_timer(period, self.publish_data)
        self.get_logger().info(f"Timer updated to {frequency} Hz.")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'publish_frequency' and param.type_ == param.Type.DOUBLE:
                if param.value <= 0.0:
                    self.get_logger().warn("publish_frequency must be greater than 0.0 Hz. Ignoring update.")
                    return SetParametersResult(successful=False)
                self.update_timer()
        return SetParametersResult(successful=True)

    def publish_data(self):
        # Call both services asynchronously
        raw_req = ThreeDOF.Request()
        raw_req.num_samples = 10
        raw_future = self.raw_client.call_async(raw_req)
        raw_future.add_done_callback(self.handle_raw_response)

        clean_req = ThreeDOF.Request()
        clean_req.num_samples = 10
        clean_future = self.cleaned_client.call_async(clean_req)
        clean_future.add_done_callback(self.handle_cleaned_response)

    def handle_raw_response(self, future):
        try:
            response = future.result()
            raw_msg = Float32MultiArray()
            raw_msg.data = response.x + response.y + response.z
            self.raw_pub.publish(raw_msg)
            self.get_logger().info(f"Published raw data: {len(raw_msg.data)} samples")
        except Exception as e:
            self.get_logger().error(f"Raw service call failed: {e}")

    def handle_cleaned_response(self, future):
        try:
            response = future.result()
            clean_msg = Float32MultiArray()
            clean_msg.data = response.x + response.y + response.z
            self.cleaned_pub.publish(clean_msg)
            self.get_logger().info(f"Published cleaned data: {len(clean_msg.data)} samples")
        except Exception as e:
            self.get_logger().error(f"Cleaned service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DataClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
