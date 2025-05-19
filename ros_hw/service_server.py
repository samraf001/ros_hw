import rclpy
from rclpy.node import Node
from ros_hw_interfaces.srv import ThreeDOF
from collections import deque
import socket
import numpy as np
import time

class SensorClient:
    def __init__(self, host='127.0.0.3', port=10000, dof=6):
        self.host = host
        self.port = port
        self.dof = dof
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))

    def request_samples(self, num_samples):
        try:
            # Send the number of samples to request
            message_string = str(num_samples)
            message = message_string.encode()
            self.sock.sendall(message)

            # Receive the data
            byte_data = self.sock.recv(10000)
            data = np.frombuffer(byte_data, dtype=np.float32)

            expected_size = self.dof * num_samples
            print(f"[SensorClient] Expected size: {expected_size}, Received: {len(data)}")

            if len(data) != expected_size:
                print("[SensorClient] Warning: Data length mismatch. Trimming data.")
                data = data[:expected_size]  # Trimming excess data

            # Reshape the data to (DOF, num_samples)
            reshaped_data = data.reshape(self.dof, num_samples)

            print(f"[SensorClient] Data shape after reshape: {reshaped_data.shape}")
            return reshaped_data
        
        except (socket.error, ConnectionResetError) as e:
            print(f"[SensorClient] Error: {e}")
            self.connect()  # Reconnect if connection is lost
            return None
        except Exception as e:
            print(f"[SensorClient] Unexpected error: {e}")
            return None


class DataServiceServer(Node):

    def __init__(self):
        super().__init__('data_service_tcp_server')

        # ROS service for raw data
        self.raw_srv = self.create_service(ThreeDOF, 'get_raw_3dof_data', self.get_raw_data_callback)

        # ROS service for cleaned data
        self.cleaned_srv = self.create_service(ThreeDOF, 'get_cleaned_3dof_data', self.get_cleaned_data_callback)

        # FIFO buffer for raw data
        self.buffer_size = 100
        self.x_buffer = deque(maxlen=self.buffer_size)
        self.y_buffer = deque(maxlen=self.buffer_size)
        self.z_buffer = deque(maxlen=self.buffer_size)

        # TCP client to sensor
        self.sensor = SensorClient(host='127.0.0.3', port=10000)

        # Timer to poll sensor at 50Hz
        self.create_timer(0.02, self.poll_sensor)

        self.get_logger().info("TCP-based data service server started")

    def poll_sensor(self):
        samples = self.sensor.request_samples(10)  # Request 10 samples
        if samples is not None:
            x, y, z = samples[0], samples[1], samples[2]
            # Add raw data to buffer
            self.x_buffer.extend(x)
            self.y_buffer.extend(y)
            self.z_buffer.extend(z)

    def get_raw_data_callback(self, request, response):
        n = request.num_samples
        available = min(n, len(self.x_buffer))

        response.x = [float(v) for v in list(self.x_buffer)[-available:]]
        response.y = [float(v) for v in list(self.y_buffer)[-available:]]
        response.z = [float(v) for v in list(self.z_buffer)[-available:]]

        self.get_logger().info(f"Returned {available} raw samples.")
        return response

    def clean_data(self, data):
        """Clean the data by removing NaN and limiting extreme values."""
        # Replace NaN with 0
        data = np.nan_to_num(data)

        # Clamp values to a reasonable range (example: -10 to 10)
        min_value = -10  # Replace with a suitable min value
        max_value = 10   # Replace with a suitable max value
        data = np.clip(data, min_value, max_value)

        return data

    def get_cleaned_data_callback(self, request, response):
        n = request.num_samples
        available = min(n, len(self.x_buffer))

        response.x = [float(v) for v in self.clean_data(list(self.x_buffer)[-available:])]
        response.y = [float(v) for v in self.clean_data(list(self.y_buffer)[-available:])]
        response.z = [float(v) for v in self.clean_data(list(self.z_buffer)[-available:])]

        self.get_logger().info(f"Returned {available} cleaned samples.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DataServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
