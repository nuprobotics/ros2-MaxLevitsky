import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')

        self.declare_parameter('service_name', '/trigger_service')
        self.declare_parameter('default_string', 'No service available')

        service_name = self.get_parameter('service_name').value
        self.default_string = self.get_parameter('default_string').value

        self.stored_response = None

        self.call_external_service()

        self.srv = self.create_service(Trigger, service_name, self.handle_service)
        self.get_logger().info(f'Service {service_name} is ready')

    def call_external_service(self):
        client = self.create_client(Trigger, '/spgc/trigger')

        if client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.done():
                try:
                    response = future.result()
                    self.stored_response = response.message
                    self.get_logger().info(f'Received response from /spgc/trigger: {response.message}')
                except Exception as e:
                    self.get_logger().warn(f'Service call failed: {e}')
                    self.stored_response = None
            else:
                self.get_logger().warn('Service call timed out')
                self.stored_response = None
        else:
            self.get_logger().warn('/spgc/trigger service not available')
            self.stored_response = None

    def handle_service(self, request, response):
        if self.stored_response is not None:
            response.success = True
            response.message = self.stored_response
        else:
            response.success = False
            response.message = self.default_string

        self.get_logger().info(f'Returning: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
