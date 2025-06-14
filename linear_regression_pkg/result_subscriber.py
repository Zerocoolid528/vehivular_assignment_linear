import rclpy
from rclpy.node import Node

# Import message type used for receiving regression results
from std_msgs.msg import Float32MultiArray

class ResultSubscriber(Node):
    def __init__(self):
                super().__init__('result_subscriber')

        # Create a subscription to the 'regression_results' topic
        # The message type is Float32MultiArray, and messages are handled by self.on_message
        self.create_subscription(
            Float32MultiArray,   # Message type
            'regression_results', # Topic name
            self.on_message,      # Callback function
            10                    # QoS profile (queue size)
        )

    # Callback function that is called when a message is received
    def on_message(self, msg):
        dataset_id, mae, r2 = msg.data
        # Log the received result for the dataset
        self.get_logger().info(
            f"Received from dataset {int(dataset_id)}: MAE={mae:.2f}, R2={r2:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)         
    node = ResultSubscriber()     
    rclpy.spin(node)              
    rclpy.shutdown()              

if __name__ == '__main__':
    main()
