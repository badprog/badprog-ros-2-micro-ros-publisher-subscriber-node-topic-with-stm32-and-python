# https://github.com/badprog

# Libraries
import rclpy # Import Ros Client Library Python
from rclpy.node import Node # Import class Node
from std_msgs.msg import Bool, Int32 # Import classes
from pathlib import Path # Import class Path

NODE_PC = 'badprog_node_pc'
TOPIC_COUNTER = 'badprog_topic_counter'
TOPIC_LED_STATE = 'badprog_topic_led_state'

# ------------------------------------------------------
# BadprogNodePC
# ------------------------------------------------------
class BadprogNodePC(Node):
    """
    ROS 2 node with a parent (inheriting from Node).
    """
    # ------------------------------------------------------
    # __init__
    # Constructor
    # ------------------------------------------------------
    def __init__(self):
        """
        Class constructor
        Init the publisher and the timer for a blinking mode.
        """
        super().__init__(NODE_PC) # Init the parent constructor (here Node)
        
        # Create a publisher talking on the same topic as the STM32 (here we publish)
        self.publisher_ = self.create_publisher(Bool, TOPIC_LED_STATE, 10)
        
        # Create a subscriber to listen form the STM32
        self.subscription = self.create_subscription(Int32, TOPIC_COUNTER, self.listener_callback, 10)
                
        # Call a timer every 0.5 s.
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.led_state: bool = False

    # ------------------------------------------------------
    # timer_callback
    # ------------------------------------------------------
    def timer_callback(self):
        """
        Change the led_state once a message is sent on the topic.
        """
        msg = Bool() # Type of the message
        msg.data = self.led_state
        self.publisher_.publish(msg) # Publish the message
        self.get_logger().info(f'The LED is turned: {"ON" if self.led_state else "OFF"}')
        
        # Invert the state for the next callback
        self.led_state = not self.led_state
        
    
    # ------------------------------------------------------
    # listener_callback
    # ------------------------------------------------------
    def listener_callback(self, msg: Int32):
        """ 
        Called each time the STM32 node sends a message. 
        """
        self.get_logger().info(f'---> Received from STM32: Counter = {msg.data}')

# ------------------------------------------------------
# main
# ------------------------------------------------------
def main(args=None):
    """
    Init and clean ROS 2.
    """
    rclpy.init(args=args) # Init RCLPY
    node = BadprogNodePC() # Instance of a node
    fileName = Path(__file__).name
    
    try:
        rclpy.spin(node) # Infinite loop if everything is OK
    except KeyboardInterrupt:
        # pass # Stop the program with CTRL + C without error displayed
        print(f'\n[INFO] [Stop] Shutting down "{fileName}".')
    finally:
        node.destroy_node() # End of node instance
        if rclpy.ok():
            rclpy.shutdown() # Clean the memory and close the ROS 2 connection

# ------------------------------------------------------
# Main guard
# ------------------------------------------------------
if __name__ == '__main__':
    main()
