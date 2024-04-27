#!/usr/bin/env python3
import rclpy
import board
import adafruit_veml7700
from rclpy.node import Node
from std_msgs.msg import Int32

class LightSensorNode(Node):
    
    def __init__(self):
        super().__init__("light_sensor")
        i2c = board.I2C()
        self.veml7700 = adafruit_veml7700.VEML7700(i2c)
        self.publisher_ = self.create_publisher(Int32, 'light_reading', 10)
        self.create_timer(0.1, self.publish_light_reading)

    def publish_light_reading(self):
        
        msg = Int32()
        msg.data = self.veml7700.light
        self.publisher_.publish(msg)


        




def main(args=None):
    rclpy.init(args=args)
    node = LightSensorNode()
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
