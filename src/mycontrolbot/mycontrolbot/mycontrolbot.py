import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time
import numpy as math
from std_msgs.msg import Float32MultiArray
import os
import datetime


class mycontrolbot(Node):
    def __init__(self):
        super().__init__('mycontrolbot')
        self.get_parameters()
        self.subscription = self.create_subscription(Int32, '/light_reading', self.update_h, 10)                
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer_ = self.create_timer(0.1, self.update_h_and_control)
        self.start_time = time.time()
        self.h_vector =[]

    def get_parameters(self):
        self.V = float(input('Enter value for V: '))
        self.omega = float(input('Enter value for ω: '))
        self.k = float(input('Enter value for k: '))
        self.c = float(input('Enter value for Filter Coefficient c: '))      
        self.z = 0.0# initial z, the range
        self.h = 0.0  # initial h, the range
        self.Lambda = float(input('Enter value for Lambda: '))


    def calculate_vr_vl(self):
        if self.elapsed_time < 0.2:
            self.z = -self.h    
    	#if self.elapsed_time < 0.2:
    	    #self.z = self.h
        delta = self.omega + self.k *math.exp(2*self.Lambda*self.elapsed_time)* ((-self.h) - self.z) #big omega
        self.VR = ((0.144 * delta) + (2 * self.V)) / 2
        self.VL = (2 * self.V - (self.VR))
        self.get_logger().info(f'VR = {self.VR}')
        self.get_logger().info(f'VL = {self.VL}')
        self.get_logger().info(f'z = {self.z}')   
        
    def update_h(self, msg):
        self.h = msg.data
        self.get_logger().info(f'Initial h = {self.h}')

    def lowpass_filter(self):
        dt = 0.1 #time stamp based on self.timer       
        zdot = float((-self.z - self.h) * self.c)
        self.z += zdot * dt
      #  self.get_logger().info(f'z = {self.z}')
      #  self.get_logger().info(f'zdot = {zdot}')
 
   
    def update_h_and_control(self):  # Update 'h' every 0.1s
        self.elapsed_time = time.time() - self.start_time #calculate the time difference between the actual time and the start time
        print(f'Elapsed Time: {self.elapsed_time} seconds')    
        self.lowpass_filter()
        self.calculate_vr_vl()
        self.control_turtlebot()
        self.h_vector.append(self.h)
       # self.get_logger().info(f'h vector = {self.h_vector}')
        return self.h_vector



    def control_turtlebot(self):
        msg = Twist()
        msg.linear.x = math.exp(-self.Lambda*self.elapsed_time)*((self.VR + self.VL) / 2)
        msg.angular.z = (self.VR - self.VL) / 0.178  # Use 0.23 as the constant
        #if msg.angular.z >2.639:
        #    msg.angular.z = 2.639
        #if msg.angular.z < -2.639:
        #    msg.angular.z = -2.639
        self.cmd_vel_pub_.publish(msg)
        self.get_logger().info(f'Linear: {msg.linear.x}, Angular: {msg.angular.z}')
        if abs(msg.linear.x) > 0.219:
            self.get_logger().info('V IS TOO BIG !!!!!!!!')
        if abs(msg.angular.z) > 2.639:
            self.get_logger().info('OMEGA IS TOO BIG !!!!!!!!')
        if abs(self.VR) > 0.219:
            self.get_logger().info('VR BIG !')
        if abs(self.VL) > 0.219:
            self.get_logger().info('VL BIG !')            

    def log_data(self):
        log_directory = 'logs'
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        log_file = os.path.join(log_directory, 'measurement_data.txt')
        with open(log_file, 'a') as f:
            f.write(f'Timestamp: {datetime.datetime.now()}, h: {self.h}\n')

def main(args=None):
    rclpy.init(args=args)
    control_node = mycontrolbot()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

