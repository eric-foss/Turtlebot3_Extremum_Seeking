import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time
import numpy as math

class mycontrolbottunefv(Node):
    def __init__(self):
        super().__init__('mycontrolbottunefv')
        self.get_parameters()
        self.subscription = self.create_subscription(Int32, '/light_reading', self.update_h, 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        #self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.1, self.update_h_and_control)
        self.start_time = time.time()
        

    def get_parameters(self):
        self.V = 0.0 #inital Velocity
        self.w = float(input('Enter value for ω: '))
        self.k = float(input('Enter value for k: '))
        self.c = float(input('Enter value for Filter Coefficient c: '))
        self.OMEGA = float(input('Enter Angular Velocity OMEGA: '))
        self.a = float(input('Enter value for Alfa: '))
        self.z = 0.0# initial z, the range
        self.h = 0.0  # initial h, the range
        self.dt = 0.1 #time stamp based on self.timer
        self.theta = 0.0
 #       self.elapsed_time = 0.0


    def calculate_vr_vl(self):
        thetadot = self.OMEGA
        self.theta += thetadot * self.dt
        self.V = (self.a*self.w)**(1/2)*math.cos(self.w*(time.time()-self.start_time)+ self.k * ((-self.h) - self.z))
        #self.VR = (2*self.V+self.OMEGA*0.142)/(2*0.142)
        #self.VL = (2*self.V-self.OMEGA*0.142)/(2*0.142)
        #self.get_logger().info(f'VR = {self.VR}')
        #self.get_logger().info(f'VL = {self.VL}')
        self.get_logger().info(f'time = {(time.time()-self.start_time)}')
  	 
    def update_h(self, msg):
        self.h = msg.data
        self.get_logger().info(f'Initial h = {self.h}')

    def lowpass_filter(self):    
        zdot = float((-self.z - self.h) * self.c)  
        self.z += zdot * self.dt
        self.get_logger().info(f'z(filtered light reading) = {self.z}')
        self.get_logger().info(f'zdot = {zdot}')
 
   
    def update_h_and_control(self):  # Update 'h' every 0.1s
        elapsed_time = time.time() - self.start_time #calculate the time difference between the actual time and the start time
        print(f'Elapsed Time: {elapsed_time} seconds')    
        self.lowpass_filter()
        self.calculate_vr_vl()
        self.control_turtlebot()

    def control_turtlebot(self):
        msg = Twist()
        msg.linear.x = self.V
        msg.angular.z = self.OMEGA
        self.get_logger().info(f'Linear: {msg.linear.x}, Angular: {msg.angular.z}')
        if abs(self.V) > 0.219:
            self.get_logger().info('V IS TOO BIG !!!!!!!!')
        self.cmd_vel_pub_.publish(msg)
	        

def main(args=None):
    rclpy.init(args=args)
    control_node = mycontrolbottunefv()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

