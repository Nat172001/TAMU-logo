#!/usr/bin/env python3
#run1 - 11 min 38 sec
#run2 - 9 min 58 sec
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from math import pow,sqrt,atan2,pi
from std_srvs.srv import Empty, SetBool
import numpy as np

class togoal(Node):
    def __init__(self):
        super().__init__("togoal")
        self.pub= self.create_publisher(Twist,"/cmd_vel",10)
        self.sub= self.create_subscription(Pose2D,"/pose",self.set_pose_variable,rclpy.qos.qos_profile_sensor_data)
        self.pose= Pose2D()

        self.linear_tol= 0.001
        self.angular_tol= 0.01 
        self.flag=True

        self.reset()
        self.goalpose = Pose2D()
        self.counter=1
        self.gt=[[-1.37, -4.57], [-1.37, -3.28], [-0.65, -3.28],[-0.65, 1.27], [-2.02, 1.27], 
                 [-2.02, 0.54], [-3.33, 0.54], [-3.33, 2.44], [3.31, 2.44], [3.31, 0.54],
                 [2.02, 0.54], [2.02, 1.27], [0.65, 1.27], [0.65, -3.28], [1.38, -3.28], 
                 [1.38, -4.57], [-1.37, -4.57],     

                 [-4.28, -2.87], [-4.28, -2.28], [-3.96, -2.28], 
                 [-3.17, -0.44], [-3.31, -0.44], [-3.31, 0.15],[-2.04, 0.15],[-2.04, -0.44],
                 [-2.17, -0.44],[-1.39, -2.28],[-1.07, -2.28],[-1.07, -2.87],[-2.22, -2.87],
                 [-2.22, -2.28],[-2.05, -2.28], [-2.19, -1.97], [-3.14, -1.97], [-3.26, -2.28], 
                 [-3.11, -2.28],[-3.11, -2.87],[-4.28, -2.87],
                 
                 [-2.87, -1.38],[-2.67, -0.88],
                 [-2.44, -1.38],[-2.87, -1.38],
                 
                 [1.07, -2.87],[1.07, -2.28],[1.31, -2.28],
                 [1.31, -0.44],[1.09, -0.44],[1.09, 0.15],[2.02, 0.15],[2.67, -1.17],
                 [3.31, 0.15],[4.28, 0.15],[4.28, -0.44],[4.04, -0.44],[4.04, -2.28],
                 [4.28, -2.28],[4.28, -2.87],[3.21, -2.87],[3.21, -2.28],[3.43, -2.28],
                 [3.43, -0.99],[2.67, -2.55],[1.92, -1.00],[1.92, -2.28],[2.16, -2.28],
                 [2.16, -2.87],[1.07, -2.87],
                 
                 [4.0,-4.0]]

        self.getgoalpose()
        self.timer=self.create_timer(0.05,self.movetogoal)

    def sim_color(self,opt):
        set_pen_client = self.create_client(SetBool, '/set_pen')
        while not set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set pen service not available, waiting again...')
        request = SetBool.Request()
        request.data = opt
        set_pen_client.call_async(request)
    
    def reset(self):
        clear_client = self.create_client(Empty, '/reset')
        while not clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset service not available, waiting again...')
        request = Empty.Request()
        clear_client.call_async(request)
    
    def getgoalpose(self):
        if(self.counter==1 or self.counter==18 or self.counter==39 or self.counter==43 or self.counter==68):
            self.sim_color(False)
            self.goalpose.x=(self.gt[self.counter-1][0])
            self.goalpose.y=(self.gt[self.counter-1][1])
        elif(self.counter!=len(self.gt)+1):
            self.sim_color(True)
            self.goalpose.x=(self.gt[self.counter-1][0]) #float(input("Enter the goal "+str(self.counter)+" x position:"))
            self.goalpose.y=(self.gt[self.counter-1][1]) #float(input("Enter the goal "+str(self.counter)+" y position:"))
        else:
            quit()
    
    def set_pose_variable(self, msg: Pose2D):
        self.pose.x=msg.x
        self.pose.y=msg.y
        self.pose.theta=(msg.theta+ pi) % (2 * pi) - pi
        #print("sub")

    def movetogoal(self):
        #print("pub")
        pub_msg = Twist() 

        kv=1 #0.8
        kw=4 #1.2

        distance=sqrt(pow(self.goalpose.x-self.pose.x,2)+pow(self.goalpose.y-self.pose.y,2))
        velocity= kv *distance  

        steering_angle = atan2(self.goalpose.y - self.pose.y, self.goalpose.x - self.pose.x)

        # Calculate the difference between the current orientation and the steering angle
        angle_diff = (steering_angle - self.pose.theta + pi) % (2 * pi) - pi

        angular_velocity = kw * angle_diff

        if abs(angle_diff) > self.angular_tol:
            pub_msg.linear.x = 0.0  
            pub_msg.angular.z = angular_velocity

        else:
            pub_msg.angular.z = 0.0 
            if distance > self.linear_tol:
                pub_msg.linear.x = velocity 
            else:
                pub_msg.linear.x = 0.0  
                self.get_logger().info("Goal " + str(self.counter) + " Reached!")
                self.counter += 1
                self.getgoalpose()
        self.pub.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)
    node=togoal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
