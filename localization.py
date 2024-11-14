import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

class localization(Node):
    
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt):
        
        # TODO Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)

        # Set an inital pose for [x, y, th, w, v, vdot]
        x= np.array([0, 0, 0, 0, 0, 0])
        
        # Me modify the scaling of Q to test
        Q= 0.5 * np.eye(6)

        # Modify the scaling of R to test
        R= 0.5 * np.eye(4)
        
        # The initial P isn't crucial to get right, the algorithmn should correct
        P= np.eye(6) # initial covariance
        
        self.kf=kalman_filter(P,Q,R, x, dt)
        
        # DONE Part 3: Use the odometry and IMU data for the EKF
        self.odom_sub=message_filters.Subscriber(self, odom, '/odom', qos_profile=odom_qos)
        self.odom_sub.registerCallback(self.odom_callback) # Must setup callback
        
        # Does not need individual data processing, so no seperate imu_callback
        self.imu_sub=message_filters.Subscriber(self, Imu, '/imu')
        
        # Combine the msg's and pass them into a combined callback
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # DONE Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        
        # Velocity taken as velocity magnitude of x and y
        # Will change if there is no measured vy from odom
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        v = np.sqrt(vx**2 + vy**2)

        # Angular velocity about z axis
        w = odom_msg.twist.twist.angular.z

        # Acceleration values extracted from IMU
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y

        # z must be a numpy array
        z = np.array([v, w, ax, ay])
        
        # Implement the two steps for estimation
        self.kf.predict() # Prediction step
        self.kf.update(z) # Update step
        
        # Get the estimate
        xhat=self.kf.get_states()

        # Update the pose estimate to be returned by getPose
        self.pose=np.array(xhat)

        # DONE Part 4: log your data
        # self.pose = [x, y, th, w, v, vdot]
        self.loc_logger.log_values([
            ax, # imu_ax
            ay, # imu_ay
            self.pose[5], # kf_ax
            self.pose[4] * self.pose[3], # kf_ay
            self.pose[4], # kf_vx
            self.pose[3], # kf_w
            self.pose[0], # kf_x
            self.pose[1], # kf_y
            Time.from_msg(imu_msg.header.stamp).nanoseconds # stamp
        ])
      
    def odom_callback(self, pose_msg):
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
