#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
import time

class PersonVelocityCalculator:
    def __init__(self):
        rospy.init_node('person_velocity_calculator', anonymous=True)
        
        self.last_pose = None
        self.last_time = None
        
        self.pose_subscriber = rospy.Subscriber('person_tracked', PoseStamped, self.pose_callback)
        self.velocity_publisher = rospy.Publisher('person_velocity', Vector3, queue_size=10)
        
    def pose_callback(self, pose_msg):
        current_time = time.time()
        
        if self.last_pose is not None and self.last_time is not None:
            time_diff = current_time - self.last_time
            position_diff_x = pose_msg.pose.position.x - self.last_pose.pose.position.x
            position_diff_y = pose_msg.pose.position.y - self.last_pose.pose.position.y
            position_diff_z = pose_msg.pose.position.z - self.last_pose.pose.position.z
            
            velocity_x = position_diff_x / time_diff
            velocity_y = position_diff_y / time_diff
            velocity_z = position_diff_z / time_diff
            
            velocity_msg = Vector3(velocity_x, velocity_y, velocity_z)
            self.velocity_publisher.publish(velocity_msg)
            
            print("Person Velocity (m/s):")
            print("X:", velocity_x)
            print("Y:", velocity_y)
            print("Z:", velocity_z)
            print("===========================")
        
        self.last_pose = pose_msg
        self.last_time = current_time
        
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        velocity_calculator = PersonVelocityCalculator()
        velocity_calculator.run()
    except rospy.ROSInterruptException:
        pass
