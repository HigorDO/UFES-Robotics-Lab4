# Este código é uma derivação do código go to goal dos treinamentos de ros
# 
# Author: Higor David Oliveira
# Date: 08/02/23
import rclpy
import rospy2 as rospy

from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class TurtleBot(Node):

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        super().__init__('turtlebot_controller')

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10
                                                )

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose',
                                                callback=self.update_pose, qos_profile=10)
        # HDO/goal node subscriber
        self.pose_subscriber_2 = self.create_subscription(Pose2D, '/HDO/goal',
                                                callback=self.update_pose2D, qos_profile=10)


        self.goal_pose = Pose2D()
        self.pose = Pose()

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        # print("Pose added")

    def update_pose2D(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber. Adds a target point for turtle bot."""
        self.goal_pose = data
        print("Pose2D goal added, calling move to goal function")
        self.move2goal_pose2D()

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = float(input("Set your tolerance: "))

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        #rospy.spin()

    def move2goal_pose2D(self, data: Pose2D = Pose2D()):
        """Moves the turtle to the goal using data listened from subscribe."""
        # self.rate = rospy.Rate(10)
        # goal_pose = Pose()
        # goal_pose.x = data.x
        # goal_pose.y = data.y
        # goal_pose.theta = data.theta

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        # distance_tolerance = float(input("Set your tolerance: "))
        distance_tolerance = 0.5

        vel_msg = Twist()

        if self.euclidean_distance(self.goal_pose) >= distance_tolerance:
            #print("Goal pose: (%.1f, %.1f, %.1f); Distance: %.2f; tolerance: %.1f", self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta, self.euclidean_distance(self.goal_pose), distance_tolerance)

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control
            
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(self.goal_pose)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(self.goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            rospy.sleep(0.1)         
        
        # Stopping our robot after the movement is over.
        else: 
            #print("Checkpoint reached")
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

        self.velocity_publisher.publish(vel_msg)

        # # If we press control + C, the node will stop.
        # rospy.spin()


def main():
    try:
        x = TurtleBot()

        x.move2goal_pose2D()
        
        rclpy.spin(x)
    except rospy.ROSInterruptException:
        pass