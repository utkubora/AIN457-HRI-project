#!/usr/bin/env python3

import sys
import time
import math
from pathlib import Path

import rospy  # Python library for ROS
from sensor_msgs.msg import LaserScan  # import library for lidar sensor
from nav_msgs.msg import Odometry  # import library for position and orientation data
from geometry_msgs.msg import Twist, Pose

from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ContactsState

import pygame
from pynput import keyboard

from fer import FER
import cv2

# rospy.init_node('object_spawner')

from tensorflow.python.client import device_lib
print(device_lib.list_local_devices())

class Controller:  # main class
    def __init__(self, spawn_model_proxy, delete_model_proxy):  # main function
        global twist_msg

        self.spawn_model_proxy = spawn_model_proxy
        self.delete_model_proxy = delete_model_proxy

        twist_msg = Twist()  # create object of twist type
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # publish message
        self.sub = rospy.Subscriber(
            "/my_bot/contact", ContactsState, self.collision_callback
        )
        self.laser = rospy.Subscriber(
            "/scan", LaserScan, self.laser_callback
        )  # subscribe message
        self.odometri = rospy.Subscriber("/odom", Odometry, self.odometri_callback)

        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        pygame.joystick.init()
        pygame.display.init()

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            rate = rospy.Rate(10)  # 10 Hz

            while not rospy.is_shutdown():
                self.pub.publish(twist_msg)
                self.update_gamepad()
                rate.sleep()

        else:
            print("No gamepad detected!")

            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown():
                self.pub.publish(twist_msg)
                rate.sleep()

    def emot_detect(self):
        ###################
        # Emotion Capture #
        ###################

        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        detector = FER(mtcnn=True)
        emotion, score = detector.top_emotion(frame)

        rospy.loginfo(f"Emotion '{emotion}' with the confidence score of {score}'")

        if emotion == "angry":
            rospy.loginfo("Angry person detected. RUN!")
            self.move_away()

        elif emotion == "happy":
            rospy.loginfo("Aww look how happy he is. Going to draw a circle.")
            self.circle()

        elif emotion == "sad":
            rospy.loginfo(
                "It's okay to be sad sometimes. Here, take this to make things worse."
            )
            self.spawn_beer()

        else:
            rospy.loginfo("emotion not detected")

    def laser_callback(self, laser):
        self.laser = laser

    def odometri_callback(self, odometri):
        self.odometri = odometri

    def collision_callback(self, msg):
        # Set velocities to zero to stop the bot
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub.publish(twist_msg)
        rospy.loginfo("Collision detected!")

    def position_callback(self):
        print("test")
        x = self.odometri.pose.pose.position.x
        y = self.odometri.pose.pose.position.y
        print("test 2")
        print(str(x) + "," + str(y))

    def on_press(self, key):
        max_speed_x = 1.2
        max_speed_z = 0.4

        # Update twist_msg based on the pressed key
        if key == keyboard.Key.up or key.char == "w":
            x = 0.5 + twist_msg.linear.x
            twist_msg.linear.x = min(max_speed_x, x)
        elif key == keyboard.Key.down or key.char == "x":
            x = -0.5 + twist_msg.linear.x
            twist_msg.linear.x = max(-max_speed_x, x)

        if key == keyboard.Key.left or key.char == "a":
            z = twist_msg.angular.z + 0.6
            twist_msg.angular.z = min(max_speed_z, z)
        elif key == keyboard.Key.right or key.char == "d":
            z = twist_msg.angular.z - 0.6
            twist_msg.angular.z = max(-max_speed_z, z)

        if key.char == "s":
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0

        if key.char == "b":
            self.spawn_beer()

        if key.char == "c":
            self.circle()

        if key.char == "p":
            self.move_away()

        if key.char == "u":
            self.emot_detect()

    def move_away(self):
        print("-------RECEIVING LIDAR SENSOR DATA-------")
        print("Front: {}".format(self.laser.ranges[0]))  # lidar data for front side
        print("Left: {}".format(self.laser.ranges[90]))  # lidar data for left side
        print("Right: {}".format(self.laser.ranges[270]))  # lidar data for right side
        print("Back: {}".format(self.laser.ranges[180]))  # lidar data for back side

        mini = min(self.laser.ranges)
        argmin = self.laser.ranges.index(mini)
        rad_min = math.radians(argmin)

        x_min = math.cos(rad_min)
        y_min = math.sin(rad_min)

        twist_msg.linear.x = -x_min
        twist_msg.linear.y = -y_min

        time.sleep(3)

        twist_msg.linear.x = 0
        twist_msg.linear.y = 0

    def on_release(self, key):
        # Stop the bot when the key is released
        if key in (
            keyboard.Key.up,
            keyboard.Key.down,
            keyboard.Key.left,
            keyboard.Key.right,
            "w",
            "s",
            "a",
            "d",
        ):
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

    def update_gamepad(self):
        # button kontrolü ekle
        pygame.event.pump()

        if self.joystick.get_button(5) != 0:
            rospy.loginfo("goodbye")
            sys.exit()

        if self.joystick.get_button(3) != 0:
            rospy.loginfo("brake")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        elif self.joystick.get_button(4) != 0:
            
            
            print("--------------------------------------")
            if self.joystick.get_axis(1) < -0.50 or self.joystick.get_axis(1) > 0.50:
                print("linear:" , (self.joystick.get_axis(1)))
                twist_msg.linear.x = -(self.joystick.get_axis(1)) * 1.3
                
            else: 
                twist_msg.linear.x = 0.0
            
            if self.joystick.get_axis(3) < -0.50 or self.joystick.get_axis(3) > 0.50:
                print("angular:" , (self.joystick.get_axis(3)))
                twist_msg.angular.z = -(self.joystick.get_axis(3)) * 0.8
            else:
                twist_msg.angular.z = 0.0

    def spawn_beer(self):
        model_name = "beer_can"
        model_path = '/home/utku/gazebo_models/gazebo_models/beer/model.sdf'  # Replace with the actual path to beer_can.sdf
        
        pose = Pose()
        pose.position.x = self.odometri.pose.pose.position.x
        pose.position.y = self.odometri.pose.pose.position.y
        pose.position.z = 0.0
        
        self.spawn_object(model_name, model_path, pose)

    def circle(self):  # function for obstacle avoidance
        print("-------RECEIVING LIDAR SENSOR DATA-------")

        print("Front: {}".format(self.laser.ranges[0]))  # lidar data for front side
        print("Left: {}".format(self.laser.ranges[90]))  # lidar data for left side
        print("Right: {}".format(self.laser.ranges[270]))  # lidar data for right side
        print("Back: {}".format(self.laser.ranges[180]))  # lidar data for back side

        # Obstacle Avoidance
        self.distance = 0.7

        if (
            self.laser.ranges[0] > self.distance
            and self.laser.ranges[15] > self.distance
            and self.laser.ranges[345] > self.distance
        ):
            # when no any obstacle near detected
            twist_msg.linear.x = 2  # go (linear velocity)
            twist_msg.angular.z = 0.4  # rotate (angular velocity)
            rospy.loginfo("Circling")  # state situation constantly

        else:  # when an obstacle near detected
            rospy.loginfo("An Obstacle Near Detected")  # state case of detection
            twist_msg.linear.x = 0.0  # stop
            twist_msg.angular.z = 0.7  # rotate counter-clockwise

        if (
            self.laser.ranges[0] > self.distance
            and self.laser.ranges[15] > self.distance
            and self.laser.ranges[345] > self.distance
            and self.laser.ranges[45] > self.distance
            and self.laser.ranges[315] > self.distance
        ):
            # when no any obstacle near detected after rotation
            twist_msg.linear.x = 2  # go
            twist_msg.angular.z = 0.4  # rotate

        self.pub.publish(twist_msg)  # publish the move object

    def spawn_object(self, model_name, model_path, pose, reference_frame="world"):
        model_xml = Path(model_path).read_text()

        response = self.spawn_model_proxy(
            model_name, model_xml, "", pose, reference_frame
        )

        if response.success:
            rospy.loginfo(f"Successfully spawned {model_name}")

        else:
            rospy.logerr(f"Failed to spawn {model_name}: {response.status_message}")

    def delete_object(self, model_name):
        request = DeleteModel()
        request.model_name = model_name

        response = delete_model_proxy(request)

        if response.success:
            rospy.loginfo(f"Successfully deleted {model_name}")

        else:
            rospy.logerr(f"Failed to delete {model_name}: {response.status_message}")


if __name__ == "__main__":
    rospy.init_node("object_spawner")  # initilize node

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/delete_model")
    spawn_model_proxy = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_model_proxy = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    Controller(spawn_model_proxy, delete_model_proxy)  # run class
    rospy.spin()  # loop it
