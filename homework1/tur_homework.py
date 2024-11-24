#!/usr/bin/env python3

import rospy
from tkinter import *
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import SetPen, Spawn

class setup:
    def __init__(self):
        rospy.init_node('Turtle_Control')
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Adjusting the GUI window size
        self.frame = Tk()
        self.frame.title('Turtle_Control')
        self.frame.geometry('520x130')  # Adjusted size to fit 4 rows and 3 columns
        
    def main(self):
        mini_width = 5
        mini_height = 1
        
        button_width = 10
        button_height = 1

        # Row 0 - Movement Buttons
        Button(self.frame, text='TL', command=self.turn_left, width=mini_width, height=mini_height).grid(row=0, column=0, padx=10, pady=5)
        Button(self.frame, text='FW', command=self.forward, width=mini_width, height=mini_height).grid(row=0, column=1, padx=10, pady=5)
        Button(self.frame, text='TR', command=self.turn_right, width=mini_width, height=mini_height).grid(row=0, column=2, padx=10, pady=5)
        
        # Row 1 - Slide Left/Right and Empty Space
        Button(self.frame, text='SL', command=self.slide_left, width=mini_width, height=mini_height).grid(row=1, column=0, padx=10, pady=5)
        Label(self.frame, text="", width=mini_width, height=mini_height).grid(row=1, column=1)
        Button(self.frame, text='SR', command=self.slide_right, width=mini_width, height=mini_height).grid(row=1, column=2, padx=10, pady=5)
        
        # Row 2 - Backward and Turn Back
        Button(self.frame, text='TBL', command=self.turn_back_left, width=mini_width, height=mini_height).grid(row=2, column=0, padx=10, pady=5)
        Button(self.frame, text='BW', command=self.backward, width=mini_width, height=mini_height).grid(row=2, column=1, padx=10, pady=5)
        Button(self.frame, text='TBR', command=self.turn_back_right, width=mini_width, height=mini_height).grid(row=2, column=2, padx=10, pady=5)

        # Row 3 - Pen Off, Pen On, Clear, and Spawn buttons
        Button(self.frame, text='Pen Off', command=self.pen_off, width=button_width, height=button_height).grid(row=0, column=3, padx=10, pady=5)
        Button(self.frame, text='Pen On', command=self.pen_on, width=button_width, height=button_height).grid(row=1, column=3, padx=10, pady=5)
        Button(self.frame, text='Clear', command=self.clear_pen, width=button_width, height=button_height).grid(row=2, column=3, padx=10, pady=5)
        Button(self.frame, text='Spawn', command=self.spawn_turtle, width=button_width, height=button_height).grid(row=0, column=4, padx=10, pady=5)
        Button(self.frame, text='Reset', command=self.reset_turtle, width=button_width, height=button_height).grid(row=1, column=4, padx=10, pady=5)

        self.frame.mainloop()

    def pen_off(self):
        rospy.wait_for_service('/turtle1/set_pen')
        try:
            set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
            set_pen(0, 0, 0, 0, 1)  # 1 turns off the pen
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def pen_on(self):
        rospy.wait_for_service('/turtle1/set_pen')
        try:
            set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
            set_pen(255, 87, 51, 3, 0)  # Red pen, width 3, pen ON
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def clear_pen(self):
        rospy.wait_for_service('/clear')
        try:
            clear = rospy.ServiceProxy('/clear', Empty)
            clear()  # Clear the screen
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            
    def reset_turtle(self):
        rospy.wait_for_service('/reset')
        try:
            reset_tur = rospy.ServiceProxy('/reset', Empty)
            reset_tur()
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def spawn_turtle(self):
        rospy.wait_for_service('/spawn')
        try:
            spawn = rospy.ServiceProxy('/spawn', Spawn)
            spawn(5.5, 5.5, 0.0, 'turtle2')
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def forward(self):
        print('Forward')
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)

    def backward(self):
        print('Backward')
        cmd = Twist()
        cmd.linear.x = -1.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        
    def turn_left(self):
        print('Turn Left')
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = 1.0
        self.pub.publish(cmd)


    def turn_right(self):
        print('Turn Right')
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = -1.0
        self.pub.publish(cmd)

    def turn_back_left(self):
        print('Turn Back Left')
        cmd = Twist()
        cmd.linear.x = -0.5
        cmd.angular.z = -1.0
        self.pub.publish(cmd)

    def turn_back_right(self):
        print('Turn Back Right')
        cmd = Twist()
        cmd.linear.x = -0.5
        cmd.angular.z = 1.0
        self.pub.publish(cmd)
        
    def slide_left(self):
        print('Slide Left')
        cmd = Twist()
        cmd.linear.y = 1.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)

    def slide_right(self):
        print('Slide Right')
        cmd = Twist()
        cmd.linear.y = -1.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)


if __name__ == '__main__':
    gui = setup()
    gui.main()
