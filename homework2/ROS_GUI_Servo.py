#!/usr/bin/env python3

from tkinter import *
import rospy
from std_msgs.msg import Int16, Int8

# Initialize the GUI
frame = Tk()
frame.geometry("250x150")
frame.title("Gui_Control_Servo_Led")

class Gui:
    def __init__(self) -> None:
        rospy.init_node("GUI_SERVO_Control", anonymous=True)
        
        # ROS publishers
        self.servo_01_pub = rospy.Publisher("Topic_servo_01", Int16, queue_size=10)
        self.servo_02_pub = rospy.Publisher("Topic_servo_02", Int16, queue_size=10)

        # GUI elements for controlling servos
        S1 = Scale(frame, from_=0, to=180, label="Servo-01", command=self.pub_servo_1)
        S1.grid(row=0, column=0)
        S1.set(90)
        
        S2 = Scale(frame, from_=0, to=180, label="Servo-02", command=self.pub_servo_2)
        S2.grid(row=0, column=1)
        S2.set(90)

        # GUI labels to show the switch states
        self.switch_1_label = Label(frame, text="Switch 1: OFF", font=("Arial", 12))
        self.switch_1_label.grid(row=2, column=0)

        self.switch_2_label = Label(frame, text="Switch 2: OFF", font=("Arial", 12))
        self.switch_2_label.grid(row=2, column=1)

        # Call main after the GUI is initialized
        frame.after(100, self.main)

    def main(self):
        # ROS subscribers to listen to switch states
        rospy.Subscriber('/switch_info1', Int8, self.sub_switch_1)
        rospy.Subscriber('/switch_info2', Int8, self.sub_switch_2)
        self.rate = rospy.Rate(10)  # Faster rate for responsive GUI updates
        
        while not rospy.is_shutdown():
            frame.update()  # Update GUI
            self.rate.sleep()
            
    def pub_servo_1(self, val1):
        servo_angle = int(val1)
        self.servo_01_pub.publish(servo_angle)
        
    def pub_servo_2(self, val2):
        servo_angle = int(val2)
        self.servo_02_pub.publish(servo_angle)

    def sub_switch_1(self, data1):
        # Update the label based on the switch state
        if data1.data == 0: # Active low in switch
            self.switch_1_label.config(text="Switch 1: ON")
        else:
            self.switch_1_label.config(text="Switch 1: OFF")

    def sub_switch_2(self, data2):
        # Update the label based on the switch state
        if data2.data == 0:
            self.switch_2_label.config(text="Switch 2: ON")
        else:
            self.switch_2_label.config(text="Switch 2: OFF")

if __name__ == '__main__':
    control = Gui()
    frame.mainloop()  # Start the Tkinter main loop
