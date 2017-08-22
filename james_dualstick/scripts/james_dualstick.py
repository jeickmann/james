#!/usr/bin/env python
import rospy
import serial
import struct
from sensor_msgs.msg import Joy
 
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f, %f", data.axes[1], data.axes[5])
    ser.write('#')
    ser.write(struct.pack('ff', data.axes[5], -data.axes[1]))     # write a string
         
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('james_dualstick', anonymous=True)
    
    rospy.Subscriber("joy", Joy, callback, None, 1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
     
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 115200)  # open serial port
    print(ser.name)         # check which port was really used
    listener()