#!/usr/bin/env python
import rospy
import numpy as np
from imu_driver.srv import convert, convertResponse

# Convert from euler to quaternion
def service_convert(req):
    roll = req.r * np.pi / 180
    pitch = req.p * np.pi / 180
    yaw = req.y * np.pi / 180
    qx = (np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)) - (np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
    qy = (np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)) + (np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2))
    qz = (np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)) - (np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2))
    qw = (np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)) + (np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
    return convertResponse(qx, qy, qz, qw)

def e_to_q_server():
    rospy.init_node('e_to_q_server')
    _ = rospy.Service('euler_to_quaternion', convert, service_convert)
    print("Ready to convert")
    rospy.spin()

if __name__ == "__main__":
    e_to_q_server()
